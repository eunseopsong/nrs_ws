#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ur10_keyboard_recorder.py  (ROS2 rclpy node)

- Subscribe:
  * pose_topic:  std_msgs/Float64MultiArray, data=[x y z rx ry rz]
  * force_topic: std_msgs/Float64MultiArray, data=[fx fy fz Mx My Mz]  -> 기록은 fx fy fz만

- Keyboard:
  * start_key: start recording (default 's')
  * stop_key : stop & save     (default 'e')
  * quit_key : quit node       (default 'q')
  (단일 키 입력, Enter 없이 동작)

- Record (timer-based, uniform dt):
  * x y z
  * vx vy vz  (pose diff / dt)
  * rx ry rz
  * rvx rvy rvz (unwrap + diff / dt)
  * fx fy fz

- Save:
  output_root_dir / run_YYYYMMDD_HHMMSS / ep_XXX_YYYYMMDD_HHMMSS /
    - xyz.txt
    - vxyz.txt
    - rpy.txt
    - vrpy.txt
    - fxyz.txt
    - plots/
        pos_xyz.png
        vel_xyz.png
        rot_rpy.png
        rotvel_rpy.png
        force_fxyz.png
"""

import os
import sys
import time
import math
import threading
import select
import termios
import tty
from datetime import datetime
from typing import Optional, List

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float64MultiArray


# ----------------------------
# Small utils
# ----------------------------
def _wrap_to_pi(d: np.ndarray) -> np.ndarray:
    """wrap each element to (-pi, pi]"""
    return (d + np.pi) % (2.0 * np.pi) - np.pi


def _ensure_dir(p: str):
    os.makedirs(p, exist_ok=True)


def _write_txt(path: str, header: str, data: np.ndarray):
    with open(path, "w") as f:
        if header:
            for line in header.strip().splitlines():
                f.write(f"# {line}\n")
        for row in data:
            f.write("\t".join(f"{v:.6f}" for v in row.tolist()) + "\n")


def _plot_xyz(t: np.ndarray, y: np.ndarray, title: str, ylabel: str, outpath: str, labels=("x", "y", "z")):
    fig = plt.figure(figsize=(14, 4))
    for i, lab in enumerate(labels):
        plt.plot(t, y[:, i], label=lab)
    plt.title(title)
    plt.xlabel("time [s]")
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(outpath, dpi=200)
    plt.close(fig)


# ----------------------------
# Keyboard reader (raw mode)
# ----------------------------
class _KeyReader:
    def __init__(self):
        self._enabled = sys.stdin.isatty()
        self._fd = None
        self._old = None
        self._stop = False
        self._last_key = None
        self._lock = threading.Lock()
        self._th = None

    def start(self):
        if not self._enabled:
            return
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        self._stop = False
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def stop(self):
        self._stop = True
        if self._enabled and self._fd is not None and self._old is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
            except Exception:
                pass

    def _loop(self):
        while not self._stop:
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if r:
                ch = sys.stdin.read(1)
                if ch:
                    with self._lock:
                        self._last_key = ch
            time.sleep(0.01)

    def pop(self) -> Optional[str]:
        with self._lock:
            k = self._last_key
            self._last_key = None
        return k


# ----------------------------
# Recorder node
# ----------------------------
class UR10KeyboardRecorder(Node):
    def __init__(self):
        super().__init__("ur10_keyboard_recorder")

        # ---- parameters ----
        self.declare_parameter("output_root_dir", os.path.expanduser("~/nrs_logs"))
        self.declare_parameter("pose_topic", "/ur10skku/currentP")
        self.declare_parameter("force_topic", "/ur10skku/currentF")
        self.declare_parameter("record_hz", 125.0)

        self.declare_parameter("start_key", "s")
        self.declare_parameter("stop_key", "e")
        self.declare_parameter("quit_key", "q")

        # optional velocity smoothing (EMA)
        self.declare_parameter("vel_ema_enable", False)
        self.declare_parameter("vel_ema_alpha", 0.25)      # 0~1 (larger = less smooth)
        self.declare_parameter("rotvel_ema_enable", False)
        self.declare_parameter("rotvel_ema_alpha", 0.25)

        self.output_root_dir = self.get_parameter("output_root_dir").get_parameter_value().string_value
        self.pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        self.force_topic = self.get_parameter("force_topic").get_parameter_value().string_value
        self.record_hz = float(self.get_parameter("record_hz").get_parameter_value().double_value)

        self.start_key = self.get_parameter("start_key").get_parameter_value().string_value[:1]
        self.stop_key = self.get_parameter("stop_key").get_parameter_value().string_value[:1]
        self.quit_key = self.get_parameter("quit_key").get_parameter_value().string_value[:1]

        self.vel_ema_enable = bool(self.get_parameter("vel_ema_enable").get_parameter_value().bool_value)
        self.vel_ema_alpha = float(self.get_parameter("vel_ema_alpha").get_parameter_value().double_value)
        self.rotvel_ema_enable = bool(self.get_parameter("rotvel_ema_enable").get_parameter_value().bool_value)
        self.rotvel_ema_alpha = float(self.get_parameter("rotvel_ema_alpha").get_parameter_value().double_value)

        # ---- run dir ----
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(self.output_root_dir, f"run_{ts}")
        _ensure_dir(self.run_dir)

        # ---- qos ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---- latest states ----
        self._have_pose = False
        self._have_force = False
        self._latest_pose = np.zeros(6, dtype=np.float64)
        self._latest_force = np.zeros(3, dtype=np.float64)

        # ---- recording buffers ----
        self._recording = False
        self._episode_idx = 0

        self._t0_wall: Optional[float] = None
        self._prev_wall: Optional[float] = None
        self._prev_pose: Optional[np.ndarray] = None
        self._prev_rpy_unwrap: Optional[np.ndarray] = None

        self._vel_ema: Optional[np.ndarray] = None
        self._rotvel_ema: Optional[np.ndarray] = None

        self._buf_xyz: List[List[float]] = []
        self._buf_vxyz: List[List[float]] = []
        self._buf_rpy: List[List[float]] = []
        self._buf_vrpy: List[List[float]] = []
        self._buf_fxyz: List[List[float]] = []

        # ---- subs ----
        self.create_subscription(Float64MultiArray, self.pose_topic, self._cb_pose, qos)
        self.create_subscription(Float64MultiArray, self.force_topic, self._cb_force, qos)

        # ---- timer ----
        dt = 1.0 / max(1e-6, self.record_hz)
        self.create_timer(dt, self._on_timer)

        # ---- keyboard ----
        self._keys = _KeyReader()
        self._keys.start()

        self.get_logger().info(f"[READY] run_dir={self.run_dir}")
        self.get_logger().info(
            f"[KEYS] start='{self.start_key}' stop='{self.stop_key}' quit='{self.quit_key}' "
            f"(press key without Enter)"
        )
        self.get_logger().info(f"[TOPICS] pose={self.pose_topic} force={self.force_topic}  record_hz={self.record_hz:.1f}")

    def destroy_node(self):
        try:
            self._keys.stop()
        except Exception:
            pass
        super().destroy_node()

    # ---- callbacks ----
    def _cb_pose(self, msg: Float64MultiArray):
        if msg.data is None or len(msg.data) < 6:
            return
        self._latest_pose = np.array(msg.data[:6], dtype=np.float64)
        self._have_pose = True

    def _cb_force(self, msg: Float64MultiArray):
        if msg.data is None or len(msg.data) < 3:
            return
        self._latest_force = np.array(msg.data[:3], dtype=np.float64)  # fx fy fz only
        self._have_force = True

    # ---- recording control ----
    def _start_recording(self):
        if not self._have_pose:
            self.get_logger().warn("[START] pose not received yet. wait pose...")
            return

        self._recording = True
        self._t0_wall = time.time()
        self._prev_wall = self._t0_wall

        self._prev_pose = self._latest_pose.copy()
        self._prev_rpy_unwrap = self._latest_pose[3:].copy()

        self._vel_ema = np.zeros(3, dtype=np.float64)
        self._rotvel_ema = np.zeros(3, dtype=np.float64)

        self._buf_xyz.clear()
        self._buf_vxyz.clear()
        self._buf_rpy.clear()
        self._buf_vrpy.clear()
        self._buf_fxyz.clear()

        # 첫 샘플: v=0 규칙 (QP-safe)
        t = 0.0
        p = self._latest_pose[:3].copy()
        r = self._latest_pose[3:].copy()
        f = self._latest_force.copy() if self._have_force else np.zeros(3, dtype=np.float64)

        self._buf_xyz.append([t, p[0], p[1], p[2]])
        self._buf_vxyz.append([t, 0.0, 0.0, 0.0])
        self._buf_rpy.append([t, r[0], r[1], r[2]])
        self._buf_vrpy.append([t, 0.0, 0.0, 0.0])
        self._buf_fxyz.append([t, f[0], f[1], f[2]])

        self.get_logger().info(f"[REC] START episode={self._episode_idx:03d}")

    def _stop_and_save(self):
        if not self._recording:
            return
        self._recording = False

        if len(self._buf_xyz) < 2:
            self.get_logger().warn("[REC] too short. nothing saved.")
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        ep_dir = os.path.join(self.run_dir, f"ep_{self._episode_idx:03d}_{ts}")
        plot_dir = os.path.join(ep_dir, "plots")
        _ensure_dir(plot_dir)

        xyz = np.array(self._buf_xyz, dtype=np.float64)
        vxyz = np.array(self._buf_vxyz, dtype=np.float64)
        rpy = np.array(self._buf_rpy, dtype=np.float64)
        vrpy = np.array(self._buf_vrpy, dtype=np.float64)
        fxyz = np.array(self._buf_fxyz, dtype=np.float64)

        # save txt
        _write_txt(os.path.join(ep_dir, "xyz.txt"),
                   "t[s] x[mm] y[mm] z[mm]", xyz)
        _write_txt(os.path.join(ep_dir, "vxyz.txt"),
                   "t[s] vx[mm/s] vy[mm/s] vz[mm/s]", vxyz)
        _write_txt(os.path.join(ep_dir, "rpy.txt"),
                   "t[s] rx[rad] ry[rad] rz[rad]", rpy)
        _write_txt(os.path.join(ep_dir, "vrpy.txt"),
                   "t[s] rvx[rad/s] rvy[rad/s] rvz[rad/s]", vrpy)
        _write_txt(os.path.join(ep_dir, "fxyz.txt"),
                   "t[s] fx[N] fy[N] fz[N]", fxyz)

        # plots
        t = xyz[:, 0]
        _plot_xyz(t, xyz[:, 1:4], "Position (x,y,z)", "mm", os.path.join(plot_dir, "pos_xyz.png"))
        _plot_xyz(t, vxyz[:, 1:4], "Velocity (vx,vy,vz)", "mm/s", os.path.join(plot_dir, "vel_xyz.png"))
        _plot_xyz(t, rpy[:, 1:4], "Rotation (rx,ry,rz)", "rad", os.path.join(plot_dir, "rot_rpy.png"), labels=("rx", "ry", "rz"))
        _plot_xyz(t, vrpy[:, 1:4], "Rot. Velocity (rvx,rvy,rvz)", "rad/s", os.path.join(plot_dir, "rotvel_rpy.png"), labels=("rvx", "rvy", "rvz"))
        _plot_xyz(t, fxyz[:, 1:4], "Forces (fx,fy,fz)", "N", os.path.join(plot_dir, "force_fxyz.png"), labels=("fx", "fy", "fz"))

        self.get_logger().info(f"[REC] SAVED -> {ep_dir}")
        self._episode_idx += 1

    # ---- timer loop ----
    def _on_timer(self):
        # keyboard events
        k = self._keys.pop()
        if k is not None:
            if k == self.start_key and not self._recording:
                self._start_recording()
            elif k == self.stop_key and self._recording:
                self._stop_and_save()
            elif k == self.quit_key:
                self.get_logger().info("[QUIT] requested.")
                # stop recording if active
                if self._recording:
                    self._stop_and_save()
                rclpy.shutdown()
                return

        if not self._recording:
            return
        if not self._have_pose:
            return

        now = time.time()
        if self._t0_wall is None:
            self._t0_wall = now
        t = now - self._t0_wall

        dt = max(1e-6, now - (self._prev_wall if self._prev_wall is not None else now))
        cur_pose = self._latest_pose.copy()
        cur_force = self._latest_force.copy() if self._have_force else np.zeros(3, dtype=np.float64)

        # velocity from pose diff
        if self._prev_pose is None or self._prev_rpy_unwrap is None:
            vpos = np.zeros(3, dtype=np.float64)
            vrot = np.zeros(3, dtype=np.float64)
            rpy_unwrap = cur_pose[3:].copy()
        else:
            vpos = (cur_pose[:3] - self._prev_pose[:3]) / dt

            # unwrap angles then diff
            dr = _wrap_to_pi(cur_pose[3:] - self._prev_pose[3:])
            rpy_unwrap = self._prev_rpy_unwrap + dr
            vrot = (rpy_unwrap - self._prev_rpy_unwrap) / dt  # == dr/dt

        # optional EMA smoothing (helps noisy derivative)
        if self.vel_ema_enable:
            a = float(np.clip(self.vel_ema_alpha, 1e-6, 1.0))
            self._vel_ema = a * vpos + (1.0 - a) * self._vel_ema
            vpos_use = self._vel_ema.copy()
        else:
            vpos_use = vpos

        if self.rotvel_ema_enable:
            a = float(np.clip(self.rotvel_ema_alpha, 1e-6, 1.0))
            self._rotvel_ema = a * vrot + (1.0 - a) * self._rotvel_ema
            vrot_use = self._rotvel_ema.copy()
        else:
            vrot_use = vrot

        # append
        self._buf_xyz.append([t, cur_pose[0], cur_pose[1], cur_pose[2]])
        self._buf_vxyz.append([t, vpos_use[0], vpos_use[1], vpos_use[2]])
        self._buf_rpy.append([t, cur_pose[3], cur_pose[4], cur_pose[5]])
        self._buf_vrpy.append([t, vrot_use[0], vrot_use[1], vrot_use[2]])
        self._buf_fxyz.append([t, cur_force[0], cur_force[1], cur_force[2]])

        # update prev
        self._prev_wall = now
        self._prev_pose = cur_pose
        self._prev_rpy_unwrap = rpy_unwrap


def main(args=None):
    rclpy.init(args=args)
    node = UR10KeyboardRecorder()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
