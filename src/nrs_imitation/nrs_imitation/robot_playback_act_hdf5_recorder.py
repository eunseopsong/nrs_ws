#!/usr/bin/env python3
# ============================================================
# robot_playback_act_hdf5_recorder.py  (Stage-2 Robot Playback)
#
# Keyboard:
#   s : start episode
#   e : end episode (save)
#   d : discard last episode (delete)
#   u : undo last discard (restore)
#   q : quit
#
# Save structure:
#   /episodes/ep_xxxx/{position, ft, images/top, images/ee}
# ============================================================

import os
import sys
import time
import threading
from typing import Optional, List, Tuple, Dict

import numpy as np
import h5py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image


# ===========================
# Image utilities
# ===========================
def _image_to_rgb_numpy(msg: Image) -> Optional[np.ndarray]:
    enc = (msg.encoding or "").lower()
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    if h <= 0 or w <= 0:
        return None

    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if buf.size < h * step:
        return None
    row = buf[: h * step].reshape(h, step)

    if enc in ("rgb8", "bgr8"):
        img = row[:, : w * 3].reshape(h, w, 3)
        return img[:, :, ::-1].copy() if enc == "bgr8" else img.copy()

    if enc in ("rgba8", "bgra8"):
        img = row[:, : w * 4].reshape(h, w, 4)[:, :, :3]
        return img[:, :, ::-1].copy() if enc == "bgra8" else img.copy()

    return None


def _now_stamp_YYYYMMDDHHMM():
    return time.strftime("%Y%m%d%H%M", time.localtime())


def _pick_image_shape(frames: List[np.ndarray]) -> Tuple[int, int]:
    for im in frames:
        if im is not None and im.ndim == 3 and im.shape[0] > 1 and im.shape[1] > 1:
            return im.shape[0], im.shape[1]
    return 1, 1


def _stack_fix(frames: List[np.ndarray], H: int, W: int) -> np.ndarray:
    T = len(frames)
    out = np.zeros((T, H, W, 3), dtype=np.uint8)
    last = np.zeros((H, W, 3), dtype=np.uint8)
    for i, im in enumerate(frames):
        if im is not None and im.shape[:2] == (H, W):
            out[i] = im
            last = im
        else:
            out[i] = last
    return out


# ===========================
# ROS2 Node
# ===========================
class RobotPlaybackACTHDF5Recorder(Node):
    def __init__(self):
        super().__init__("robot_playback_act_hdf5_recorder")

        # ---------- parameters ----------
        self.declare_parameter("save_root_dir", "/home/eunseop/nrs_lab2/datasets/ACT/merged_hdf5")
        self.declare_parameter("position_topic", "/ur10skku/currentP")
        self.declare_parameter("ft_topic", "/ur10skku/currentF")
        self.declare_parameter("top_image_topic", "/realsense/top/color/image_raw")
        self.declare_parameter("ee_image_topic", "/realsense/ee/color/image_raw")
        self.declare_parameter("sample_hz", 30.0)

        self.save_root_dir = self.get_parameter("save_root_dir").value
        self.position_topic = self.get_parameter("position_topic").value
        self.ft_topic = self.get_parameter("ft_topic").value
        self.top_image_topic = self.get_parameter("top_image_topic").value
        self.ee_image_topic = self.get_parameter("ee_image_topic").value
        self.sample_hz = float(self.get_parameter("sample_hz").value)

        # ---------- HDF5 ----------
        os.makedirs(self.save_root_dir, exist_ok=True)
        self.save_stamp = _now_stamp_YYYYMMDDHHMM()
        self.save_path = os.path.join(self.save_root_dir, self.save_stamp)
        self.h5 = h5py.File(self.save_path, "a")

        if "episodes" not in self.h5:
            self.h5.create_group("episodes")

        self.ep_idx = self._next_ep_index()

        # ---------- runtime ----------
        self.lock = threading.Lock()
        self.recording = False

        self.latest_pos = None
        self.latest_ft = None
        self.latest_top = None
        self.latest_ee = None

        self.buf_pos, self.buf_ft = [], []
        self.buf_top, self.buf_ee = [], []

        self.last_saved_ep: Optional[str] = None
        self.undo_buffer: Optional[Dict] = None

        self._cmd_queue = []

        # ---------- subscribers ----------
        self.create_subscription(Float64MultiArray, self.position_topic, self._pos_cb, 10)
        self.create_subscription(Float64MultiArray, self.ft_topic, self._ft_cb, 10)
        self.create_subscription(Image, self.top_image_topic, self._top_img_cb, 10)
        self.create_subscription(Image, self.ee_image_topic, self._ee_img_cb, 10)

        # ---------- timers ----------
        self.create_timer(1.0 / self.sample_hz, self._tick)
        self.create_timer(2.0, self._status)

        threading.Thread(target=self._keyboard_loop, daemon=True).start()

        self.get_logger().info("Recorder ready | s=start e=end d=discard u=undo q=quit")

    # ===========================
    # callbacks
    # ===========================
    def _pos_cb(self, msg):
        if len(msg.data) >= 6:
            self.latest_pos = np.array(msg.data[:6], dtype=np.float32)

    def _ft_cb(self, msg):
        if len(msg.data) >= 3:
            self.latest_ft = np.array(msg.data[:3], dtype=np.float32)

    def _top_img_cb(self, msg):
        self.latest_top = _image_to_rgb_numpy(msg)

    def _ee_img_cb(self, msg):
        self.latest_ee = _image_to_rgb_numpy(msg)

    # ===========================
    # keyboard
    # ===========================
    def _keyboard_loop(self):
        while rclpy.ok():
            c = sys.stdin.readline().strip().lower()
            if c:
                with self.lock:
                    self._cmd_queue.append(c[0])

    # ===========================
    # timers
    # ===========================
    def _status(self):
        self.get_logger().info(
            f"[STATUS] recording={self.recording} ep_idx={self.ep_idx} steps={len(self.buf_pos)}"
        )

    def _tick(self):
        with self.lock:
            cmds = self._cmd_queue[:]
            self._cmd_queue.clear()

        for c in cmds:
            if c == "s":
                self._start()
            elif c == "e":
                self._end(save=True)
            elif c == "d":
                self._discard()
            elif c == "u":
                self._undo()
            elif c == "q":
                if self.recording:
                    self._end(save=True)
                rclpy.shutdown()
                return

        if not self.recording:
            return

        if self.latest_pos is None or self.latest_ft is None:
            return

        self.buf_pos.append(self.latest_pos.copy())
        self.buf_ft.append(self.latest_ft.copy())
        self.buf_top.append(self.latest_top)
        self.buf_ee.append(self.latest_ee)

    # ===========================
    # episode control
    # ===========================
    def _start(self):
        if self.recording:
            return
        self.recording = True
        self.buf_pos.clear()
        self.buf_ft.clear()
        self.buf_top.clear()
        self.buf_ee.clear()
        self.get_logger().info(f"=== START ep_{self.ep_idx:04d}")

    def _end(self, save=True):
        self.recording = False
        if save:
            self._save()

    def _save(self):
        ep = f"ep_{self.ep_idx:04d}"
        pos = np.asarray(self.buf_pos)
        ft = np.asarray(self.buf_ft)

        Ht, Wt = _pick_image_shape(self.buf_top)
        He, We = _pick_image_shape(self.buf_ee)

        top = _stack_fix(self.buf_top, Ht, Wt)
        ee = _stack_fix(self.buf_ee, He, We)

        g = self.h5["episodes"].create_group(ep)
        g.create_dataset("position", data=pos)
        g.create_dataset("ft", data=ft)
        ig = g.create_group("images")
        ig.create_dataset("top", data=top, compression="gzip", compression_opts=4)
        ig.create_dataset("ee", data=ee, compression="gzip", compression_opts=4)

        self.h5.flush()
        self.last_saved_ep = ep
        self.undo_buffer = None
        self.ep_idx += 1

        self.get_logger().info(f"[SAVE] {ep}")

    def _discard(self):
        if self.recording:
            self.recording = False
            self.buf_pos.clear()
            self.buf_ft.clear()
            self.buf_top.clear()
            self.buf_ee.clear()
            self.get_logger().warn("Recording discarded (not saved)")
            return

        if self.last_saved_ep is None:
            self.get_logger().warn("No episode to discard")
            return

        ep = self.last_saved_ep
        grp = self.h5["episodes"][ep]

        self.undo_buffer = {
            "name": ep,
            "position": grp["position"][()],
            "ft": grp["ft"][()],
            "top": grp["images/top"][()],
            "ee": grp["images/ee"][()],
        }

        del self.h5["episodes"][ep]
        self.h5.flush()
        self.last_saved_ep = None

        self.get_logger().warn(f"[DELETE] {ep} (undo available)")

    def _undo(self):
        if self.undo_buffer is None:
            self.get_logger().warn("Nothing to undo")
            return

        ep = self.undo_buffer["name"]
        g = self.h5["episodes"].create_group(ep)
        g.create_dataset("position", data=self.undo_buffer["position"])
        g.create_dataset("ft", data=self.undo_buffer["ft"])
        ig = g.create_group("images")
        ig.create_dataset("top", data=self.undo_buffer["top"], compression="gzip", compression_opts=4)
        ig.create_dataset("ee", data=self.undo_buffer["ee"], compression="gzip", compression_opts=4)

        self.h5.flush()
        self.last_saved_ep = ep
        self.undo_buffer = None

        self.get_logger().info(f"[UNDO] Restored {ep}")

    # ===========================
    def _next_ep_index(self):
        idx = -1
        for k in self.h5["episodes"].keys():
            if k.startswith("ep_"):
                idx = max(idx, int(k.split("_")[1]))
        return idx + 1


def main():
    rclpy.init()
    node = RobotPlaybackACTHDF5Recorder()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
