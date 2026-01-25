#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import time
import threading
import subprocess
from dataclasses import dataclass
from typing import Optional, List

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Float64MultiArray


# =========================
# (옵션) SciPy 있으면 Whittaker(CG) 사용
# =========================
_HAS_SCIPY = False
try:
    import scipy.sparse as sp
    import scipy.sparse.linalg as spla
    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False


@dataclass
class TrajSession:
    traj_id: str
    hz: float
    expected_n: int
    t_start: float
    end_received: bool = False
    t_last_point: float = 0.0


def _parse_start(s: str):
    """
    START id=xxxx hz=125.000000 N=4000
    """
    m = re.search(r"START\s+id=([0-9a-fA-F]+)\s+hz=([0-9eE\.\+\-]+)\s+N=(\d+)", s)
    if not m:
        return None
    return m.group(1), float(m.group(2)), int(m.group(3))


def _parse_end(s: str):
    """
    END id=xxxx N=4000
    """
    m = re.search(r"END\s+id=([0-9a-fA-F]+)\s+N=(\d+)", s)
    if not m:
        return None
    return m.group(1), int(m.group(2))


def hampel_filter_1d(x: np.ndarray, k: int = 7, nsig: float = 3.0) -> np.ndarray:
    """
    Hampel filter (median + MAD) for outlier suppression.
    """
    x = x.astype(np.float64, copy=True)
    n = x.shape[0]
    if n < 2 * k + 1:
        return x
    y = x.copy()
    for i in range(n):
        i0 = max(0, i - k)
        i1 = min(n, i + k + 1)
        w = x[i0:i1]
        med = np.median(w)
        mad = np.median(np.abs(w - med)) + 1e-12
        sigma = 1.4826 * mad
        if np.abs(x[i] - med) > nsig * sigma:
            y[i] = med
    return y


def whittaker_smooth(y: np.ndarray, lam: float) -> np.ndarray:
    """
    Solve: (I + lam * D2^T D2) z = y
    D2: second difference operator.
    Uses SciPy sparse CG if available; otherwise fallback to moving average.
    """
    y = np.asarray(y, dtype=np.float64)
    n = y.shape[0]
    if n < 5:
        return y.copy()

    if _HAS_SCIPY:
        # D2: (n-2) x n
        D = sp.diags([1.0, -2.0, 1.0], [0, 1, 2], shape=(n - 2, n), format="csr")
        A = sp.eye(n, format="csr") + (lam * (D.T @ D))

        z = np.zeros_like(y)
        # CG per column
        if y.ndim == 1:
            sol, info = spla.cg(A, y, maxiter=2000, atol=1e-10, tol=1e-10)
            if info != 0:
                # fallback
                return moving_average(y, win=9)
            return sol
        else:
            for d in range(y.shape[1]):
                sol, info = spla.cg(A, y[:, d], maxiter=2000, atol=1e-10, tol=1e-10)
                if info != 0:
                    z[:, d] = moving_average(y[:, d], win=9)
                else:
                    z[:, d] = sol
            return z

    # fallback: simple moving average
    if y.ndim == 1:
        return moving_average(y, win=9)
    z = np.zeros_like(y)
    for d in range(y.shape[1]):
        z[:, d] = moving_average(y[:, d], win=9)
    return z


def moving_average(x: np.ndarray, win: int = 9) -> np.ndarray:
    x = np.asarray(x, dtype=np.float64)
    if win <= 1 or x.shape[0] < win:
        return x.copy()
    w = np.ones(win, dtype=np.float64) / float(win)
    pad = win // 2
    xp = np.pad(x, (pad, pad), mode="edge")
    return np.convolve(xp, w, mode="valid")


def retime_uniform(traj: np.ndarray, in_hz: float, time_scale: float) -> np.ndarray:
    """
    Uniform resampling by time_scale.
    - time_scale=1.0 -> unchanged
    - time_scale=8.0 -> ~8x more samples
    """
    if time_scale <= 1.0:
        return traj.copy()

    n = traj.shape[0]
    t_in = np.arange(n, dtype=np.float64) / float(in_hz)
    out_n = int(np.floor(n * time_scale))
    if out_n < 2:
        return traj.copy()
    t_out = np.arange(out_n, dtype=np.float64) / float(in_hz * time_scale)

    out = np.zeros((out_n, traj.shape[1]), dtype=np.float64)
    for d in range(traj.shape[1]):
        out[:, d] = np.interp(t_out, t_in, traj[:, d])
    return out


class ActTxtPostprocessor(Node):
    def __init__(self):
        super().__init__("act_txt_postprocessor")

        # ---------------- params
        self.declare_parameter("traj_state_topic", "/act_infer/traj_state")
        self.declare_parameter("traj_point_topic", "/act_infer/traj_point")

        self.declare_parameter("record_hz", 125.0)

        self.declare_parameter("save_path", "/tmp/cmd_continue9D.txt")
        self.declare_parameter("transfer_enable", False)
        self.declare_parameter("remote_user", "user")
        self.declare_parameter("remote_ip", "127.0.0.1")
        self.declare_parameter("remote_dir", "/tmp")

        # 수신 안정화 파라미터
        self.declare_parameter("point_qos_depth", 20000)           # ★ 충분히 크게
        self.declare_parameter("state_qos_depth", 50)
        self.declare_parameter("inactivity_timeout_sec", 0.5)      # END 후, 마지막 point 이후 이만큼 조용하면 finalize
        self.declare_parameter("wait_all_points_sec", 5.0)         # END를 받았는데도 N이 안 채워지면 최대 대기

        # 후처리 파라미터(기본값은 네 로그 스타일과 유사하게)
        self.declare_parameter("time_scale", 1.0)                  # 필요하면 8.0
        self.declare_parameter("hampel_k", 7)
        self.declare_parameter("hampel_nsig", 3.0)
        self.declare_parameter("lam_pos", 2000.0)
        self.declare_parameter("lam_ang", 200.0)

        self.state_topic = str(self.get_parameter("traj_state_topic").value)
        self.point_topic = str(self.get_parameter("traj_point_topic").value)

        self.record_hz = float(self.get_parameter("record_hz").value)

        self.save_path = str(self.get_parameter("save_path").value)
        self.transfer_enable = bool(self.get_parameter("transfer_enable").value)
        self.remote_user = str(self.get_parameter("remote_user").value)
        self.remote_ip = str(self.get_parameter("remote_ip").value)
        self.remote_dir = str(self.get_parameter("remote_dir").value)

        self.point_qos_depth = int(self.get_parameter("point_qos_depth").value)
        self.state_qos_depth = int(self.get_parameter("state_qos_depth").value)
        self.inactivity_timeout_sec = float(self.get_parameter("inactivity_timeout_sec").value)
        self.wait_all_points_sec = float(self.get_parameter("wait_all_points_sec").value)

        self.time_scale = float(self.get_parameter("time_scale").value)
        self.hampel_k = int(self.get_parameter("hampel_k").value)
        self.hampel_nsig = float(self.get_parameter("hampel_nsig").value)
        self.lam_pos = float(self.get_parameter("lam_pos").value)
        self.lam_ang = float(self.get_parameter("lam_ang").value)

        # ---------------- session state
        self._lock = threading.Lock()
        self._session: Optional[TrajSession] = None
        self._buf: List[np.ndarray] = []
        self._processing = False
        self._t_end_received = 0.0

        # ---------------- QoS (핵심)
        qos_state = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=max(10, self.state_qos_depth),
        )
        qos_point = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=max(1000, self.point_qos_depth),
        )

        self.sub_state = self.create_subscription(String, self.state_topic, self._cb_state, qos_state)
        self.sub_point = self.create_subscription(Float64MultiArray, self.point_topic, self._cb_point, qos_point)

        # timer to finalize safely (drain)
        self.timer = self.create_timer(0.02, self._tick)  # 50 Hz

        self.get_logger().info(f"Initialized ActTxtPostprocessor.")
        self.get_logger().info(f"Listening: {self.state_topic} (String), {self.point_topic} (Float64MultiArray size=9)")
        self.get_logger().info(f"QoS: point depth={qos_point.depth}, RELIABLE")
        self.get_logger().info(f"Save path: {self.save_path}")
        self.get_logger().info(f"SCP: enable={self.transfer_enable}, dst={self.remote_user}@{self.remote_ip}:{self.remote_dir}")

    # ---------------- callbacks
    def _cb_state(self, msg: String):
        s = (msg.data or "").strip()

        st = _parse_start(s)
        if st is not None:
            traj_id, hz, expected_n = st
            with self._lock:
                self._session = TrajSession(traj_id=traj_id, hz=hz, expected_n=expected_n, t_start=time.time())
                self._buf = []
                self._processing = False
                self._t_end_received = 0.0
            self.get_logger().info(f"[START] id={traj_id} hz={hz} expected_n={expected_n}")
            return

        ed = _parse_end(s)
        if ed is not None:
            traj_id, expected_n = ed
            with self._lock:
                if self._session is None:
                    self.get_logger().warn(f"[END] received but no active session. ignore.")
                    return
                if traj_id != self._session.traj_id:
                    self.get_logger().warn(f"[END] id mismatch: got={traj_id}, active={self._session.traj_id}. ignore.")
                    return
                self._session.end_received = True
                self._t_end_received = time.time()
            self.get_logger().info(f"[END] id={traj_id} expected_n={expected_n} (will finalize after drain)")
            return

    def _cb_point(self, msg: Float64MultiArray):
        data = msg.data
        if data is None or len(data) < 9:
            return
        p = np.array(data[:9], dtype=np.float64)

        with self._lock:
            if self._session is None:
                return
            if self._processing:
                # 처리 중에는 추가 수신 무시(혹은 별도 버퍼링하고 싶으면 여기서 append)
                return
            self._buf.append(p)
            self._session.t_last_point = time.time()

    # ---------------- timer tick: finalize condition
    def _tick(self):
        with self._lock:
            sess = self._session
            if sess is None or self._processing is True:
                return
            if not sess.end_received:
                return

            n = len(self._buf)
            now = time.time()
            quiet = (now - sess.t_last_point) if sess.t_last_point > 0 else 1e9
            since_end = now - self._t_end_received if self._t_end_received > 0 else 1e9

            # 조건 A: 기대 N(=4000) 채움
            if n >= sess.expected_n and sess.expected_n > 0:
                self._processing = True
                buf_copy = np.stack(self._buf, axis=0)
                sess_copy = TrajSession(**sess.__dict__)
                self.get_logger().info(f"[FINALIZE] got all points: {n}/{sess.expected_n}")
                threading.Thread(target=self._process_and_save, args=(sess_copy, buf_copy), daemon=True).start()
                return

            # 조건 B: END 이후 너무 오래 기다림
            if since_end >= self.wait_all_points_sec:
                self._processing = True
                buf_copy = np.stack(self._buf, axis=0) if n > 0 else np.zeros((0, 9), dtype=np.float64)
                sess_copy = TrajSession(**sess.__dict__)
                self.get_logger().warn(f"[FINALIZE] timeout waiting all points: got {n}/{sess.expected_n} (since_end={since_end:.2f}s)")
                threading.Thread(target=self._process_and_save, args=(sess_copy, buf_copy), daemon=True).start()
                return

            # 조건 C: 마지막 point 이후 잠잠하면(드레인 끝)
            if quiet >= self.inactivity_timeout_sec and n > 0:
                self._processing = True
                buf_copy = np.stack(self._buf, axis=0)
                sess_copy = TrajSession(**sess.__dict__)
                self.get_logger().info(f"[FINALIZE] inactivity drain: got {n}/{sess.expected_n}, quiet={quiet:.3f}s")
                threading.Thread(target=self._process_and_save, args=(sess_copy, buf_copy), daemon=True).start()
                return

    # ---------------- processing
    def _process_and_save(self, sess: TrajSession, traj_raw: np.ndarray):
        """
        traj_raw: (N,9)  [x y z wx wy wz fx fy fz]
        """
        try:
            if traj_raw.shape[0] == 0:
                self.get_logger().error("[PROCESS] empty trajectory. abort.")
                self._reset_session()
                return

            n_raw = traj_raw.shape[0]
            hz_in = sess.hz if sess.hz > 0 else self.record_hz

            # --------- Hampel (outliers)
            traj = traj_raw.copy()
            for d in range(9):
                traj[:, d] = hampel_filter_1d(traj[:, d], k=self.hampel_k, nsig=self.hampel_nsig)

            # --------- Whittaker smooth for pose only (0:6)
            pose = traj[:, :6].copy()
            pos = pose[:, :3]
            ang = pose[:, 3:6]

            pos_s = whittaker_smooth(pos, lam=self.lam_pos)
            ang_s = whittaker_smooth(ang, lam=self.lam_ang)

            pose_s = np.concatenate([pos_s, ang_s], axis=1)
            traj[:, :6] = pose_s

            # --------- retime (optional)
            if self.time_scale > 1.0:
                traj_out = retime_uniform(traj, in_hz=hz_in, time_scale=self.time_scale)
            else:
                traj_out = traj.copy()

            n_out = traj_out.shape[0]

            # --------- save
            os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
            with open(self.save_path, "w") as f:
                for i in range(n_out):
                    row = traj_out[i, :]
                    f.write("\t".join(f"{v:.6f}" for v in row.tolist()) + "\n")

            self.get_logger().info(
                f"[SAVE] local file: {self.save_path} (raw_rows={n_raw} -> out_rows={n_out}, hz_in={hz_in:.3f}, time_scale={self.time_scale})"
            )

            # --------- scp
            if self.transfer_enable:
                self._scp_to_remote(self.save_path)

        except Exception as e:
            self.get_logger().error(f"[PROCESS] failed: {e}")
        finally:
            self._reset_session()

    def _scp_to_remote(self, local_path: str):
        # remote_dir 끝에 / 없으면 붙이기
        rdir = self.remote_dir
        if not rdir.endswith("/"):
            rdir += "/"

        dst = f"{self.remote_user}@{self.remote_ip}:{rdir}"
        cmd = [
            "scp",
            "-o", "StrictHostKeyChecking=no",
            "-o", "UserKnownHostsFile=/dev/null",
            local_path,
            dst
        ]
        self.get_logger().info(f"[SCP] sending -> {dst}")
        try:
            p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            if p.returncode != 0:
                self.get_logger().error(f"[SCP] failed (code={p.returncode}): {p.stderr.strip()}")
            else:
                self.get_logger().info(f"[SCP] SUCCESS: File transferred to control PC ({dst})")
        except Exception as e:
            self.get_logger().error(f"[SCP] exception: {e}")

    def _reset_session(self):
        with self._lock:
            self._session = None
            self._buf = []
            self._processing = False
            self._t_end_received = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = ActTxtPostprocessor()

    # MultiThreadedExecutor로 callback 밀림 최소화
    exec_ = MultiThreadedExecutor(num_threads=2)
    exec_.add_node(node)

    try:
        exec_.spin()
    except KeyboardInterrupt:
        pass
    finally:
        exec_.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
