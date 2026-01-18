#!/usr/bin/env python3
# ============================================================
# vr_demo_hdf5_episode_pusher.py
#
# Stage-1 (Human Demo) HDF5 -> (one episode) TXT -> send to B PC
#
# Key mapping (requested):
#   [Enter] : push CURRENT idx episode
#   d       : idx + 1  (move)  + show status
#   a       : idx - 1  (move)  + show status
#   r       : re-push CURRENT idx
#   g <idx> : go to idx (0-based)
#   l       : list episodes
#   s       : show status
#   h       : help
#   q       : quit
#
# Notes:
# - This node DOES NOT subscribe/publish ROS topics. ROS2 node is used only for logging & packaging.
# - It reads HDF5, writes /tmp/cmd_continue9D.txt, and scp to B PC target path.
# ============================================================

import os
import sys
import time
import shlex
import threading
import subprocess
from typing import List, Tuple

import numpy as np

import rclpy
from rclpy.node import Node


class VRDemoHDF5EpisodePusher(Node):
    def __init__(self):
        super().__init__('vr_demo_hdf5_episode_pusher')

        # =========================
        # Parameters
        # =========================
        self.declare_parameter(
            'hdf5_path',
            '/home/eunseop/dev_ws/src/y2_ur10skku_control/Y2RobMotion/datasets/vr_demo_stage1.hdf5'
        )
        self.declare_parameter('episodes_group', 'episodes')
        self.declare_parameter('traj_dataset', 'traj')

        # local txt path (A PC)
        self.declare_parameter('local_txt_path', '/tmp/cmd_continue9D.txt')

        # remote (B PC)
        self.declare_parameter('remote_user', 'nrs_forcecon')
        self.declare_parameter('remote_ip', '192.168.0.151')
        self.declare_parameter(
            'remote_txt_path',
            '/home/nrs_forcecon/dev_ws/src/y2_ur10skku_control/Y2RobMotion/txtcmd/cmd_continue9D.txt'
        )

        # txt formatting
        self.declare_parameter('txt_fmt', '%.10f')   # float format
        self.declare_parameter('use_atomic_remote_replace', True)  # upload to tmp then mv on B PC

        # =========================
        # Load params
        # =========================
        self.hdf5_path = str(self.get_parameter('hdf5_path').value)
        self.episodes_group = str(self.get_parameter('episodes_group').value)
        self.traj_dataset = str(self.get_parameter('traj_dataset').value)

        self.local_txt_path = str(self.get_parameter('local_txt_path').value)

        self.remote_user = str(self.get_parameter('remote_user').value)
        self.remote_ip = str(self.get_parameter('remote_ip').value)
        self.remote_txt_path = str(self.get_parameter('remote_txt_path').value)

        self.txt_fmt = str(self.get_parameter('txt_fmt').value)
        self.use_atomic_remote_replace = bool(self.get_parameter('use_atomic_remote_replace').value)

        # =========================
        # Load episodes list
        # =========================
        self._require_h5py()

        import h5py
        if not os.path.exists(self.hdf5_path):
            raise FileNotFoundError(f"HDF5 not found: {self.hdf5_path}")

        with h5py.File(self.hdf5_path, 'r') as f:
            if self.episodes_group not in f:
                raise KeyError(f"Group '{self.episodes_group}' not found in HDF5.")
            keys = list(f[self.episodes_group].keys())

        # keep only ep_* and sort
        self.episodes = sorted([k for k in keys if k.startswith('ep_')])
        if len(self.episodes) == 0:
            raise RuntimeError("No episodes found (expected keys like ep_0000).")

        self.num_eps = len(self.episodes)
        self.cur_idx = 0  # start at first episode

        # keyboard thread control
        self._stop_event = threading.Event()
        self._kbd_thread = threading.Thread(target=self.keyboard_loop, daemon=True)

        # Banner
        self.get_logger().info("============================================================")
        self.get_logger().info("VRDemoHDF5EpisodePusher initialized (Stage-1 -> TXT -> B PC)")
        self.get_logger().info(f"  HDF5:   {self.hdf5_path}")
        self.get_logger().info(f"  Group:  {self.episodes_group} , traj: {self.traj_dataset}")
        self.get_logger().info(f"  Episodes: {self.num_eps}  (0..{self.num_eps - 1})")
        self.get_logger().info(f"  Local TXT:  {self.local_txt_path}")
        self.get_logger().info(f"  Remote: {self.remote_user}@{self.remote_ip}:{self.remote_txt_path}")
        self.get_logger().info("============================================================")
        self.print_help()
        self.get_logger().info(self.status_line())

        self._kbd_thread.start()

    # -------------------------
    # Helpers
    # -------------------------
    @staticmethod
    def _require_h5py():
        try:
            import h5py  # noqa: F401
        except Exception as e:
            raise RuntimeError(
                "h5py is required. Install: sudo apt-get install python3-h5py  (or pip install h5py)"
            ) from e

    def ep_name(self, idx: int) -> str:
        idx = int(np.clip(idx, 0, self.num_eps - 1))
        return self.episodes[idx]

    def status_line(self) -> str:
        # Always show total + current idx + ep name
        return f"[STATUS] total={self.num_eps} | cur_idx={self.cur_idx} | cur_ep={self.ep_name(self.cur_idx)}"

    def print_help(self):
        self.get_logger().info("")
        self.get_logger().info("================= Keyboard Commands =================")
        self.get_logger().info("  [Enter]         : push CURRENT idx episode (send TXT)")
        self.get_logger().info("  d               : idx + 1 (move) ")
        self.get_logger().info("  a               : idx - 1 (move) ")
        self.get_logger().info("  r               : re-push CURRENT idx episode")
        self.get_logger().info("  g <idx>         : go to idx (0-based)")
        self.get_logger().info("  l               : list episodes")
        self.get_logger().info("  s               : show status")
        self.get_logger().info("  h               : help")
        self.get_logger().info("  q               : quit")
        self.get_logger().info("=====================================================")

    def list_episodes(self, max_show: int = 50):
        self.get_logger().info(f"[LIST] total={self.num_eps}")
        if self.num_eps <= max_show:
            for i, k in enumerate(self.episodes):
                mark = "<--" if i == self.cur_idx else ""
                self.get_logger().info(f"  {i:4d}: {k} {mark}")
        else:
            # show head/tail
            head_n = max_show // 2
            tail_n = max_show - head_n
            for i in range(head_n):
                k = self.episodes[i]
                mark = "<--" if i == self.cur_idx else ""
                self.get_logger().info(f"  {i:4d}: {k} {mark}")
            self.get_logger().info("  ...")
            for i in range(self.num_eps - tail_n, self.num_eps):
                k = self.episodes[i]
                mark = "<--" if i == self.cur_idx else ""
                self.get_logger().info(f"  {i:4d}: {k} {mark}")

    # -------------------------
    # Core: push current episode
    # -------------------------
    def push_current(self):
        import h5py

        idx = self.cur_idx
        ep = self.ep_name(idx)

        t0 = time.time()
        self.get_logger().info(f"[PUSH] idx={idx} ({ep}) loading...")

        with h5py.File(self.hdf5_path, 'r') as f:
            grp_path = f"{self.episodes_group}/{ep}"
            if grp_path not in f:
                raise KeyError(f"Episode group not found: {grp_path}")
            g = f[grp_path]
            if self.traj_dataset not in g:
                raise KeyError(f"Dataset '{self.traj_dataset}' not found in {grp_path}")
            traj = np.array(g[self.traj_dataset], dtype=np.float64)

        if traj.ndim != 2 or traj.shape[1] != 9:
            raise ValueError(f"traj must be (T,9). got shape={traj.shape}")

        # Save local txt
        local_dir = os.path.dirname(self.local_txt_path) or "."
        os.makedirs(local_dir, exist_ok=True)

        self.get_logger().info(f"[PUSH] traj shape={traj.shape}  saving local txt...")
        np.savetxt(self.local_txt_path, traj, fmt=self.txt_fmt)

        # Send to B PC
        self.get_logger().info("[PUSH] sending to B PC via scp/ssh...")

        if self.use_atomic_remote_replace:
            # Upload as remote tmp and then atomic mv on remote
            remote_tmp = self.remote_txt_path + ".tmp"

            # 1) scp local -> remote_tmp
            self._scp(self.local_txt_path, remote_tmp)

            # 2) remote mv tmp -> final
            self._ssh(f"mv -f {shlex.quote(remote_tmp)} {shlex.quote(self.remote_txt_path)}")
        else:
            # Direct scp overwrite
            self._scp(self.local_txt_path, self.remote_txt_path)

        dt = time.time() - t0
        self.get_logger().info(f"[DONE] idx={idx} ({ep}) pushed in {dt:.2f}s")
        self.get_logger().info(self.status_line())

    # -------------------------
    # SCP / SSH helpers
    # -------------------------
    def _scp(self, local_path: str, remote_path: str):
        # remote_path is remote absolute path
        dst = f"{self.remote_user}@{self.remote_ip}:{remote_path}"
        cmd = ["scp", local_path, dst]
        result = subprocess.run(cmd, capture_output=True, text=True)

        if result.returncode != 0:
            stderr = (result.stderr or "").strip()
            raise RuntimeError(f"SCP failed:\n{stderr}")

    def _ssh(self, remote_cmd: str):
        # run command on remote
        host = f"{self.remote_user}@{self.remote_ip}"
        cmd = ["ssh", host, remote_cmd]
        result = subprocess.run(cmd, capture_output=True, text=True)

        if result.returncode != 0:
            stderr = (result.stderr or "").strip()
            raise RuntimeError(f"SSH failed:\n{stderr}")

    # -------------------------
    # Keyboard loop
    # -------------------------
    def keyboard_loop(self):
        try:
            while not self._stop_event.is_set():
                # Read one line command
                line = sys.stdin.readline()
                if line == "":  # EOF
                    time.sleep(0.05)
                    continue

                cmd = line.strip()

                # Enter => cmd == ""
                if cmd == "":
                    # push CURRENT
                    try:
                        self.push_current()
                    except Exception as e:
                        self.get_logger().error(f"[ERR] push failed: {e}")
                    continue

                # single key
                if cmd == "q":
                    self.get_logger().info("Quit requested.")
                    self._stop_event.set()
                    rclpy.shutdown()
                    return

                if cmd == "h":
                    self.print_help()
                    self.get_logger().info(self.status_line())
                    continue

                if cmd == "s":
                    self.get_logger().info(self.status_line())
                    continue

                if cmd == "l":
                    self.list_episodes()
                    self.get_logger().info(self.status_line())
                    continue

                if cmd == "d":
                    # idx + 1
                    if self.cur_idx < self.num_eps - 1:
                        self.cur_idx += 1
                    self.get_logger().info(self.status_line())
                    continue

                if cmd == "a":
                    # idx - 1
                    if self.cur_idx > 0:
                        self.cur_idx -= 1
                    self.get_logger().info(self.status_line())
                    continue

                if cmd == "r":
                    # re-push current
                    try:
                        self.push_current()
                    except Exception as e:
                        self.get_logger().error(f"[ERR] re-push failed: {e}")
                    continue

                # g <idx>
                if cmd.startswith("g "):
                    parts = cmd.split()
                    if len(parts) >= 2:
                        try:
                            idx = int(parts[1])
                            idx = int(np.clip(idx, 0, self.num_eps - 1))
                            self.cur_idx = idx
                            self.get_logger().info(self.status_line())
                        except Exception:
                            self.get_logger().warn("Usage: g <idx>   (idx is 0-based integer)")
                    else:
                        self.get_logger().warn("Usage: g <idx>")
                    continue

                self.get_logger().warn(f"Unknown command: '{cmd}'  (press 'h' for help)")

        except Exception as e:
            self.get_logger().error(f"keyboard loop error: {e}")
            try:
                rclpy.shutdown()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = VRDemoHDF5EpisodePusher()

    # Node doesn't need spin, but keep alive until shutdown.
    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
