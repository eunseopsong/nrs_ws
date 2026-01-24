#!/usr/bin/env python3
# ============================================================
# robot_playback_act_hdf5_recorder.py  (Stage-2 Robot Playback)
#
# Keyboard:
#   s : start episode (begin recording)
#   e : end episode (save)
#   q : quit (if recording, auto-save then quit)
#
# Save structure (ACT-like, but "position" instead of "joints"):
#   /meta/...
#   /episodes/ep_0000/position        (T,6) float32   # [x y z wx wy wz] from /ur10skku/currentP
#   /episodes/ep_0000/ft              (T,3) float32
#   /episodes/ep_0000/images/top      (T,H,W,3) uint8
#   /episodes/ep_0000/images/ee       (T,H,W,3) uint8
#
# NOTE: timestamps group is NOT saved (removed as requested)
#
# Default save path:
#   /home/eunseop/nrs_lab2/datasets/ACT/merged_hdf5/YYYYMMDDHHMM
# ============================================================

import os
import sys
import time
import threading
from typing import Optional, List, Tuple

import numpy as np
import h5py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image


# ---------------------------
# Image utilities (no cv_bridge)
# ---------------------------
def _image_to_rgb_numpy(msg: Image) -> Optional[np.ndarray]:
    """
    Convert sensor_msgs/Image to (H,W,3) uint8 RGB array.
    Supports: rgb8, bgr8, rgba8, bgra8.
    Returns None if unsupported.
    """
    enc = (msg.encoding or "").lower()
    h = int(msg.height)
    w = int(msg.width)
    step = int(msg.step)

    if h <= 0 or w <= 0:
        return None

    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if buf.size < h * step:
        return None

    # Row-wise view (handles potential padding via step)
    row = buf[: h * step].reshape(h, step)

    if enc in ("rgb8", "bgr8"):
        need = w * 3
        if step < need:
            return None
        img = row[:, :need].reshape(h, w, 3)
        if enc == "bgr8":
            img = img[:, :, ::-1]  # BGR -> RGB
        return img.copy()

    if enc in ("rgba8", "bgra8"):
        need = w * 4
        if step < need:
            return None
        img = row[:, :need].reshape(h, w, 4)
        img = img[:, :, :3]  # drop alpha
        if enc == "bgra8":
            img = img[:, :, ::-1]  # BGR -> RGB (after dropping A)
        return img.copy()

    return None


def _now_stamp_YYYYMMDDHHMM() -> str:
    return time.strftime("%Y%m%d%H%M", time.localtime())


def _pick_image_shape(frames: List[np.ndarray]) -> Tuple[int, int]:
    """
    Choose a stable (H,W) from frames.
    Prefer the first frame with H>1 and W>1 (i.e., not placeholder).
    Fallback: first frame if exists else (1,1).
    """
    for im in frames:
        if im is None:
            continue
        if im.ndim == 3 and im.shape[0] > 1 and im.shape[1] > 1 and im.shape[2] == 3:
            return int(im.shape[0]), int(im.shape[1])
    if frames:
        im0 = frames[0]
        if im0 is not None and im0.ndim == 3 and im0.shape[2] == 3:
            return int(im0.shape[0]), int(im0.shape[1])
    return 1, 1


def _stack_fix(frames: List[np.ndarray], H: int, W: int) -> np.ndarray:
    """
    Stack frames into (T,H,W,3). If mismatch, reuse last valid frame (or black).
    """
    T = len(frames)
    out = np.empty((T, H, W, 3), dtype=np.uint8)
    last_valid = np.zeros((H, W, 3), dtype=np.uint8)

    for i, im in enumerate(frames):
        if im is None or im.ndim != 3 or im.shape[2] != 3 or im.shape[0] != H or im.shape[1] != W:
            out[i] = last_valid
        else:
            out[i] = im
            last_valid = im
    return out


# ---------------------------
# ROS2 Node
# ---------------------------
class RobotPlaybackACTHDF5Recorder(Node):
    def __init__(self):
        super().__init__("robot_playback_act_hdf5_recorder")

        # ===== parameters =====
        # Save path rule: /home/eunseop/nrs_lab2/datasets/ACT/merged_hdf5/YYYYMMDDHHMM
        self.declare_parameter("save_root_dir", "/home/eunseop/nrs_lab2/datasets/ACT/merged_hdf5")
        self.declare_parameter("overwrite", False)

        # Replace joints -> position (/ur10skku/currentP)
        self.declare_parameter("position_topic", "/ur10skku/currentP")
        self.declare_parameter("ft_topic", "/ur10skku/currentF")
        self.declare_parameter("top_image_topic", "/realsense/top/color/image_raw")
        self.declare_parameter("ee_image_topic", "/realsense/ee/color/image_raw")

        self.declare_parameter("sample_hz", 30.0)
        self.declare_parameter("require_both_images", True)

        # HDF5 compression (images can be huge; keep default on)
        self.declare_parameter("img_compression", "gzip")  # "gzip" or ""(none)
        self.declare_parameter("img_gzip_level", 4)        # 1..9
        self.declare_parameter("img_chunk_t", 8)           # chunk along T

        # ===== load params =====
        self.save_root_dir = str(self.get_parameter("save_root_dir").value)
        self.overwrite = bool(self.get_parameter("overwrite").value)

        self.position_topic = str(self.get_parameter("position_topic").value)
        self.ft_topic = str(self.get_parameter("ft_topic").value)
        self.top_image_topic = str(self.get_parameter("top_image_topic").value)
        self.ee_image_topic = str(self.get_parameter("ee_image_topic").value)

        self.sample_hz = float(self.get_parameter("sample_hz").value)
        self.require_both_images = bool(self.get_parameter("require_both_images").value)

        self.img_compression = str(self.get_parameter("img_compression").value).strip().lower()
        self.img_gzip_level = int(self.get_parameter("img_gzip_level").value)
        self.img_chunk_t = int(self.get_parameter("img_chunk_t").value)

        if self.sample_hz <= 0.0:
            raise ValueError("sample_hz must be > 0")

        # ===== file init (timestamped path) =====
        os.makedirs(self.save_root_dir, exist_ok=True)
        self.save_stamp = _now_stamp_YYYYMMDDHHMM()
        self.save_hdf5 = os.path.join(self.save_root_dir, self.save_stamp)  # no extension (as requested)

        if self.overwrite and os.path.exists(self.save_hdf5):
            os.remove(self.save_hdf5)

        self.h5 = h5py.File(self.save_hdf5, "a")
        self._ensure_root_groups()

        # episode index = next available
        self.ep_idx = self._find_next_episode_index()

        # ===== runtime states =====
        self.lock = threading.Lock()
        self.recording = False
        self.shutdown_req = False

        self.latest_pos: Optional[np.ndarray] = None   # (6,) [x y z wx wy wz]
        self.latest_ft: Optional[np.ndarray] = None    # (3,)
        self.latest_top: Optional[np.ndarray] = None   # (H,W,3) uint8 RGB
        self.latest_ee: Optional[np.ndarray] = None    # (H,W,3) uint8 RGB
        self.have_pos = False
        self.have_ft = False
        self.have_top = False
        self.have_ee = False

        # buffers for current episode
        self.buf_pos: List[np.ndarray] = []
        self.buf_ft: List[np.ndarray] = []
        self.buf_top: List[np.ndarray] = []
        self.buf_ee: List[np.ndarray] = []

        # keyboard command queue
        self._cmd_queue: List[str] = []

        # ===== subscribers =====
        self.create_subscription(Float64MultiArray, self.position_topic, self._pos_cb, 10)
        self.create_subscription(Float64MultiArray, self.ft_topic, self._ft_cb, 10)
        self.create_subscription(Image, self.top_image_topic, self._top_img_cb, 10)
        self.create_subscription(Image, self.ee_image_topic, self._ee_img_cb, 10)

        # ===== timers =====
        self.timer = self.create_timer(1.0 / self.sample_hz, self._tick)
        self.status_timer = self.create_timer(2.0, self._status_tick)

        # ===== keyboard thread =====
        self.kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self.kb_thread.start()

        self.get_logger().info("=" * 70)
        self.get_logger().info("RobotPlaybackACTHDF5Recorder initialized (Stage-2 Robot Playback)")
        self.get_logger().info(f"  Save HDF5: {self.save_hdf5}")
        self.get_logger().info(f"  position: {self.position_topic}  (dim=6) -> saved as /episodes/ep_xxxx/position")
        self.get_logger().info(f"  ft      : {self.ft_topic}        (dim=3)")
        self.get_logger().info(f"  top img : {self.top_image_topic}")
        self.get_logger().info(f"  ee  img : {self.ee_image_topic}")
        self.get_logger().info(f"  sample_hz={self.sample_hz}  require_both_images={self.require_both_images}")
        self.get_logger().info("-" * 70)
        self.get_logger().info("Keyboard (press then Enter):  s=start ep   e=end ep(save)   q=quit")
        self.get_logger().info("=" * 70)

    # ---------------------------
    # HDF5 helpers
    # ---------------------------
    def _ensure_root_groups(self):
        if "meta" not in self.h5:
            self.h5.create_group("meta")
        if "episodes" not in self.h5:
            self.h5.create_group("episodes")

        meta = self.h5["meta"]
        meta.attrs["created_by"] = "RobotPlaybackACTHDF5Recorder"
        meta.attrs["format"] = "act_stage2_like_position"
        meta.attrs["note"] = "timestamps are NOT saved (removed)"
        meta.attrs["sample_hz"] = float(self.sample_hz)
        meta.attrs["require_both_images"] = bool(self.require_both_images)
        meta.attrs["position_topic"] = self.position_topic
        meta.attrs["ft_topic"] = self.ft_topic
        meta.attrs["top_image_topic"] = self.top_image_topic
        meta.attrs["ee_image_topic"] = self.ee_image_topic
        meta.attrs["columns_position"] = np.string_("x,y,z,wx,wy,wz")
        meta.attrs["save_stamp"] = self.save_stamp

        self.h5.flush()

    def _find_next_episode_index(self) -> int:
        eps = self.h5["episodes"]
        max_idx = -1
        for k in eps.keys():
            if k.startswith("ep_"):
                try:
                    idx = int(k.split("_")[1])
                    max_idx = max(max_idx, idx)
                except Exception:
                    pass
        return max_idx + 1

    def _ep_name(self, idx: int) -> str:
        return f"ep_{idx:04d}"

    # ---------------------------
    # callbacks
    # ---------------------------
    def _pos_cb(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            return
        arr = np.array(msg.data[:6], dtype=np.float64)
        with self.lock:
            self.latest_pos = arr
            self.have_pos = True

    def _ft_cb(self, msg: Float64MultiArray):
        if len(msg.data) < 3:
            return
        arr = np.array(msg.data[:3], dtype=np.float64)
        with self.lock:
            self.latest_ft = arr
            self.have_ft = True

    def _top_img_cb(self, msg: Image):
        img = _image_to_rgb_numpy(msg)
        if img is None:
            return
        with self.lock:
            self.latest_top = img
            self.have_top = True

    def _ee_img_cb(self, msg: Image):
        img = _image_to_rgb_numpy(msg)
        if img is None:
            return
        with self.lock:
            self.latest_ee = img
            self.have_ee = True

    # ---------------------------
    # keyboard
    # ---------------------------
    def _keyboard_loop(self):
        try:
            while rclpy.ok():
                line = sys.stdin.readline()  # requires Enter
                if not line:
                    time.sleep(0.05)
                    continue
                cmd = line.strip().lower()
                if cmd == "":
                    continue
                c = cmd[0]
                if c in ("s", "e", "q"):
                    with self.lock:
                        self._cmd_queue.append(c)
        except Exception as e:
            self.get_logger().error(f"keyboard loop error: {e}")

    # ---------------------------
    # timers
    # ---------------------------
    def _status_tick(self):
        with self.lock:
            steps = len(self.buf_pos) if self.recording else 0
            ep_idx = self.ep_idx
            rec = self.recording
        self.get_logger().info(f"[STATUS] recording={rec}  ep_idx={ep_idx}  steps_in_ep={steps}")

    def _tick(self):
        # 1) consume keyboard commands
        cmds = []
        with self.lock:
            if self._cmd_queue:
                cmds = self._cmd_queue[:]
                self._cmd_queue.clear()

        for c in cmds:
            if c == "s":
                self._start_episode()
            elif c == "e":
                self._end_episode(save=True)
            elif c == "q":
                # auto-save if recording
                if self.recording:
                    self._end_episode(save=True)
                self._request_shutdown()
                return

        # 2) sample data if recording
        with self.lock:
            if not self.recording:
                return

            if not (self.have_pos and self.have_ft):
                return

            if self.require_both_images:
                if not (self.have_top and self.have_ee):
                    return
            else:
                if not (self.have_top or self.have_ee):
                    return

            p = self.latest_pos.copy()
            f = self.latest_ft.copy()

            top = self.latest_top.copy() if self.latest_top is not None else None
            ee = self.latest_ee.copy() if self.latest_ee is not None else None

        # append (outside lock)
        self.buf_pos.append(p)
        self.buf_ft.append(f)

        # keep alignment (T must match)
        self.buf_top.append(top if top is not None else np.zeros((1, 1, 3), dtype=np.uint8))
        self.buf_ee.append(ee if ee is not None else np.zeros((1, 1, 3), dtype=np.uint8))

    # ---------------------------
    # episode control
    # ---------------------------
    def _start_episode(self):
        with self.lock:
            if self.recording:
                self.get_logger().warn("Already recording. Ignore 's'.")
                return
            self.recording = True
            self.buf_pos.clear()
            self.buf_ft.clear()
            self.buf_top.clear()
            self.buf_ee.clear()
        self.get_logger().info(f"=== EPISODE START ===  idx={self.ep_idx} ({self._ep_name(self.ep_idx)})")

    def _end_episode(self, save: bool):
        with self.lock:
            if not self.recording:
                self.get_logger().warn("Not recording. Ignore 'e'.")
                return
            self.recording = False

        if save:
            self._save_current_episode()

        with self.lock:
            self.ep_idx += 1

    def _request_shutdown(self):
        with self.lock:
            self.shutdown_req = True
        self.get_logger().info("Shutting down.")
        try:
            self.h5.flush()
            self.h5.close()
        except Exception:
            pass
        rclpy.shutdown()

    # ---------------------------
    # HDF5 save
    # ---------------------------
    def _save_current_episode(self):
        # snapshot buffers
        position = np.asarray(self.buf_pos, dtype=np.float32)  # (T,6)
        ft = np.asarray(self.buf_ft, dtype=np.float32)         # (T,3)

        top_list = self.buf_top
        ee_list = self.buf_ee

        if position.shape[0] == 0:
            self.get_logger().warn("Episode empty. Skip save.")
            return

        # infer image shapes robustly
        try:
            Ht, Wt = _pick_image_shape(top_list)
            He, We = _pick_image_shape(ee_list)
            top = _stack_fix(top_list, Ht, Wt)
            ee = _stack_fix(ee_list, He, We)
        except Exception as e:
            self.get_logger().error(f"Image stacking failed: {e}")
            return

        # write
        ep_name = self._ep_name(self.ep_idx)

        if ep_name in self.h5["episodes"]:
            del self.h5["episodes"][ep_name]

        g_ep = self.h5["episodes"].create_group(ep_name)
        g_ep.attrs["steps"] = int(position.shape[0])
        g_ep.attrs["saved_unix_time"] = float(time.time())

        # datasets
        g_ep.create_dataset("position", data=position, dtype=np.float32)
        g_ep.create_dataset("ft", data=ft, dtype=np.float32)

        g_img = g_ep.create_group("images")

        comp = None if self.img_compression == "" else self.img_compression
        gzip_lvl = int(np.clip(self.img_gzip_level, 1, 9))

        def _img_kwargs(arr: np.ndarray):
            T, H, W, C = arr.shape
            ct = max(1, min(self.img_chunk_t, T))
            kw = {
                "dtype": np.uint8,
                "chunks": (ct, H, W, C),
                "shuffle": True,
            }
            if comp == "gzip":
                kw["compression"] = "gzip"
                kw["compression_opts"] = gzip_lvl
            return kw

        g_img.create_dataset("top", data=top, **_img_kwargs(top))
        g_img.create_dataset("ee", data=ee, **_img_kwargs(ee))

        # NOTE: timestamps group intentionally NOT created

        self.h5.flush()
        self.get_logger().info(
            f"[SAVE] {ep_name}  T={position.shape[0]}  top={top.shape}  ee={ee.shape}  -> {self.save_hdf5}"
        )

    def destroy_node(self):
        try:
            if hasattr(self, "h5") and self.h5:
                self.h5.flush()
                self.h5.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotPlaybackACTHDF5Recorder()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
