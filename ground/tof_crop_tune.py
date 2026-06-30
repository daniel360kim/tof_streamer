#!/usr/bin/env python3
"""Tune ToF vertical crop on the ground without recompiling the drone.

Shows raw planar depth with the 86 deg crop rectangle + upscaled 9x16 output.
Restart with different --crop-v-anchor / --crop-v-shift until ceiling is gone.

Tuning workflow (robot container, ROS_DOMAIN_ID=1):

  # 1) VOXL: raw TOF3 only (no onboard TOF2)
  ./tof_udp_stream $GROUND_IP 5600 --raw-only

  # 2) Bridge: publish raw planar only
  python3 ground/tof_udp_bridge.py --port 5600 \\
      --raw-topic /drone_1/perception/tof_raw --ignore-tof2

  # 3) This script: preprocess + debug viz + /perception/tof for commander
  python3 ground/tof_crop_tune.py \\
      --raw-topic /drone_1/perception/tof_raw \\
      --crop-v-anchor bottom --crop-v-shift 0

  # RViz: /svg/drone_1/tof_crop_debug  (green box = crop, right = 9x16)
  # When happy, deploy same flags on drone (DiffAero default: v_shift=-100):
  ./tof_udp_stream $GROUND_IP 5600 --crop-v-anchor=bottom --crop-v-shift=-100
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'drone'))
from depth_preprocess import (  # noqa: E402
    POLICY_MAX_DIST,
    CropConfig,
    raw_planar_z_to_perception_debug,
)

GRID_H, GRID_W = 9, 16


def _closeness_rgb(closeness: np.ndarray, invalid: np.ndarray | None = None) -> np.ndarray:
    t = np.clip(closeness, 0.0, 1.0)
    rgb = np.zeros((*t.shape, 3), dtype=np.float32)
    invalid = (t <= 0.0) if invalid is None else (invalid | (t <= 0.0))
    valid = ~invalid
    u = t[valid]
    rgb[valid, 0] = u
    rgb[valid, 2] = 1.0 - u
    return (rgb * 255.0).astype(np.uint8)


def _resize_nearest(arr: np.ndarray, out_h: int, out_w: int) -> np.ndarray:
    if arr.shape == (out_h, out_w):
        return arr
    yi = np.linspace(0, arr.shape[0] - 1, out_h).astype(np.int64)
    xi = np.linspace(0, arr.shape[1] - 1, out_w).astype(np.int64)
    return arr[yi][:, xi]


def _debug_panel(planar: np.ndarray, crop_box: tuple[int, int, int, int],
                 perception: np.ndarray, scale: int) -> np.ndarray:
    row0, row1, col0, col1 = crop_box
    max_dist = POLICY_MAX_DIST
    z = np.where(np.isfinite(planar), planar, max_dist)
    z = np.where(z <= 1e-3, max_dist, z)
    closeness = 1.0 - np.clip(z, 0.0, max_dist) / max_dist
    invalid = ~np.isfinite(planar) | (planar <= 1e-3)
    raw_rgb = _closeness_rgb(closeness, invalid)

    # Green crop rectangle on raw view.
    marked = raw_rgb.copy()
    marked[row0, col0:col1] = [0, 255, 0]
    marked[row1 - 1, col0:col1] = [0, 255, 0]
    marked[row0:row1, col0] = [0, 255, 0]
    marked[row0:row1, col1 - 1] = [0, 255, 0]

    raw_big = _resize_nearest(marked, planar.shape[0] * scale, planar.shape[1] * scale)
    proc_u8 = (np.clip(perception, 0.0, 1.0) * 255.0).astype(np.uint8)
    # Match raw panel height so side-by-side concat works.
    proc_h = raw_big.shape[0]
    proc_w = max(1, int(round(GRID_W * proc_h / GRID_H)))
    proc_big = _resize_nearest(proc_u8, proc_h, proc_w)
    proc_rgb = _closeness_rgb(proc_big.astype(np.float32) / 255.0)

    gap = np.zeros((raw_big.shape[0], 8, 3), dtype=np.uint8)
    return np.concatenate([raw_big, gap, proc_rgb], axis=1)


class TofCropTune(Node):
    def __init__(self, raw_topic: str, out_topic: str, debug_topic: str,
                 crop: CropConfig, flip_lr: bool, flip_ud: bool, viz_scale: int):
        super().__init__('tof_crop_tune')
        self._crop = crop
        self._flip_lr = flip_lr
        self._flip_ud = flip_ud
        self._viz_scale = viz_scale
        self._pub = self.create_publisher(Float32MultiArray, out_topic, 10)
        self._dbg_pub = self.create_publisher(Image, debug_topic, 10)
        self.create_subscription(Float32MultiArray, raw_topic, self._on_raw, 10)
        self.get_logger().info(
            f'crop v_anchor={crop.v_anchor} v_shift={crop.v_shift_px} '
            f'flip_lr={flip_lr} flip_ud={flip_ud} -> {out_topic} debug={debug_topic}')

    def _on_raw(self, msg: Float32MultiArray):
        dims = [d.size for d in msg.layout.dim]
        if len(dims) != 2:
            return
        h, w = dims
        planar = np.array(msg.data, dtype=np.float32).reshape(h, w)
        perception, crop_box, oriented = raw_planar_z_to_perception_debug(
            planar, flip_lr=self._flip_lr, flip_ud=self._flip_ud, crop=self._crop)

        out = Float32MultiArray()
        out.data = perception.reshape(-1).tolist()
        for label, size in [('height', GRID_H), ('width', GRID_W)]:
            dim = MultiArrayDimension()
            dim.label = label
            dim.size = size
            dim.stride = size
            out.layout.dim.append(dim)
        self._pub.publish(out)

        rgb = _debug_panel(oriented, crop_box, perception, self._viz_scale)
        img = Image()
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = 'map'
        img.height = rgb.shape[0]
        img.width = rgb.shape[1]
        img.encoding = 'rgb8'
        img.is_bigendian = 0
        img.step = rgb.shape[1] * 3
        img.data = rgb.tobytes()
        self._dbg_pub.publish(img)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--raw-topic', default='/drone_1/perception/tof_raw')
    parser.add_argument('--out-topic', default='/drone_1/perception/tof')
    parser.add_argument('--debug-topic', default='/svg/drone_1/tof_crop_debug')
    parser.add_argument('--crop-v-anchor', default='center',
                        choices=['center', 'bottom', 'top'])
    parser.add_argument('--crop-v-shift', type=int, default=0,
                        help='pixels; positive = shift crop down (discard more ceiling)')
    parser.add_argument('--flip-h', action='store_true')
    parser.add_argument('--no-flip-v', action='store_true')
    parser.add_argument('--viz-scale', type=int, default=3)
    args = parser.parse_args()

    crop = CropConfig(v_anchor=args.crop_v_anchor, v_shift_px=args.crop_v_shift)
    rclpy.init()
    node = TofCropTune(
        args.raw_topic, args.out_topic, args.debug_topic, crop,
        flip_lr=args.flip_h, flip_ud=not args.no_flip_v, viz_scale=args.viz_scale)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
