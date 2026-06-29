#!/usr/bin/env python3
"""Receive raw ToF over UDP, preprocess on ground, publish to ROS 2 (Jazzy).

Run on the ground PC inside the AirStack robot container or a Jazzy env:

    ROS_DOMAIN_ID=1 python3 ground/tof_udp_bridge.py --port 5600 \\
        --topic /drone_1/perception/tof

The drone sends chunked raw planar-Z (TOF3). This node reassembles frames,
runs depth_preprocess on the ground, and publishes the 9x16 DiffAero grid.

Legacy TOF2 (pre-encoded 9x16) packets are still accepted for older builds.

No Foxy / domain 0 on the ground — safe alongside the VOXL bench drone.
"""

import argparse
import socket
import struct
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

# Shared preprocessing with the legacy ROS drone node.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'drone'))
from depth_preprocess import raw_planar_z_to_perception  # noqa: E402

UDP_MAGIC_RAW = 0x544F4633  # 'TOF3'
RAW_HEADER_FMT = '<IIIHHQHH'
RAW_HEADER_SIZE = struct.calcsize(RAW_HEADER_FMT)

UDP_MAGIC_ENC = 0x544F4632  # 'TOF2' (legacy: drone-side preprocessing)
ENC_HEADER_FMT = '<IIQ'
ENC_HEADER_SIZE = struct.calcsize(ENC_HEADER_FMT)

GRID_H, GRID_W = 9, 16
ENC_PACKET_SIZE = ENC_HEADER_SIZE + GRID_H * GRID_W * 4

MAX_UDP_SIZE = 65535


class FrameAssembler:
    """Reassemble chunked TOF3 raw planar-Z frames."""

    def __init__(self):
        self._frames = {}

    def ingest(self, data: bytes):
        if len(data) < RAW_HEADER_SIZE:
            return None

        (
            magic,
            _seq,
            frame_id,
            chunk_index,
            chunk_count,
            timestamp_ns,
            width,
            height,
        ) = struct.unpack_from(RAW_HEADER_FMT, data, 0)
        if magic != UDP_MAGIC_RAW:
            return None
        if chunk_count == 0 or chunk_index >= chunk_count:
            return None
        if width == 0 or height == 0:
            return None

        payload = data[RAW_HEADER_SIZE:]
        state = self._frames.get(frame_id)
        if state is None:
            state = {
                'width': width,
                'height': height,
                'timestamp_ns': timestamp_ns,
                'chunk_count': chunk_count,
                'chunks': {},
            }
            self._frames[frame_id] = state

        state['chunks'][chunk_index] = payload

        # Drop stale partial frames to bound memory.
        if len(self._frames) > 8:
            for old_id in sorted(self._frames)[:-4]:
                del self._frames[old_id]

        if len(state['chunks']) < chunk_count:
            return None

        ordered = b''.join(state['chunks'][i] for i in range(chunk_count))
        del self._frames[frame_id]

        expected_floats = width * height
        planar = np.frombuffer(ordered, dtype=np.float32, count=expected_floats)
        if planar.size != expected_floats:
            return None
        return planar.reshape(height, width), timestamp_ns


class TofUdpBridge(Node):
    def __init__(self, bind_host: str, port: int, topic: str, raw_topic: str | None):
        super().__init__('tof_udp_bridge')
        self._pub = self.create_publisher(Float32MultiArray, topic, 10)
        self._raw_pub = (
            self.create_publisher(Float32MultiArray, raw_topic, 10)
            if raw_topic else None
        )
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((bind_host, port))
        self._sock.setblocking(False)
        self._timer = self.create_timer(0.001, self._poll)
        self._assembler = FrameAssembler()
        self._frames_published = 0
        self._rate_window_start = None
        self._rate_window_frames = 0
        raw_desc = f' + raw {raw_topic}' if raw_topic else ''
        self.get_logger().info(
            f'Listening UDP {bind_host}:{port} -> {topic}{raw_desc} '
            f'(raw TOF3 chunked + legacy TOF2 9x16)'
        )

    def _log_publish_rate(self, label: str):
        now = self.get_clock().now()
        if self._rate_window_start is None:
            self._rate_window_start = now
            self._rate_window_frames = 0
            return

        self._rate_window_frames += 1
        elapsed = (now - self._rate_window_start).nanoseconds * 1e-9
        if elapsed < 1.0:
            return

        hz = self._rate_window_frames / elapsed
        self.get_logger().info(f'Publishing {label} at {hz:.1f} Hz')
        self._rate_window_start = now
        self._rate_window_frames = 0

    def _float_array(self, arr: np.ndarray, dims: list[tuple[str, int]]) -> Float32MultiArray:
        msg = Float32MultiArray()
        msg.data = arr.reshape(-1).tolist()
        for label, size in dims:
            dim = MultiArrayDimension()
            dim.label = label
            dim.size = size
            dim.stride = size
            msg.layout.dim.append(dim)
        return msg

    def _publish_grid(self, grid: np.ndarray, seq_or_frame, addr: str, label: str):
        self._pub.publish(self._float_array(grid, [('height', GRID_H), ('width', GRID_W)]))
        self._log_publish_rate(label)

        self._frames_published += 1
        if self._frames_published == 1 or self._frames_published % 100 == 0:
            self.get_logger().info(
                f'Published {label} id={seq_or_frame} from {addr} '
                f'(total={self._frames_published})'
            )

    def _publish_raw(self, planar_z: np.ndarray):
        if self._raw_pub is None:
            return
        h, w = planar_z.shape
        self._raw_pub.publish(self._float_array(planar_z, [('height', h), ('width', w)]))

    def _handle_legacy_encoded(self, data: bytes, addr: str) -> bool:
        if len(data) != ENC_PACKET_SIZE:
            return False

        magic, seq, _timestamp_ns = struct.unpack_from(ENC_HEADER_FMT, data, 0)
        if magic != UDP_MAGIC_ENC:
            return False

        grid = np.frombuffer(data, dtype=np.float32, offset=ENC_HEADER_SIZE).reshape(GRID_H, GRID_W)
        self._publish_grid(grid, seq, addr[0], 'legacy TOF2')
        return True

    def _poll(self):
        while rclpy.ok():
            try:
                data, addr = self._sock.recvfrom(MAX_UDP_SIZE)
            except BlockingIOError:
                return
            except OSError:
                return

            if self._handle_legacy_encoded(data, addr):
                continue

            assembled = self._assembler.ingest(data)
            if assembled is None:
                magic = struct.unpack_from('<I', data, 0)[0] if len(data) >= 4 else 0
                if magic not in (UDP_MAGIC_RAW, UDP_MAGIC_ENC):
                    self.get_logger().warning(
                        f'Ignored {len(data)}-byte packet from {addr[0]} '
                        f'(bad magic 0x{magic:08x})'
                    )
                continue

            planar_z, timestamp_ns = assembled
            try:
                self._publish_raw(planar_z)
                perception = raw_planar_z_to_perception(planar_z)
            except Exception as exc:
                self.get_logger().warning(f'Preprocess failed: {exc}')
                continue

            self._publish_grid(perception, timestamp_ns, addr[0], 'raw TOF3')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bind', default='0.0.0.0', help='UDP bind address')
    parser.add_argument('--port', type=int, default=5600, help='UDP port')
    parser.add_argument(
        '--topic',
        default='/drone_1/perception/tof',
        help='ROS 2 output topic (DiffAero 9x16 policy grid)',
    )
    parser.add_argument(
        '--raw-topic',
        default='/drone_1/perception/tof_raw',
        help='ROS 2 raw planar-Z topic for visualization (empty to disable)',
    )
    args = parser.parse_args()
    raw_topic = args.raw_topic or None

    rclpy.init()
    node = TofUdpBridge(args.bind, args.port, args.topic, raw_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
