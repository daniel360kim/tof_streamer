import rclpy
import numpy as np
import cv2

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from topics import PERCEPTION_TOPIC
from depth_preprocess import perception_to_vis


class PerceptionViewer(Node):
    def __init__(self):
        super().__init__('perception_viewer')
        self.create_subscription(Float32MultiArray, PERCEPTION_TOPIC, self._on_perception, 10)

    def _on_perception(self, msg):
        dims = [d.size for d in msg.layout.dim]
        h, w = dims if len(dims) == 2 else (9, 16)
        perception = np.array(msg.data, dtype=np.float32).reshape(h, w)

        vis = perception_to_vis(perception)
        cv2.imshow('tof_streamer perception', vis)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = PerceptionViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
