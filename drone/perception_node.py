import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from depth_preprocess import (
    parse_image_msg,
    policy_depth_config,
    depth_u8_to_perception,
)
from topics import PERCEPTION_TOPIC


def _float_array(arr, dims):
    msg = Float32MultiArray()
    msg.data = np.asarray(arr, dtype=np.float32).reshape(-1).tolist()
    for label, size in dims:
        dim = MultiArrayDimension()
        dim.label = label
        dim.size = size
        dim.stride = size
        msg.layout.dim.append(dim)
    return msg


class Perception(Node):
    def __init__(self):
        super().__init__('perception')
        
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        self.create_subscription(Image, 'tof_depth', self._on_depth, qos_sub)
        self._pub_perception = self.create_publisher(Float32MultiArray, PERCEPTION_TOPIC, 10)

        self._depth_cfg = policy_depth_config(flip_h=False, flip_v=False)

    def _on_depth(self, msg):
        depth_u8 = parse_image_msg(msg)
        if depth_u8 is None:
            return

        perception, _ = depth_u8_to_perception(depth_u8, self._depth_cfg)
        self._pub_perception.publish(
            _float_array(perception, [("height", 9), ("width", 16)])
        )



def main():
    rclpy.init()
    node = Perception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()