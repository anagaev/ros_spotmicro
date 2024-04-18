from adafruit_servokit import ServoKit
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('ServoController')
        self.get_logger().info('Creating ServoController')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('offsets', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('limits', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('orientation', rclpy.Parameter.Type.INTEGER_ARRAY),
            ]
        )
        self._servo_offsets = np.array(self.get_parameter('offsets').value)
        self._servo_limits = np.reshape(np.array(self.get_parameter('limits').value), (-1, 2))
        self.orientation = np.array(self.get_parameter('orientation').value, dtype=float)
        self.kit = ServoKit(channels=16)
        self.subscriber_joints_angles = self.create_subscription(Float32MultiArray,
                                                                 '/joints_angles',
                                                                 self.set_joints_angles,
                                                                 10)

    def set_joints_angles(self, msg: Float32MultiArray):
        self.get_logger().info('Set angles')
        joints_angles = np.array(msg.data)
        for i in range(12):
            k = (i // 3) * 4 + (i % 3)
            min_angle, max_angle = self._servo_limits[k]
            angle = self._servo_offsets[k] + self.orientation[k] * joints_angles[i] * 180.0 / np.pi
            self.kit.servo[k].angle = np.clip(angle,
                                              min_angle,
                                              max_angle)
            self.get_logger().info(f'Angle {i} ' + str(angle) + ' ' + str(joints_angles[i] * 180.0 / np.pi) + ' ' + str(self.kit.servo[k].angle))


def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
