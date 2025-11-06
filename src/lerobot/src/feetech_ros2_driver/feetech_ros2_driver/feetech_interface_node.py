#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from feetech_ros2_driver.msg import JointCommand

class FeetechInterface(Node):
    def __init__(self):
        super().__init__('feetech_interface_node')
        self.pub = self.create_publisher(JointCommand, '/feetech_joint_command', 10)
        self.sub = self.create_subscription(JointState, '/feetech_joint_state', self.state_callback, 10)
        self.current_positions = {}

    def state_callback(self, msg):
        for i, name in enumerate(msg.name):
            self.current_positions[name] = msg.position[i]

    def set_positions(self, ids, positions):
        msg = JointCommand()
        msg.ids = ids
        msg.positions = positions
        self.pub.publish(msg)

    def get_positions(self):
        return self.current_positions

def main():
    rclpy.init()
    node = FeetechInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
