#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from feetech_ros2_driver.msg import JointCommand
from scservo_sdk import *  # Feetech官方Python SDK

ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56
ADDR_GOAL_VELOCITY = 46  # 目标速度地址
ADDR_PRESENT_VELOCITY = 58  # 当前速度地址
ADDR_GOAL_CURRENT = 40  # 目标电流地址 (effort)
ADDR_PRESENT_CURRENT = 54  # 当前电流地址 (effort)
PROTOCOL_VERSION = 0  # Feetech SC系列为0
BAUDRATE = 1000000
PORT = '/dev/ttyACM0'

class FeetechDriver(Node):
    def __init__(self):
        super().__init__('feetech_driver_node')
        self.port = PortHandler(PORT)
        self.packet = PacketHandler(PROTOCOL_VERSION)
        self.pub = self.create_publisher(JointState, '/feetech_joint_state', 10)
        self.sub = self.create_subscription(JointCommand, '/feetech_joint_command', self.command_callback, 10)
        #self.servo_ids = [1, 2, 3, 4, 5, 6]
        self.servo_ids = [1, ]

        if not self.port.openPort():
            self.get_logger().error("无法打开串口，请检查连接")
        else:
            self.get_logger().info(f"成功打开 {PORT}")

        self.port.setBaudRate(BAUDRATE)
        self.timer = self.create_timer(0.05, self.publish_joint_states)

    def command_callback(self, msg):
        for i, servo_id in enumerate(msg.ids):
            # 写入位置
            if i < len(msg.positions):
                position = int(msg.positions[i])
                dxl_comm_result, dxl_error = self.packet.write2ByteTxRx(
                    self.port, servo_id, ADDR_GOAL_POSITION, position
                )
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().warn(f"写入舵机 {servo_id} 位置失败")
            
            # 写入速度
            if i < len(msg.velocities):
                velocity = int(msg.velocities[i])
                dxl_comm_result, dxl_error = self.packet.write2ByteTxRx(
                    self.port, servo_id, ADDR_GOAL_VELOCITY, velocity
                )
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().warn(f"写入舵机 {servo_id} 速度失败")
            
            # 写入力矩/电流
            if i < len(msg.efforts):
                effort = int(msg.efforts[i])
                dxl_comm_result, dxl_error = self.packet.write2ByteTxRx(
                    self.port, servo_id, ADDR_GOAL_CURRENT, effort
                )
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().warn(f"写入舵机 {servo_id} 力矩失败")

    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        for servo_id in self.servo_ids:
            # 读取位置
            dxl_present_position, dxl_comm_result, dxl_error = self.packet.read2ByteTxRx(
                self.port, servo_id, ADDR_PRESENT_POSITION
            )
            
            # 读取速度
            dxl_present_velocity, dxl_comm_result_vel, dxl_error_vel = self.packet.read2ByteTxRx(
                self.port, servo_id, ADDR_PRESENT_VELOCITY
            )
            
            # 读取电流/力矩
            dxl_present_current, dxl_comm_result_cur, dxl_error_cur = self.packet.read2ByteTxRx(
                self.port, servo_id, ADDR_PRESENT_CURRENT
            )
            
            js.name.append(str(servo_id))
            js.position.append(float(dxl_present_position) if dxl_comm_result == COMM_SUCCESS else 0.0)
            js.velocity.append(float(dxl_present_velocity) if dxl_comm_result_vel == COMM_SUCCESS else 0.0)
            js.effort.append(float(dxl_present_current) if dxl_comm_result_cur == COMM_SUCCESS else 0.0)
        
        self.pub.publish(js)

def main():
    rclpy.init()
    node = FeetechDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
