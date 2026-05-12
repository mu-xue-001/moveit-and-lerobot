#!/usr/bin/env python3
"""ROS 2 node: Publish Feetech servo positions to /joint_states for MoveIt2."""

from __future__ import annotations
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
from so101_hw_interface.motors.feetech.feetech import FeetechMotorsBus
from so101_hw_interface.motors import Motor, MotorNormMode

# ------------------- 配置 -------------------
PORT_DEFAULT = "/dev/ttyACM0"

JOINTS = {
    "joint_1": {"id": 1, "model": "sts3215"},
    "joint_2": {"id": 2, "model": "sts3215"},
    "joint_3": {"id": 3, "model": "sts3215"},
    "joint_4": {"id": 4, "model": "sts3215"},
    "joint_5": {"id": 5, "model": "sts3215"},
    "joint_6": {"id": 6, "model": "sts3215"},
}

STEPS_PER_RAD = 4096.0 / (2 * math.pi)  # STS3215 分辨率

# -------------------------------------------

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__("feetech_joint_state_pub")
        self.declare_parameter("port", PORT_DEFAULT)
        port = self.get_parameter("port").get_parameter_value().string_value
        if not port:
            port = PORT_DEFAULT

        # 创建舵机对象
        self.motors = {name: Motor(cfg["id"], cfg["model"], MotorNormMode.DEGREES) 
                       for name, cfg in JOINTS.items()}
        self.bus = FeetechMotorsBus(port, self.motors)

        self.get_logger().info(f"Connecting to Feetech bus on {port} …")
        try:
            self.bus.connect()
            self.bus.configure_motors()
            self.bus.enable_torque()
            self.get_logger().info("Motor bus connected and configured.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to motor bus: {e}")
            raise

        # Publisher 发布到 MoveIt2 默认 joint_states
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        # 读取舵机位置的 home_offset
        self.home_offsets = None

        # 定时器，每 20ms 发布一次
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        try:
            raw_positions = self.bus.sync_read("Present_Position", normalize=False)
            if self.home_offsets is None:
                self.home_offsets = raw_positions
                self.get_logger().info(f"Captured home offsets: {self.home_offsets}")

            # 转换为弧度
            positions_rad = {}
            for name, raw in raw_positions.items():
                home = self.home_offsets.get(name, 0)
                positions_rad[name] = (raw - home) / STEPS_PER_RAD

            # 发布 JointState
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = list(JOINTS.keys())
            js.position = [positions_rad[n] for n in js.name]
            js.velocity = []
            js.effort = []

            self.joint_pub.publish(js)

        except Exception as e:
            self.get_logger().warn(f"Failed to read or publish joint positions: {e}")


def main():
    rclpy.init()
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.bus.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()