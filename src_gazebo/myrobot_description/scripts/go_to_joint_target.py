#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
import sys

def main():
    rclpy.init()
    node = Node("go_to_joint_target")
    
    # 关节名称（根据你的 URDF 修改）
    joint_names = [
        'joint1', 'joint2', 'joint3', 
        'joint4', 'joint5', 'joint6'
    ]
    
    # 目标关节角度（弧度）
    target_positions = [0.000, 0.716, 1.850, 1.571, 0.000, 0.000]
    
    # 创建 Action Client
    action_client = ActionClient(
        node, 
        FollowJointTrajectory, 
        '/arm_controller/follow_joint_trajectory'  # 使用 arm_controller
    )
    
    # 等待服务器
    node.get_logger().info("等待 arm_controller...")
    if not action_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("arm_controller 未找到！")
        return
    
    # 创建目标轨迹
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    
    # 创建目标点
    point = JointTrajectoryPoint()
    point.positions = target_positions
    point.time_from_start = Duration(sec=3)  # 3秒内完成运动
    
    trajectory.points.append(point)
    
    # 创建 Action Goal
    goal_msg = FollowJointTrajectory.Goal()
    goal_msg.trajectory = trajectory
    goal_msg.goal_time_tolerance = Duration(sec=1)
    
    # 发送目标
    node.get_logger().info(f"发送目标关节位置: {target_positions}")
    future = action_client.send_goal_async(goal_msg)
    
    # 等待结果
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        goal_handle = future.result()
        if goal_handle.accepted:
            node.get_logger().info("目标已被接受，开始运动...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future)
            node.get_logger().info("运动完成！")
        else:
            node.get_logger().error("目标被拒绝")
    else:
        node.get_logger().error("发送目标失败")
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()