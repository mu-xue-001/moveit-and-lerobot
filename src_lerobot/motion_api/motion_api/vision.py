#!/usr/bin/env python3
import time
import rclpy
import cv2
import numpy as np
import os
import sys
from rclpy.logging import get_logger
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute("so_arm_101_groups", robot_trajectory)
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def kill_opencv_windows():
    """强制杀死所有OpenCV窗口"""
    cv2.destroyAllWindows()
    # 多次调用确保关闭
    for _ in range(5):
        cv2.waitKey(1)


def detect_red_object_2seconds(camera_index=2):
    """
    弹出窗口检测红色物体，2秒后强制关闭
    返回: 是否检测到红色物体
    """
    print(f"\n[相机] 打开摄像头 /dev/video{camera_index}")
    print("[提示] 检测窗口将运行2秒后自动关闭")
    
    # 打开摄像头
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"[错误] 无法打开摄像头")
        return False
    
    # 设置参数
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # 创建窗口
    cv2.namedWindow('Red Detection', cv2.WINDOW_NORMAL)
    
    # 红色HSV范围
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])
    
    start_time = time.time()
    red_detected = False
    detection_start = None
    
    print("[检测] 开始检测...")
    
    # 运行2秒
    while time.time() - start_time < 2.0:
        ret, frame = cap.read()
        if not ret:
            continue
        
        # 检测红色
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = mask1 | mask2
        
        # 计算红色像素
        red_pixels = np.sum(red_mask > 0)
        
        # 显示信息
        remaining = 2.0 - (time.time() - start_time)
        cv2.putText(frame, f"Time: {remaining:.1f}s", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 检测红色物体（超过500个红色像素）
        if red_pixels > 500:
            if detection_start is None:
                detection_start = time.time()
                print(f"[检测] 发现红色物体! 像素数: {red_pixels}")
            red_detected = True
            cv2.putText(frame, "RED DETECTED!", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            detection_start = None
            cv2.putText(frame, "No Red", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # 显示画面
        cv2.imshow('Red Detection', frame)
        cv2.waitKey(1)
    
    # 强制关闭所有窗口
    print("[关闭] 2秒时间到，强制关闭检测窗口...")
    cap.release()
    kill_opencv_windows()
    
    return red_detected


def send_joint_positions(robot, arm_group, logger, joint_positions):
    """发送关节位置给MoveIt并执行"""
    robot_model = robot.get_robot_model()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('so_arm_101_groups', joint_positions)
    arm_group.set_goal_state(robot_state=robot_state)
    arm_group.set_start_state_to_current_state()
    plan_and_execute(robot, arm_group, logger, sleep_time=3.0)


if __name__ == "__main__":
    # 目标关节角度
    target_degrees = [7.0, 17.0, 9.0, 50.0, 0.0, 22.0]
    target_radians = [deg * np.pi / 180.0 for deg in target_degrees]
    
    print("="*50)
    print("红色检测 + 机器人控制")
    print("="*50)
    print(f"目标位置: {target_degrees} 度")
    print("="*50)
    
    # 1. 检测红色物体（2秒后自动关闭）
    print("\n>>> 第1步: 红色物体检测（2秒） <<<")
    red_detected = detect_red_object_2seconds(camera_index=2)
    
    if red_detected:
        print(f"\n[结果] ✓ 检测到红色物体!")
    else:
        print(f"\n[结果] ✗ 未检测到红色物体")
    
    # 2. 移动机器人（无论是否检测到都移动）
    print("\n>>> 第2步: 移动机器人 <<<")
    
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")
    
    # 配置文件路径
    config_path = '/home/ubuntu/my_test_ws/src/my_test_moveit_config/config/moveit_cpp.yaml'
    
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="so101_new_calib", package_name="my_test_moveit_config"
        )
        .moveit_cpp(config_path)
        .to_moveit_configs()
    )
    
    robot = MoveItPy(node_name="moveit_py", config_dict=moveit_config.to_dict())
    arm_group = robot.get_planning_component("so_arm_101_groups")
    
    print(f"[机器人] 移动到: {target_degrees} 度")
    send_joint_positions(robot, arm_group, logger, target_radians)
    
    print("\n[完成] 程序结束")
    rclpy.shutdown()