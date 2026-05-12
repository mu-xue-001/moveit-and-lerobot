#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import cv2
import numpy as np
from builtin_interfaces.msg import Duration
import time
import math

class VisionGrasp(Node):
    def __init__(self):
        super().__init__('vision_grasp')
        
        # 关节名称
        self.arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.gripper_joints = ['gripper_left_finger_joint']
        
        # 抓取位置（成功的位置）
        self.GRASP_JOINTS = [
            0.0,
            74.0 * math.pi / 180.0,  # joint2: 74°
            80.0 * math.pi / 180.0,  # joint3: 80°
            90.0 * math.pi / 180.0,  # joint4: 90°
            0.0,
            0.0
        ]
        
        # 抬起位置
        self.LIFT_JOINTS = [
            0.0,
            60.0 * math.pi / 180.0,  # joint2: 60°
            70.0 * math.pi / 180.0,  # joint3: 70°
            90.0 * math.pi / 180.0,  # joint4: 90°
            0.0,
            0.0
        ]
        
        self.bridge = CvBridge()
        
        # 直接连接控制器（不通过MoveIt）
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        
        # 等待控制器
        print('等待arm_controller...')
        self.arm_client.wait_for_server()
        print('arm_controller已连接')
        
        print('等待gripper_controller...')
        self.gripper_client.wait_for_server()
        print('gripper_controller已连接')
        
        # 启动视觉检测
        self.image_sub = self.create_subscription(Image, '/camera_sensor/image_raw', self.image_callback, 10)
        self.detected = False
        print('='*50)
        print('等待检测红色立方体...')
        print('抓取位置: joint2=74°, joint3=80°, joint4=90°')
        print('='*50)
    
    def image_callback(self, msg):
        if self.detected:
            return
        
        # 转换图像
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 红色检测（收紧范围，更准确）
        lower_red1 = np.array([0, 120, 100])
        upper_red1 = np.array([5, 255, 255])
        lower_red2 = np.array([175, 120, 100])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # 形态学处理
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            
            # 面积阈值
            if area > 300:
                print(f'\n✓ 检测到红色立方体！面积: {area:.0f}')
                
                # 绘制检测结果
                x, y, w, h = cv2.boundingRect(largest)
                debug_img = cv_image.copy()
                cv2.rectangle(debug_img, (x,y), (x+w,y+h), (0,255,0), 3)
                cv2.circle(debug_img, (x+w//2, y+h//2), 8, (0,0,255), -1)
                cv2.putText(debug_img, "RED Cube Detected!", (x, y-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.imshow("Detection", debug_img)
                cv2.waitKey(1500)
                cv2.destroyAllWindows()
                
                # 停止视觉检测
                self.detected = True
                self.image_sub.destroy()
                
                # 延迟1秒后执行抓取
                self.timer = self.create_timer(1.0, self.execute_grasp)
                return
        
        # 实时显示检测画面
        debug_img = cv_image.copy()
        cv2.putText(debug_img, "Searching for RED cube...", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
        cv2.imshow("Detection", debug_img)
        cv2.waitKey(1)
    
    def send_arm(self, positions, duration=3.0):
        """直接发送机械臂关节目标"""
        traj = JointTrajectory()
        traj.joint_names = self.arm_joints
        
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=int(duration), nanosec=0)
        traj.points.append(point)
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)
        
        print(f'  发送机械臂指令...')
        self.arm_client.send_goal_async(goal)
    
    def send_gripper(self, position, duration=1.0):
        """直接发送夹爪目标"""
        traj = JointTrajectory()
        traj.joint_names = self.gripper_joints
        
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = Duration(sec=int(duration), nanosec=0)
        traj.points.append(point)
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)
        
        print(f'  发送夹爪指令: {position}')
        self.gripper_client.send_goal_async(goal)
    
    def execute_grasp(self):
        """执行抓取流程"""
        self.timer.cancel()
        
        print('\n' + '='*50)
        print('开始抓取流程')
        print('='*50)
        
        # 步骤1: 移动到抓取位置
        print('\n[步骤1] 移动到抓取位置')
        print('  joint2=74°, joint3=80°, joint4=90°')
        self.send_arm(self.GRASP_JOINTS, 3.0)
        print('  等待5秒...')
        time.sleep(5)
        
        # 步骤2: 闭合夹爪
        print('\n[步骤2] 抓取物体')
        print('  夹爪闭合到 0.036')
        self.send_gripper(0.036, 1.0)
        print('  等待3秒...')
        time.sleep(3)
        
        # 步骤3: 抬起机械臂
        print('\n[步骤3] 抬起机械臂')
        print('  joint2=60°, joint3=70°, joint4=90°')
        self.send_arm(self.LIFT_JOINTS, 2.0)
        print('  等待3秒...')
        time.sleep(3)
        
        # 步骤4: 张开夹爪
        print('\n[步骤4] 张开夹爪')
        print('  夹爪张开到 0.010')
        self.send_gripper(0.010, 1.0)
        print('  等待2秒...')
        time.sleep(2)
        
        print('\n' + '='*50)
        print('🎉 抓取流程完成！')
        print('='*50)
        
        # 保持节点运行
        print('\n程序继续运行，可按 Ctrl+C 退出')

def main(args=None):
    rclpy.init(args=args)
    node = VisionGrasp()
    rclpy.spin(node)

if __name__ == '__main__':
    main()