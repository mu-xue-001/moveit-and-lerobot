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

class Grasp(Node):
    def __init__(self):
        super().__init__('grasp')
        
        self.arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.gripper_joints = ['gripper_left_finger_joint']
        
        # 抓取位置（保持不变）: [0, 74, 80, 90, 0, 0]
        self.GRASP_JOINTS = [
            0.0,
            74.0 * math.pi / 180.0,  # joint2: 74°
            80.0 * math.pi / 180.0,  # joint3: 80°
            90.0 * math.pi / 180.0,  # joint4: 90°
            0.0,
            0.0
        ]
        
        # 抬起位置（修改关节2和关节3）: [0, 60, 70, 90, 0, 0]
        self.LIFT_JOINTS = [
            0.0,
            60.0 * math.pi / 180.0,  # joint2: 60°
            70.0 * math.pi / 180.0,  # joint3: 70°
            90.0 * math.pi / 180.0,  # joint4: 90°
            0.0,
            0.0
        ]
        
        self.bridge = CvBridge()
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        
        self.image_sub = self.create_subscription(Image, '/camera_sensor/image_raw', self.image_callback, 10)
        self.detected = False
        print('等待检测红色立方体...')
        print(f'抓取位置: joint2=74°, joint3=80°, joint4=90°')
        print(f'抬起位置: joint2=60°, joint3=70°, joint4=90°')
    
    def image_callback(self, msg):
        if self.detected:
            return
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            
            if area > 300:
                print('✓ 检测到位置')
                
                x, y, w, h = cv2.boundingRect(largest)
                debug_img = cv_image.copy()
                cv2.rectangle(debug_img, (x,y), (x+w,y+h), (0,255,0), 3)
                cv2.circle(debug_img, (x+w//2, y+h//2), 8, (0,0,255), -1)
                cv2.putText(debug_img, "Cube Detected!", (x, y-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.imshow("Detection", debug_img)
                cv2.waitKey(2000)
                cv2.destroyAllWindows()
                
                self.detected = True
                self.image_sub.destroy()
                self.timer = self.create_timer(1.0, self.execute_grasp)
                return
        
        debug_img = cv_image.copy()
        cv2.putText(debug_img, "Searching for red cube...", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
        cv2.imshow("Detection", debug_img)
        cv2.waitKey(1)
    
    def send_arm(self, positions, duration=3.0):
        traj = JointTrajectory()
        traj.joint_names = self.arm_joints
        
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=int(duration), nanosec=0)
        traj.points.append(point)
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)
        
        self.arm_client.send_goal_async(goal)
        
    def send_gripper(self, position, duration=1.0):
        traj = JointTrajectory()
        traj.joint_names = self.gripper_joints
        
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = Duration(sec=int(duration), nanosec=0)
        traj.points.append(point)
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)
        
        self.gripper_client.send_goal_async(goal)
    
    def execute_grasp(self):
        self.timer.cancel()
        
        print('='*40)
        
        # 1. 移动到抓取位置
        print('移动到抓取位置')
        print('  joint2=74°, joint3=80°, joint4=90°')
        self.send_arm(self.GRASP_JOINTS, 3.0)
        time.sleep(5)
        
        # 2. 抓取（闭合夹爪）
        print('抓取')
        self.send_gripper(0.036, 1.0)
        time.sleep(3)
        
        # 3. 抬起机械臂
        print('抬起机械臂')
        print('  joint2=60°, joint3=70°, joint4=90°')
        self.send_arm(self.LIFT_JOINTS, 2.0)
        time.sleep(3)
        
        # 4. 张开夹爪
        print('张开夹爪')
        self.send_gripper(0.010, 1.0)
        time.sleep(2)
        
        print('完成')
        print('='*40)

def main(args=None):
    rclpy.init(args=args)
    node = Grasp()
    rclpy.spin(node)

if __name__ == '__main__':
    main()