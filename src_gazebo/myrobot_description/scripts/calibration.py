#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import cv2
import numpy as np
from builtin_interfaces.msg import Duration
import time
import math
import json
import os

class Calibration(Node):
    def __init__(self):
        super().__init__('calibration')
        
        self.arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.gripper_joints = ['gripper_left_finger_joint']
        
        self.bridge = CvBridge()
        
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        
        print('等待控制器...')
        self.arm_client.wait_for_server()
        print('控制器已连接')
        
        # 订阅关节状态
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        self.current_joint2 = None
        self.current_joint3 = None
        self.current_pixel = None
        self.detected = False
        self.data_saved = False
        
        print('='*60)
        print('标定程序 - 自动记录模式')
        print('='*60)
        
        # 检查是否已有数据
        self.data_file = os.path.expanduser('~/myrobt_ws/calib_data.json')
        if os.path.exists(self.data_file):
            with open(self.data_file, 'r') as f:
                self.existing_data = json.load(f)
            print(f'已有数据: {list(self.existing_data.keys())}')
            if 'point1' in self.existing_data and 'point2' not in self.existing_data:
                print('请移动机械臂到成功抓取位置，程序将自动记录点2')
            elif 'point1' in self.existing_data and 'point2' in self.existing_data:
                print('已有两个点，将重新计算参数')
        else:
            self.existing_data = {}
            print('请确保机械臂在观测位置，立方体在视野中')
            print('程序将自动记录点1')
        
        input('按回车开始...')
        
        # 启动相机
        self.image_sub = self.create_subscription(Image, '/camera_sensor/image_raw', self.image_callback, 10)
        print('相机已启动，等待检测立方体...')
    
    def joint_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == 'joint2':
                self.current_joint2 = msg.position[i] * 180 / math.pi
            elif name == 'joint3':
                self.current_joint3 = msg.position[i] * 180 / math.pi
    
    def image_callback(self, msg):
        if self.data_saved:
            return
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            mask1 = cv2.inRange(hsv, np.array([0, 43, 46]), np.array([10, 255, 255]))
            mask2 = cv2.inRange(hsv, np.array([156, 43, 46]), np.array([180, 255, 255]))
            mask = cv2.bitwise_or(mask1, mask2)
            
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            debug_img = frame.copy()
            
            if contours:
                largest = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest)
                
                if area > 500:
                    rect = cv2.boundingRect(largest)
                    u = rect[0] + rect[2] / 2.0
                    v = rect[1] + rect[3] / 2.0
                    
                    self.current_pixel = (u, v)
                    
                    # 绘制检测框
                    cv2.rectangle(debug_img, (rect[0], rect[1]), 
                                  (rect[0]+rect[2], rect[1]+rect[3]), (0,255,0), 3)
                    cv2.circle(debug_img, (int(u), int(v)), 8, (0,0,255), -1)
                    cv2.putText(debug_img, f"Cube at ({u:.0f}, {v:.0f})", (rect[0], rect[1]-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                    
                    # 显示关节角度
                    if self.current_joint2 is not None:
                        cv2.putText(debug_img, f"joint2={self.current_joint2:.1f}°, joint3={self.current_joint3:.1f}°", 
                                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
                    
                    cv2.imshow("Calibration", debug_img)
                    cv2.waitKey(1)
                    
                    # 自动保存（检测到数据后延迟2秒保存）
                    if self.current_joint2 is not None and self.current_pixel is not None and not self.detected:
                        self.detected = True
                        self.get_logger().info('检测到立方体，2秒后自动保存...')
                        self.timer = self.create_timer(2.0, self.save_data)
                    return
            
            # 未检测到立方体
            cv2.putText(debug_img, "No cube detected", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
            cv2.putText(debug_img, "Make sure the red cube is in view", (20, 70), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
            cv2.imshow("Calibration", debug_img)
            cv2.waitKey(1)
            
        except Exception as e:
            print(f'错误: {e}')
    
    def save_data(self):
        self.timer.cancel()
        
        data = {
            'pixel': self.current_pixel,
            'joint2': self.current_joint2,
            'joint3': self.current_joint3
        }
        
        if 'point1' not in self.existing_data:
            self.existing_data['point1'] = data
            print(f'\n✓ 点1已保存: 像素({data["pixel"][0]:.1f}, {data["pixel"][1]:.1f})')
            print(f'  关节: joint2={data["joint2"]:.1f}°, joint3={data["joint3"]:.1f}°')
            print('\n请移动机械臂到成功抓取位置，然后重新运行此程序')
            
            with open(self.data_file, 'w') as f:
                json.dump(self.existing_data, f, indent=2)
            
            cv2.destroyAllWindows()
            rclpy.shutdown()
            exit(0)
            
        elif 'point2' not in self.existing_data:
            self.existing_data['point2'] = data
            print(f'\n✓ 点2已保存: 像素({data["pixel"][0]:.1f}, {data["pixel"][1]:.1f})')
            print(f'  关节: joint2={data["joint2"]:.1f}°, joint3={data["joint3"]:.1f}°')
            
            self.calculate_params(self.existing_data['point1'], self.existing_data['point2'])
        
        self.data_saved = True
    
    def calculate_params(self, p1, p2):
        # 已知成功抓取位置
        GRASP_JOINT2 = 74.0
        GRASP_JOINT3 = 80.0
        
        du = p2['pixel'][0] - p1['pixel'][0]
        dv = p2['pixel'][1] - p1['pixel'][1]
        
        # 关节偏差
        d2 = GRASP_JOINT2 - p1['joint2']
        d3 = GRASP_JOINT3 - p1['joint3']
        
        # 修正系数
        K_u = d2 / du if du != 0 else 0
        K_v = d3 / dv if dv != 0 else 0
        
        params = {
            'pixel_ref': p1['pixel'],
            'joint_ref': [p1['joint2'], p1['joint3']],
            'pixel_target': p2['pixel'],
            'joint_target': [GRASP_JOINT2, GRASP_JOINT3],
            'K_u': K_u,
            'K_v': K_v
        }
        
        calib_file = os.path.expanduser('~/myrobt_ws/calibration.json')
        with open(calib_file, 'w') as f:
            json.dump(params, f, indent=2)
        
        print('\n' + '='*60)
        print('标定完成！修正参数已保存')
        print('='*60)
        print(f'观测位置像素: ({p1["pixel"][0]:.1f}, {p1["pixel"][1]:.1f})')
        print(f'观测位置关节: ({p1["joint2"]:.1f}°, {p1["joint3"]:.1f}°)')
        print(f'抓取位置像素: ({p2["pixel"][0]:.1f}, {p2["pixel"][1]:.1f})')
        print(f'抓取位置关节: ({GRASP_JOINT2:.1f}°, {GRASP_JOINT3:.1f}°)')
        print(f'修正系数: K_u={K_u:.4f}, K_v={K_v:.4f}')
        print('='*60)
        
        # 删除临时数据文件
        os.remove(self.data_file)
        
        cv2.destroyAllWindows()
        rclpy.shutdown()
        exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = Calibration()
    rclpy.spin(node)

if __name__ == '__main__':
    main()