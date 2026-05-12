#!/usr/bin/env python3
"""
智能金属管跟随节点
- 自动检测夹爪与金属管的接触
- 接触时自动启用跟随
- 分离时自动禁用跟随
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Twist, Point
from tf2_ros import TransformListener, Buffer
import math

class SmartPipeFollower(Node):
    def __init__(self):
        super().__init__('smart_pipe_follower')
        
        # 订阅 Gazebo 的接触信息
        self.contact_sub = self.create_subscription(
            ContactsState,
            '/gazebo/default/contacts',  # Gazebo 接触话题
            self.contact_callback,
            10
        )
        
        # 创建 TF 监听器（用于获取夹爪位置）
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 创建 Gazebo 服务客户端
        self.set_state_client = self.create_client(SetModelState, '/gazebo/set_model_state')
        self.get_state_client = self.create_client(GetModelState, '/gazebo/get_model_state')
        
        # 等待服务可用
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Gazebo services...')
        while not self.get_state_client.wait_for_service(timeout_sec=1.0):
            pass
        
        # 状态变量
        self.is_contacting = False  # 是否接触
        self.pipe_attached = False  # 是否已启用跟随
        self.last_pipe_pos = None   # 上次金属管位置
        
        # 夹爪的 link 名称（根据你的 URDF 修改）
        # 请根据你的实际 URDF 修改这些名称
        self.gripper_links = [
            'gripper_left_finger_link',
            'gripper_right_finger_link',
            'gripper_base_link',
            'hand_link',
            'tool_link'
        ]
        
        # 金属管名称
        self.pipe_name = 'metal_pipe'
        
        # 创建定时器，持续更新金属管位置（如果接触）
        self.create_timer(0.05, self.update_pipe_position)
        
        self.get_logger().info('Smart pipe follower ready!')
        self.get_logger().info('Will automatically follow when gripper contacts pipe')
    
    def contact_callback(self, msg):
        """处理接触信息，检测夹爪是否接触金属管"""
        
        contacting = False
        
        # 检查所有接触
        for contact in msg.states:
            # 获取接触的两个物体
            coll1 = contact.collision1
            coll2 = contact.collision2
            
            # 检查是否涉及金属管和夹爪
            pipe_involved = (self.pipe_name in coll1) or (self.pipe_name in coll2)
            
            gripper_involved = False
            for link in self.gripper_links:
                if link in coll1 or link in coll2:
                    gripper_involved = True
                    break
            
            # 如果同时涉及金属管和夹爪，说明正在接触
            if pipe_involved and gripper_involved:
                contacting = True
                break
        
        # 更新状态
        if contacting and not self.is_contacting:
            # 刚接触上
            self.is_contacting = True
            self.pipe_attached = True
            self.get_logger().info('Pipe contacted! Enabling automatic following...')
            
            # 禁用金属管重力（让它跟随）
            self.disable_pipe_gravity()
            
        elif not contacting and self.is_contacting:
            # 刚分离
            self.is_contacting = False
            self.pipe_attached = False
            self.get_logger().info('Pipe released! Disabling following...')
            
            # 恢复金属管重力（让它正常物理掉落）
            self.enable_pipe_gravity()
    
    def disable_pipe_gravity(self):
        """禁用金属管重力，使其可以跟随"""
        try:
            req = SetModelState.Request()
            req.model_state.model_name = self.pipe_name
            req.model_state.twist.linear.x = 0.0
            req.model_state.twist.linear.y = 0.0
            req.model_state.twist.linear.z = 0.0
            req.model_state.twist.angular.x = 0.0
            req.model_state.twist.angular.y = 0.0
            req.model_state.twist.angular.z = 0.0
            
            self.set_state_client.call_async(req)
            self.get_logger().debug('Pipe gravity disabled')
            
        except Exception as e:
            self.get_logger().error(f'Failed to disable pipe gravity: {e}')
    
    def enable_pipe_gravity(self):
        """恢复金属管重力"""
        # 注意：Gazebo 没有直接恢复重力的服务
        # 这里通过设置一个很小的速度来让物理引擎重新激活
        try:
            # 获取金属管当前位置
            get_req = GetModelState.Request()
            get_req.model_name = self.pipe_name
            future = self.get_state_client.call_async(get_req)
            
            # 恢复重力只需要不再设置位置，让物理引擎接管
            # 所以什么都不做即可
            self.get_logger().debug('Pipe gravity enabled')
            
        except Exception as e:
            self.get_logger().error(f'Failed to enable pipe gravity: {e}')
    
    def update_pipe_position(self):
        """更新金属管位置，使其跟随夹爪"""
        if not self.pipe_attached:
            return  # 未附着，不更新
        
        try:
            # 获取夹爪位置（使用夹爪基座或手指的位置）
            # 尝试多个 link，直到成功
            gripper_pos = None
            gripper_rot = None
            
            for link in self.gripper_links:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        'world',
                        link,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.05)
                    )
                    gripper_pos = transform.transform.translation
                    gripper_rot = transform.transform.rotation
                    break
                except:
                    continue
            
            if gripper_pos is None:
                return
            
            # 计算金属管位置
            # 假设金属管在夹爪中心，夹爪闭合后金属管在夹爪之间
            # 根据你的夹爪几何调整这个偏移
            pipe_x = gripper_pos.x
            pipe_y = gripper_pos.y
            pipe_z = gripper_pos.z - 0.04  # 夹爪下方4cm
            
            # 如果金属管位置变化太大，平滑处理
            if self.last_pipe_pos:
                # 限制最大移动速度，避免跳跃
                max_move = 0.05  # 最大移动5cm/步
                pipe_x = self.smooth_move(self.last_pipe_pos.x, pipe_x, max_move)
                pipe_y = self.smooth_move(self.last_pipe_pos.y, pipe_y, max_move)
                pipe_z = self.smooth_move(self.last_pipe_pos.z, pipe_z, max_move)
            
            # 设置金属管位置
            req = SetModelState.Request()
            req.model_state.model_name = self.pipe_name
            req.model_state.reference_frame = 'world'
            req.model_state.pose.position.x = pipe_x
            req.model_state.pose.position.y = pipe_y
            req.model_state.pose.position.z = pipe_z
            req.model_state.pose.orientation = gripper_rot
            req.model_state.twist = Twist()  # 速度为0，禁用物理
            
            self.set_state_client.call_async(req)
            
            # 记录位置
            self.last_pipe_pos = Point(x=pipe_x, y=pipe_y, z=pipe_z)
            
        except Exception as e:
            # 忽略 TF 获取失败的错误
            pass
    
    def smooth_move(self, current, target, max_step):
        """平滑移动，避免跳跃"""
        diff = target - current
        if abs(diff) > max_step:
            return current + max_step * (1 if diff > 0 else -1)
        return target

def main(args=None):
    rclpy.init(args=args)
    node = SmartPipeFollower()
    rclpy.spin(node)

if __name__ == '__main__':
    main()