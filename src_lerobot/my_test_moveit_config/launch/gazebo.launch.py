#!/usr/bin/env python3
"""
Gazebo仿真启动文件 - 适配so101机械臂
"""
import os
import re
import subprocess
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 包路径
    pkg_my_test_description = get_package_share_directory('my_test_description')
    pkg_my_test_moveit_config = get_package_share_directory('my_test_moveit_config')
    
    # URDF文件路径
    urdf_file = os.path.join(pkg_my_test_description, 'urdf', 'so101.urdf')
    
    # 读取URDF内容
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()
    
    # 移除XML声明
    robot_description_content = re.sub(r'<\?xml.*?\?>', '', robot_description_content)
    
    # 替换mesh路径为绝对路径（Gazebo需要绝对路径）
    robot_description_content = robot_description_content.replace(
        'package://my_test_description',
        f'file://{pkg_my_test_description}'
    )
    
    # 控制器配置文件路径 - 使用MoveIt的配置
    controller_config = os.path.join(
        pkg_my_test_moveit_config, 
        'config', 
        'ros2_controllers.yaml'
    )
    
    # 如果MoveIt的配置不存在，使用本地的
    if not os.path.exists(controller_config):
        controller_config = os.path.join(
            pkg_my_test_description, 
            'config', 
            'ros2_controllers.yaml'
        )
    
    print(f"Using controller config: {controller_config}")
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )
    
    # 启动Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Spawn机器人
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'so101_robot',
            '-x', '0',
            '-y', '0',
            '-z', '0.1',  # 提高一点避免穿模
            '-R', '0',
            '-P', '0',
            '-Y', '0'
        ],
        output='screen'
    )
    
    # 加载 joint_state_broadcaster - 使用正确的控制器名称
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",  # 这是配置文件中的名称
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "10"
        ],
        output='screen'
    )
    
    # 加载 so_arm_101_groups_controller - 使用正确的控制器名称
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "so_arm_101_groups_controller",  # 这是配置文件中的名称
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "10"
        ],
        output='screen'
    )
    
    # 使用事件处理确保控制器按顺序加载
    # 先等待 spawn_entity 完成
    spawn_entity_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    
    # 加载 joint_state_broadcaster 后再加载 arm_controller
    broadcaster_exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )
    
    # 添加一个静态变换发布器（用于RViz）
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_world_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base'],
        output='screen'
    )
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # 1. 启动robot_state_publisher
        robot_state_publisher,
        
        # 2. 启动Gazebo
        gazebo,
        
        # 3. 等待5秒后spawn机器人
        TimerAction(
            period=5.0,
            actions=[spawn_entity]
        ),
        
        # 4. 添加事件处理器
        spawn_entity_handler,
        broadcaster_exit_handler,
        
        # 5. 可选的静态变换（如果需要）
        # static_transform_publisher,
    ])