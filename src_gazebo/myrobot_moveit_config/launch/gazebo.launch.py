import os
import re
import xacro

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

def generate_launch_description():
    robot_name_in_model = 'my_robot'
    package_name = 'myrobot_description'
    urdf_name = "my_robot.urdf.xacro"

    # 支撑台参数（尺寸：0.15 x 0.15 x 0.05）
    stand_urdf_name = "stand.urdf"
    stand_model_name = "stand"
    stand_x, stand_y, stand_z = 0.90, 0.0, 0.35  # 台子中心位置
    
    # 小方块参数（尺寸：0.03 x 0.03 x 0.03）
    cube_urdf_name = "cube.urdf"
    cube_model_name = "cube"
    # 台子高度 0.05m，方块高度 0.03m
    # 方块中心应该放在台面上方 0.015m 处
    # 台面高度 = stand_z + 台子半高 = 0.35 + 0.025 = 0.375
    # 方块中心 = 台面高度 + 方块半高 = 0.375 + 0.015 = 0.39
    cube_x, cube_y, cube_z = 0.90, 0.0, 0.39

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, 'urdf', urdf_name)
    stand_urdf_path = os.path.join(pkg_share, 'urdf', stand_urdf_name)
    cube_urdf_path = os.path.join(pkg_share, 'urdf', cube_urdf_name)

    # 启动 Gazebo
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', 
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             '-s', 'libgazebo_ros_force_system.so'],  
        output='screen'
    )

    # 处理 URDF
    robot_description_config = xacro.process_file(urdf_model_path)
    robot_description = {'robot_description': remove_comments(robot_description_config.toxml())}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, robot_description, {"publish_frequency": 50.0}],
        output='screen'
    )

    # 生成机械臂
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, '-topic', 'robot_description'],
        output='screen'
    )

    # 生成支撑台
    spawn_stand_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', stand_urdf_path,
            '-entity', stand_model_name,
            '-x', str(stand_x),
            '-y', str(stand_y),
            '-z', str(stand_z),
            '-R', '0', '-P', '0', '-Y', '0'
        ],
        output='screen'
    )

    # 延迟1秒生成小方块（确保台子已稳定）
    delayed_spawn_cube = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file', cube_urdf_path,
                    '-entity', cube_model_name,
                    '-x', str(cube_x),
                    '-y', str(cube_y),
                    '-z', str(cube_z),
                    '-R', '0', '-P', '0', '-Y', '0'
                ],
                output='screen'
            )
        ]
    )

    # 延迟加载控制器
    def load_controllers():
        return [
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen'
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller'],
                output='screen'
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['gripper_controller'],
                output='screen'
            )
        ]

    delayed_load_controllers = TimerAction(
        period=3.0,
        actions=load_controllers()
    )

    # LaunchDescription
    ld = LaunchDescription()
    ld.add_action(start_gazebo_cmd)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(spawn_stand_cmd)        # 先生成台子
    ld.add_action(delayed_spawn_cube)     # 1秒后生成方块
    ld.add_action(delayed_load_controllers)  # 3秒后加载控制器

    return ld