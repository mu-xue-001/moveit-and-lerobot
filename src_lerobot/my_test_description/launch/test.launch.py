# demo.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    # ===============================
    # 1️⃣ 构建 MoveIt 配置
    # ===============================
    # "so101_new_calib" 是你 MoveIt2 配置文件夹名
    # "my_test_moveit_config" 是你 workspace 中的包名
    moveit_config = MoveItConfigsBuilder(
        "so101_new_calib",
        package_name="my_test_moveit_config"
    ).to_moveit_configs()

    # ===============================
    # 2️⃣ 生成 MoveIt demo launch
    # ===============================
    # 这里包含 robot_state_publisher、planning_pipeline、RViz 等
    demo_launch = generate_demo_launch(moveit_config)

    # ===============================
    # 3️⃣ 可选：启动 TrajFollower 桥接节点
    # ===============================
    traj_bridge_node = Node(
        package="my_test_description",  # 你的桥接节点所在包名
        executable="traj_to_jointstate",  # 脚本文件名去掉 .py
        name="traj_follower_bridge",
        output="screen",
        parameters=[
            {"port": "/dev/ttyACM0"}  # 串口参数，可根据你的机械臂修改
        ]
    )

    # ===============================
    # 4️⃣ 返回 LaunchDescription
    # ===============================
    return LaunchDescription([
        *demo_launch.entities,  # MoveIt demo
        traj_bridge_node         # 桥接节点
    ])