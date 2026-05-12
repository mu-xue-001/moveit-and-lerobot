import rclpy
from rclpy.node import Node

from moveit.planning import MoveItPy


class MoveItJointGoal(Node):
    def __init__(self):
        super().__init__("moveit_joint_goal_py")

        # 初始化 MoveItPy
        self.moveit = MoveItPy(node_name="moveit_py_node")

        # 你的 planning group
        self.arm = self.moveit.get_planning_component("so_arm_101_groups")

    def go(self, joints):
        if len(joints) != 6:
            self.get_logger().error("必须是6个关节角度")
            return

        self.get_logger().info(f"目标关节: {joints}")

        # 设置目标关节状态
        self.arm.set_goal_state(configuration_name=joints)

        # 规划
        plan_result = self.arm.plan()

        if not plan_result:
            self.get_logger().error("规划失败")
            return

        self.get_logger().info("执行轨迹...")
        self.moveit.execute(plan_result.trajectory, controllers=["so_arm_101_groups_controller"])

        self.get_logger().info("完成")


def main():
    rclpy.init()

    node = MoveItJointGoal()

    # 示例角度（rad）
    target = [0.0, -0.5, 0.3, 0.0, 0.2, 0.5]

    node.go(target)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()