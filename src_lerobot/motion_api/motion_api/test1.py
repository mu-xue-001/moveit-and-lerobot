import time
import rclpy
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


if __name__ == "__main__":
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    path = __file__.split('motion_api/test')[0] + 'config/moveit_cpp.yaml'
    print(f'moveit cpp config path is :{path}')

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="so101_new_calib", package_name="my_test_moveit_config"
        )
        .moveit_cpp(path)
        .to_moveit_configs()
    )

    params = moveit_config.to_dict()
    robot = MoveItPy(node_name="moveit_py", config_dict=params)
    arm_group = robot.get_planning_component("so_arm_101_groups")
    logger.info("MoveItPy instance created")
    print(robot)

    robot_model = robot.get_robot_model()
    robot_state = RobotState(robot_model)

    # ==================== 动作一 ====================
    first_goal_radians = [0.1222, 0.2967, 0.1919, 0.8727, 0.0, 0.3840]
    robot_state.set_joint_group_positions('so_arm_101_groups', first_goal_radians)
    arm_group.set_goal_state(robot_state=robot_state)
    arm_group.set_start_state_to_current_state()
    plan_and_execute(robot, arm_group, logger, sleep_time=3.0)

    # ==================== 动作二 ====================
    second_goal_radians = first_goal_radians[:5] + [0.0098]
    robot_state.set_joint_group_positions('so_arm_101_groups', second_goal_radians)
    arm_group.set_goal_state(robot_state=robot_state)
    arm_group.set_start_state_to_current_state()
    plan_and_execute(robot, arm_group, logger, sleep_time=3.0)

    # ==================== 动作三 ====================
    third_goal_radians = [0.4712, -0.0349, -0.1571, 0.6283, 0.0, 0.0098]
    robot_state.set_joint_group_positions('so_arm_101_groups', third_goal_radians)
    arm_group.set_goal_state(robot_state=robot_state)
    arm_group.set_start_state_to_current_state()
    plan_and_execute(robot, arm_group, logger, sleep_time=3.0)
        # ==================== 动作四 ====================
    fourth_goal_radians = third_goal_radians[:5] + [0.3840]  # 第六关节改为22度
    robot_state.set_joint_group_positions('so_arm_101_groups', fourth_goal_radians)
    arm_group.set_goal_state(robot_state=robot_state)
    arm_group.set_start_state_to_current_state()
    plan_and_execute(robot, arm_group, logger, sleep_time=3.0)