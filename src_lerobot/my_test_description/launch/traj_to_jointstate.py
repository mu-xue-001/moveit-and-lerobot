#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
from builtin_interfaces.msg import Time
import time

class MoveIt2ToFeetechBridge(Node):
    def __init__(self):
        super().__init__("moveit2_to_feetech_bridge")
        self.subscription = self.create_subscription(
            DisplayTrajectory,
            "/display_planned_path",
            self.trajectory_cb,
            10
        )
        self.publisher = self.create_publisher(JointState, "so101_follower/joint_commands", 10)
        self.get_logger().info("MoveIt2 -> Feetech bridge ready.")

    def trajectory_cb(self, msg: DisplayTrajectory):
        """
        Callback for DisplayTrajectory.
        Sends trajectory points sequentially to Feetech bridge.
        """
        if not msg.trajectory:
            self.get_logger().warn("Empty trajectory received")
            return

        joint_traj = msg.trajectory[0].joint_trajectory
        self.get_logger().info(f"Received trajectory with {len(joint_traj.points)} points.")

        start_time = self.get_clock().now().nanoseconds / 1e9

        for point in joint_traj.points:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = joint_traj.joint_names
            js.position = point.positions

            # Publish
            self.publisher.publish(js)

            # Sleep until next point time_from_start
            tfs = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            now = self.get_clock().now().nanoseconds / 1e9
            sleep_time = (start_time + tfs) - now
            if sleep_time > 0:
                time.sleep(sleep_time)

def main():
    rclpy.init()
    node = MoveIt2ToFeetechBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()