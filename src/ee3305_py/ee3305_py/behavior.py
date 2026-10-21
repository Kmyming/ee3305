from math import hypot

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


class Behavior(Node):

    def __init__(self, node_name="behavior"):
        # Node Constructor =============================================================
        super().__init__(node_name)

        # Parameters: Declare
        self.declare_parameter("frequency", float(10))
        self.declare_parameter("plan_frequency", float(2))

        # Parameters: Get Values
        self.frequency_ = self.get_parameter("frequency").value
        self.plan_frequency_ = self.get_parameter("plan_frequency").value

        # Handles: Topic Subscribers
        # !TODO: Goal pose subscriber
        self.sub_goal_ = self.create_subscription(
            PoseStamped, "goal_pose", self.callbackSubGoalPose_, 10)

        # !TODO: Odometry subscriber
        self.sub_odom_ = self.create_subscription(
            Odometry, "odom", self.callbackSubOdom_, qos_profile_sensor_data)

        # Handles: Topic Publishers
        # !TODO: Path request publisher
        self.pub_path_request_ = self.create_publisher(Path, "path_request", 10)

        # Handles: Timers
        self.timer = self.create_timer(1.0 / self.frequency_, self.callbackTimer_)

        self.timer_plan_ = self.create_timer(
            1.0 / self.plan_frequency_, self.callbackTimerPlan_
        )

        # Other Instance Variables
        self.received_goal_coords_ = False
        self.received_rbt_coords_ = False
        self.goal_reached_ = True

    # Callbacks =============================================================

    # Goal pose subscriber callback.
    def callbackSubGoalPose_(self, msg: PoseStamped):
        self.received_goal_coords_ = True
        self.goal_reached_ = (
            True  # "cancel" the goal and trigger the if condition in timer.
        )

        # !TODO: Copy to goal_x_, goal_y_.
        self.goal_x_ = msg.pose.position.x
        self.goal_y_ = msg.pose.position.y

        self.get_logger().info(
            f"Received New Goal @ ({self.goal_x_:7.3f}, {self.goal_y_:7.3f})."
        )

    # Odometry subscriber callback.
    def callbackSubOdom_(self, msg: Odometry):
        self.received_rbt_coords_ = True

        # !TODO: Copy to rbt_x_, rbt_y_.
        self.rbt_x_ = msg.pose.pose.orientation.x
        self.rbt_y_ = msg.pose.pose.position.y

    # Callback for timer.
    # Normally the decisions of the robot system are made here, and this callback is dramatically simplified.
    # The callback contains some example code for waypoint detection.
    def callbackTimer_(self):
        if not self.received_rbt_coords_ or not self.received_goal_coords_:
            return  # silently return if none of the coords are received from the subscribers.

        dx = self.goal_x_ - self.rbt_x_
        dy = self.goal_y_ - self.rbt_y_

        goal_is_close = hypot(dx, dy) < 0.1

        if goal_is_close and not self.goal_reached_:
            self.get_logger().info(
                f"Reached Goal @ ({self.goal_x_:7.3f}, {self.goal_y_:7.3f})."
            )
            self.goal_reached_ = True
        elif not goal_is_close and self.goal_reached_:
            self.get_logger().info(
                f"Going to Goal @ ({self.goal_x_:7.3f}, {self.goal_y_:7.3f})."
            )
            self.goal_reached_ = False

    # Callback for publishing path requests between clicked_point (goal) and robot position.
    # Normally path requests are implemented with ROS2 service, and the service is called in the main timer.
    # To keep things simple for this course, we use only ROS2 tpics.
    def callbackTimerPlan_(self):
        if not self.received_goal_coords_ or not self.received_rbt_coords_:
            return  # silently return if none of the coords are received from the subscribers

        # Create a new message for publishing
        msg_path_request = Path()
        msg_path_request.header.stamp = self.get_clock().now().to_msg()
        msg_path_request.header.frame_id = "map"

        # !TODO: write the robot coordinates
        rbt_pose = PoseStamped()
        rbt_pose.pose.position.x = self.rbt_x_
        rbt_pose.pose.position.y = self.rbt_y_

        # !TODO: write the goal coordinates
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = self.goal_x_
        goal_pose.pose.position.y = self.goal_y_        
        
        # !TODO: fill up the array containing the robot coordinates at [0] and goal coordinates at [1]
        msg_path_request.poses.append(rbt_pose)
        msg_path_request.poses.append(goal_pose)

        # publish the message
        self.get_logger().info(
            f"Sending Path Planning Request from Rbt @ ({self.rbt_x_:7.3f}, {self.rbt_y_:7.3f}) to Goal @ ({self.goal_x_:7.3f}, {self.goal_y_:7.3f})"
        )
        self.pub_path_request_.publish(msg_path_request)


# Main Boiler Plate =============================================================
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Behavior())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
