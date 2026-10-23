import rclpy
import csv
import os
import math

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path

class Evaluator(Node):
    def __init__(self, node_name="evaluator"):
        super().__init__(node_name)

        # parameter declaration
        self.declare_parameter("scenario", "S1")
        self.scenario_ = self.get_parameter("scenario").value
        self.declare_parameter("config", "test")
        self.config_ = self.get_parameter("config").value
        self.declare_parameter("run", 1)
        self.run_ = self.get_parameter("run").value
        self.declare_parameter("log_dir", "~/ee3305/logs")
        self.log_dir_ = self.get_parameter("log_dir").value

        # create log folder
        logs_dir = os.path.expanduser(self.log_dir_)
        self.run_dir_ = os.path.join(logs_dir, f"{self.config_}_{self.scenario_}_{self.run_}")
        os.makedirs(self.run_dir_, exist_ok=True)
        self.get_logger().info(f"logging to {self.run_dir_}")
        self.files_ = []

        # create odom.csv
        self.odom_csv_ = self.openCsv_("odom.csv", ["t", "x", "y", "yaw", "v", "w"])
        
        # create cmd_vel.csv
        self.cmd_vel_csv_ = self.openCsv_("cmd_vel.csv", ["t", "v", "w"])

        # goal.csv
        self.goal_csv_ = self.openCsv_("goal.csv", ["t", "x", "y"])

        # path_request.csv
        self.path_request_csv_ = self.openCsv_("path_request.csv", ["t", "rx", "ry", "gx", "gy"])

        # path.csv
        self.path_csv_ = self.openCsv_("path.csv", ["t", "path_id", "i", "x", "y"])
        self.path_count_ = 0

        # Odometry subscriber
        self.sub_odom_ = self.create_subscription(
            Odometry,
            "odom",
            self.callbackSubOdom_,
            qos_profile_sensor_data
        )

        # cmd_vel subscriber
        self.sub_cmd_vel_ = self.create_subscription(
            TwistStamped,
            "cmd_vel",
            self.callbackSubCmdVel_,
            10
        )

        # Goal pose subscriber
        self.sub_goal_ = self.create_subscription(
            PoseStamped,
            "goal_pose",
            self.callbackSubGoalPose_,
            10
        )

        # Path Request subscriber
        self.sub_path_request_ = self.create_subscription(
            Path,
            "path_request",
            self.callbackSubPathRequest_,
            10
        )

        # Path subscriber
        self.sub_path_ = self.create_subscription(
            Path,
            "path",
            self.callbackSubPath_,
            10
        )
        
    # csv creator helper function
    def openCsv_(self, name, header):
        file_ = open(os.path.join(self.run_dir_, name), "w", newline="", buffering=1)
        csv_ = csv.writer(file_)
        csv_.writerow(header)
        self.files_.append(file_)
        return csv_

    # Odometry subscriber callback
    def callbackSubOdom_(self, msg: Odometry):
        t = self.get_clock().now().nanoseconds * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.odom_csv_.writerow([t, x, y, yaw, v, w])

    #cmd_vel subscriber callback
    def callbackSubCmdVel_(self, msg: TwistStamped):
        t = self.get_clock().now().nanoseconds * 1e-9
        v = msg.twist.linear.x
        w = msg.twist.angular.z
        self.cmd_vel_csv_.writerow([t, v, w])

    # Goal Pose subscriber callback
    def callbackSubGoalPose_(self, msg: PoseStamped):
        t = self.get_clock().now().nanoseconds * 1e-9
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.goal_csv_.writerow([t, x, y])

    # Path request subscriber callback
    def callbackSubPathRequest_(self, msg: Path):
        if len(msg.poses) < 2:
            self.get_logger().info(f"Path length is less than 2")
            return
        else:
            t = self.get_clock().now().nanoseconds * 1e-9
            rx = msg.poses[0].pose.position.x
            ry = msg.poses[0].pose.position.y
            gx = msg.poses[1].pose.position.x
            gy = msg.poses[1].pose.position.y
            self.path_request_csv_.writerow([t, rx, ry, gx, gy])

    # Path subscriber callback
    def callbackSubPath_(self, msg: Path):
        t = self.get_clock().now().nanoseconds * 1e-9
        self.path_count_ += 1
        for i, pose in enumerate(msg.poses):
            x = pose.pose.position.x
            y = pose.pose.position.y
            self.path_csv_.writerow([t, self.path_count_, i, x, y])

def main(args=None):
    rclpy.init(args=args)
    node = Evaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for f in node.files_:
            f.close()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()

