import rclpy
import csv
import os
import math
import yaml

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from ament_index_python.packages import get_package_share_directory

class Evaluator(Node):
    def __init__(self, node_name="evaluator"):
        super().__init__(node_name)

        # instance variables
        self.goal_sent_ = False
        self.t_goal_ = None
        self.rbt_x_ = None
        self.rbt_y_ = None
        self.done_ = False
        self.result_ = None
        self.elapsed_ = None
        self.distance_ = None

        # parameter declaration
        self.declare_parameter("goal_tol", 0.15)
        self.goal_tol_ =  self.get_parameter("goal_tol").value
        self.declare_parameter("scenario", "S1")
        self.scenario_ = self.get_parameter("scenario").value
        self.declare_parameter("config", "test")
        self.config_ = self.get_parameter("config").value
        self.declare_parameter("run", 1)
        self.run_ = self.get_parameter("run").value
        self.declare_parameter("log_dir", "~/ee3305/logs")
        self.log_dir_ = self.get_parameter("log_dir").value

        # load scenarios
        share = get_package_share_directory("ee3305_bringup")
        path = os.path.join(share, "params", "scenarios.yaml")
        with open(path) as f:
            data = yaml.safe_load(f)
        if self.scenario_ not in data["scenarios"]:
            self.get_logger().error("scenario does not exist!")
            raise SystemExit(1)
        else:
            self.goal_x_ = data["scenarios"][self.scenario_]["goal"][0]
            self.goal_y_ = data["scenarios"][self.scenario_]["goal"][1]
            self.timeout_ = data["scenarios"][self.scenario_]["timeout"]
            self.get_logger().info(f"scenario: {self.scenario_}, goal_x: {self.goal_x_}, goal_y: {self.goal_y_}, timeout: {self.timeout_}")

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

        # Goal pose publisher
        self.pub_goal_ = self.create_publisher(
            PoseStamped, 
            "goal_pose", 
            10
        )
            
        # Handles: Timers
        self.timer_ = self.create_timer(0.5, self.callbackTimer_)

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
        self.rbt_x_ = x
        self.rbt_y_ = y
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

    # callback for timer
    def callbackTimer_(self):
        if self.done_: 
            return
        if self.rbt_x_ is None:
            return
        if not self.goal_sent_:
            if self.pub_goal_.get_subscription_count() < 2:
                return
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = self.goal_x_
            goal_pose.pose.position.y = self.goal_y_
            goal_pose.pose.orientation.w = 1.0 

            self.get_logger().info(
                        f"Sending Goal pose @ ({self.goal_x_:7.3f}, {self.goal_y_:7.3f})"
            )
            self.pub_goal_.publish(goal_pose)
            self.goal_sent_ = True
            self.t_goal_ = self.get_clock().now().nanoseconds * 1e-9
            
        dx = self.goal_x_ - self.rbt_x_
        dy = self.goal_y_ - self.rbt_y_
        distance = math.hypot(dx,dy)
        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self.t_goal_
        self.elapsed_ = elapsed
        self.distance_ = distance

        if distance < self.goal_tol_ :
            self.done_ = True
            self.result_ = "reached"
            self.get_logger().info(f"Reached goal in {elapsed} s ({distance} m away)")
            self.writeResult([self.config_, self.scenario_, self.run_, self.result_, self.elapsed_, self.distance_, self.goal_x_, self.goal_y_, self.timeout_, self.goal_tol_])
        elif elapsed > self.timeout_ :
            self.done_ = True
            self.result_ = "timeout"
            self.get_logger().info(f"Did not reach goal, timed out in {elapsed} s ({distance} m away)")
            self.writeResult([self.config_, self.scenario_, self.run_, self.result_, self.elapsed_, self.distance_, self.goal_x_, self.goal_y_, self.timeout_, self.goal_tol_])

    def writeResult(self, header):
        resultcsv = self.openCsv_("result.csv", ["config", "scenario", "run", "result", "elapsed", "distance", "goal_x", "goal_y", "timeout", "goal_tol"])    
        resultcsv.writerow(header)

def main(args=None):
    rclpy.init(args=args)
    node = Evaluator()
    try:
        while rclpy.ok() and not node.done_:
            rclpy.spin_once(node,timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        for f in node.files_:
            f.close()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()

