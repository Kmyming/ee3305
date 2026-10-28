from math import hypot

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, Path


class Behavior(Node):

    def __init__(self, node_name="behavior"):
        # Node Constructor =============================================================
        super().__init__(node_name)

        # Parameters: Declare
        self.declare_parameter("frequency", float(10))
        self.declare_parameter("plan_frequency", float(2))
        self.declare_parameter("use_fsm", False)        # True: state machine, False: baseline 2 Hz requests
        self.declare_parameter("goal_tol", 0.15)        # m, a waypoint counts as reached inside this
        self.declare_parameter("replan_dist", 0.3)      # m, replan when the robot is this far off the path
        self.declare_parameter("plan_timeout", 3.0)     # s, resend the request if no path arrives by then

        # Parameters: Get Values
        self.frequency_ = self.get_parameter("frequency").value
        self.plan_frequency_ = self.get_parameter("plan_frequency").value
        self.use_fsm_ = self.get_parameter("use_fsm").value
        self.goal_tol_ = self.get_parameter("goal_tol").value
        self.replan_dist_ = self.get_parameter("replan_dist").value
        self.plan_timeout_ = self.get_parameter("plan_timeout").value

        # Handles: Topic Subscribers
        # !TODO: Goal pose subscriber
        self.sub_goal_ = self.create_subscription(
            PoseStamped, "goal_pose", self.callbackSubGoalPose_, 10)

        # !TODO: Odometry subscriber
        self.sub_odom_ = self.create_subscription(
            Odometry, "odom", self.callbackSubOdom_, qos_profile_sensor_data)

        # Path subscriber (FSM: to know when a planned path arrives, and how far off the robot is from said path)
        self.sub_path_ = self.create_subscription(
             Path,
             "path",
             self.callbackSubPath_,
             10
        )

        # Waypoints subscriber (FSM: a list of goals to visit in order)
        self.sub_waypoints_ = self.create_subscription(
            PoseArray, "waypoints", self.callbackSubWaypoints_, 10)

        # Handles: Topic Publishers
        # !TODO: Path request publisher
        self.pub_path_request_ = self.create_publisher(Path, "path_request", 10)

        # Handles: Timers
        self.timer = self.create_timer(1.0 / self.frequency_, self.callbackTimer_)
        # Baseline only: request a path at plan_frequency. The FSM sends its own requests.
        if not self.use_fsm_:
            self.timer_plan_ = self.create_timer(
                1.0 / self.plan_frequency_, self.callbackTimerPlan_
            )

        # Other Instance Variables
        self.received_goal_coords_ = False
        self.received_rbt_coords_ = False
        self.goal_reached_ = True
        self.state_ = "IDLE" # IDLE | PLANNING | FOLLOWING | REACHED | DONE | FAILED
        self.waypoints_ = [] #[(x,y),...] to visit in order
        self.wp_idx_ = 0 # index of the current waypoint in waypoints
        self.path_ =  []
        self.request_sent_ = False # a request for the current waypoint has gone out
        self.path_received_ = False # if path has come back
        self.t_request_ = None # time the last request was sent
        self.plan_tries_ = 0 # timed-out requests for the current plan

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

        #FSM: a single goal is a list of one waypoint
        if self.use_fsm_:
            self.waypoints_ = [(self.goal_x_, self.goal_y_)]
            self.startMission_()

    # Waypoints subscriber callback: replace the mission with this list
    def callbackSubWaypoints_(self, msg: PoseArray):
        if not self.use_fsm_ or len(msg.poses) == 0:
            return
        self.waypoints_ = [(p.position.x, p.position.y) for p in msg.poses]
        self.startMission_()

    # Odometry subscriber callback.
    def callbackSubOdom_(self, msg: Odometry):
        self.received_rbt_coords_ = True

        # !TODO: Copy to rbt_x_, rbt_y_.
        self.rbt_x_ = msg.pose.pose.position.x
        self.rbt_y_ = msg.pose.pose.position.y

    # Path subscriber callback: keep the latest path if it leads to the current waypoint. request-path pairing
    def callbackSubPath_(self, msg: Path):
        if len(msg.poses) == 0 or not self.waypoints_:
            return
        end = msg.poses[-1].pose.position
        wp_x, wp_y = self.waypoints_[self.wp_idx_]
        if hypot(end.x - wp_x, end.y - wp_y) > self.goal_tol_:
            return # a late answer to an older request: ignore it
        self.path_ = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.path_received_ = True

    # Callback for timer.
    # Normally the decisions of the robot system are made here, and this callback is dramatically simplified.
    # The callback contains some example code for waypoint detection.
    def callbackTimer_(self):
        if self.use_fsm_:
            self.stepFsm_()
            return
        
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

    # Starts a mission from waypoints_: plan to the first one
    def startMission_(self):
        self.wp_idx_ = 0 
        self.goal_x_, self.goal_y_ = self.waypoints_[0]
        self.received_goal_coords_ = True
        self.request_sent_ = False
        self.plan_tries_ = 0
        self.get_logger().info(f"Mission: {len(self.waypoints_)} waypoint(s).")
        self.setState_("PLANNING")

    # changes the FSM state and logs the change
    def setState_(self, new_state):
        if new_state != self.state_:
            self.get_logger().info(f"FSM: {self.state_} -> {new_state}")
            self.state_ = new_state

    # State machine: runs once per timer tick (frequency)
    def stepFsm_(self):
        if self.state_ == "PLANNING":
            if not self.received_rbt_coords_:
                return # no odom yet, so the path's start is unknown

            # Send one request, then wait for its path
            if not self.request_sent_:
                self.sendPathRequest_()
                self.request_sent_ = True
                self.path_received_ = False
                self.t_request_ = self.get_clock().now()
                return

            if self.path_received_:
                self.plan_tries_ = 0
                self.setState_("FOLLOWING")
                return

            waited = (self.get_clock().now() - self.t_request_).nanoseconds * 1e-9
            if waited > self.plan_timeout_:
                self.plan_tries_ += 1
                if self.plan_tries_ >= 3:
                    self.get_logger().error("No Path after 3 tries.")
                    self.setState_("FAILED")
                else:
                    self.get_logger().warn(f"No path after {waited: .1f} s, resending (try {self.plan_tries_ + 1} of 3).")
                    self.request_sent_ = False

        elif self.state_ == "FOLLOWING":
            # Arrived at the current waypint?
            wp_x, wp_y = self.waypoints_[self.wp_idx_]
            if hypot(wp_x - self.rbt_x_, wp_y - self.rbt_y_) < self.goal_tol_:
                self.setState_("REACHED")
                return

            # replan when the robot is more than replan_dist off the path
            off_path = min(hypot(px - self.rbt_x_, py - self.rbt_y_) for px,py in self.path_)
            if off_path > self.replan_dist_:
                self.get_logger().warn(f"{off_path:.2f} m off the path, replanning.")
                self.request_sent_ = False
                self.setState_("PLANNING")

        elif self.state_ == "REACHED":
            self.get_logger().info(f"Reached waypoint {self.wp_idx_ + 1} of {len(self.waypoints_)}.")
            self.wp_idx_ += 1
            if self.wp_idx_< len(self.waypoints_):
                # The next waypoint becomes the goal, then plan to it
                self.goal_x_, self.goal_y_ = self.waypoints_[self.wp_idx_]
                self.request_sent_ = False
                self.setState_("PLANNING")
            else:
                self.setState_("DONE")

        # IDLE, DONE and FAILED wait for a new goal (callbackSubGoalPose_)

    # Callback for publishing path requests between clicked_point (goal) and robot position.
    # Normally path requests are implemented with ROS2 service, and the service is called in the main timer.
    # To keep things simple for this course, we use only ROS2 tpics.
    def callbackTimerPlan_(self):
        if not self.received_goal_coords_ or not self.received_rbt_coords_:
            return  # silently return if none of the coords are received from the subscribers

        self.sendPathRequest_()

    def sendPathRequest_(self):
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
