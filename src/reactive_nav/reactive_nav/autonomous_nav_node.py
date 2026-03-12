import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class AutonomousNavNode(Node):
    def __init__(self):
        super().__init__('autonomous_nav_node')
        
        # Parameters
        self.declare_parameter('max_speed', 0.15)
        self.declare_parameter('safety_dist', 0.4)
        self.declare_parameter('turning_speed', 0.5)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('mline_tolerance', 0.1)
        self.declare_parameter('progress_delta', 0.1)
        self.declare_parameter('wall_side', 'left') # 'left' or 'right'

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Robot State
        self.status = "IDLE"
        self.state = "GO_TO_POINT" # GO_TO_POINT, WALL_FOLLOW
        
        self.pose = None # (x, y, yaw)
        self.goal = None # (x, y)
        self.regions = None # {'front', 'left', 'right', ...}
        
        # Bug2 specific
        self.start_pos = None
        self.hit_point = None
        self.prev_mline_side = None # To detect crossing
        
        # Statistics
        self.total_distance = 0.0
        self.last_pos = None
        self.start_nav_time = None
        self.full_goal_pose = None # (x, y, z)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Bug2 Autonomous Navigation Node Started')

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        
        # Track distance
        curr_p = (pos.x, pos.y, pos.z)
        if self.status == "NAVIGATING" and self.last_pos is not None:
            d = math.sqrt((curr_p[0]-self.last_pos[0])**2 + (curr_p[1]-self.last_pos[1])**2 + (curr_p[2]-self.last_pos[2])**2)
            self.total_distance += d
        self.last_pos = curr_p

        # Convert quaternion to yaw
        siny_cosp = 2 * (orient.w * orient.z + orient.x * orient.y)
        cosy_cosp = 1 - 2 * (orient.y * orient.y + orient.z * orient.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.pose = (pos.x, pos.y, yaw)
        if self.start_pos is None:
            self.start_pos = (pos.x, pos.y)

    def scan_callback(self, msg):
        # Divide ranges into 5 regions
        ranges = msg.ranges
        size = len(ranges)
        self.regions = {
            'right':  min(min(ranges[270:306]), 10),
            'fright': min(min(ranges[306:342]), 10),
            'front':  min(min(ranges[342:size] + ranges[0:18]), 10),
            'fleft':  min(min(ranges[18:54]), 10),
            'left':   min(min(ranges[54:90]), 10),
        }

    def goal_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.full_goal_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.state = "GO_TO_POINT"
        self.status = "NAVIGATING"
        self.total_distance = 0.0
        self.start_nav_time = self.get_clock().now()
        
        # Reset start pos for m-line from current position
        if self.pose:
            self.start_pos = (self.pose[0], self.pose[1])
        self.prev_mline_side = None
        self.get_logger().info(f'New Goal Received: {self.goal}')

    def is_on_mline(self):
        if not self.pose or not self.goal or not self.start_pos:
            return False
        
        x0, y0 = self.start_pos
        x1, y1 = self.goal
        curr_x, curr_y, _ = self.pose
        
        # Line equation: (y0 - y1)x + (x1 - x0)y + (x0y1 - x1y0) = 0
        A = y0 - y1
        B = x1 - x0
        C = x0 * y1 - x1 * y0
        
        val = A * curr_x + B * curr_y + C
        dist = abs(val) / math.sqrt(A**2 + B**2) if (A**2 + B**2) != 0 else 0
        
        # Side tracking: sign of the cross product
        curr_side = 1 if val >= 0 else -1
        
        # Check if we are closer to the goal than the hit point
        dist_to_goal = math.sqrt((curr_x - x1)**2 + (curr_y - y1)**2)
        dist_hit_to_goal = 1000.0
        if self.hit_point:
            dist_hit_to_goal = math.sqrt((self.hit_point[0] - x1)**2 + (self.hit_point[1] - y1)**2)
        
        # Progress check: must be closer to goal than hit point
        progress_delta = self.get_parameter('progress_delta').get_parameter_value().double_value
        has_progressed = dist_to_goal < (dist_hit_to_goal - progress_delta)
        
        # Intersection logic: either we are very close to the line, 
        # OR we just swapped sides (crossed it)
        mline_tolerance = self.get_parameter('mline_tolerance').get_parameter_value().double_value
        crossed_line = False
        if self.prev_mline_side is not None and self.prev_mline_side != curr_side:
            crossed_line = True
        
        self.prev_mline_side = curr_side
        
        return (dist < mline_tolerance or crossed_line) and has_progressed

    def control_loop(self):
        if not self.pose or not self.goal or not self.regions:
            return

        twist = Twist()
        curr_x, curr_y, curr_yaw = self.pose
        goal_x, goal_y = self.goal
        
        dist_to_goal = math.sqrt((goal_x - curr_x)**2 + (goal_y - curr_y)**2)
        
        goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        if dist_to_goal < goal_tolerance:
            self.status = "IDLE"
            
            # Calculate and print summary
            end_time = self.get_clock().now()
            duration = (end_time - self.start_nav_time).nanoseconds / 1e9 if self.start_nav_time else 0.0
            
            final_p = self.last_pos if self.last_pos else (curr_x, curr_y, 0.0)
            gp = self.full_goal_pose if self.full_goal_pose else (goal_x, goal_y, 0.0)
            error = math.sqrt((gp[0]-final_p[0])**2 + (gp[1]-final_p[1])**2 + (gp[2]-final_p[2])**2)

            summary = (
                "\n" + "="*40 + "\n"
                "       NAVIGATION SUMMARY\n" +
                "="*40 + "\n"
                f"Status:          Goal Reached\n"
                f"Time Taken:      {duration:.2f} seconds\n"
                f"Total Distance:  {self.total_distance:.2f} meters\n"
                f"Goal Pose (XYZ): ({gp[0]:.3f}, {gp[1]:.3f}, {gp[2]:.3f})\n"
                f"Final Pose(XYZ): ({final_p[0]:.3f}, {final_p[1]:.3f}, {final_p[2]:.3f})\n"
                f"Error: {error:.4f} meters\n" +
                "="*40 + "\n"
            )
            self.get_logger().info(summary)
            
            self.goal = None
            self.cmd_pub.publish(twist)
            return

        safety_dist = self.get_parameter('safety_dist').get_parameter_value().double_value
        max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        turning_speed = self.get_parameter('turning_speed').get_parameter_value().double_value

        if self.state == "GO_TO_POINT":
            if self.regions['front'] < safety_dist:
                self.state = "WALL_FOLLOW"
                self.hit_point = (curr_x, curr_y)
                self.get_logger().warn(f"OBSTACLE HIT! Front: {self.regions['front']:.2f}m. Switching to Wall Following (Left-Hand).")
            else:
                # Move toward goal
                goal_yaw = math.atan2(goal_y - curr_y, goal_x - curr_x)
                angle_to_goal = normalize_angle(goal_yaw - curr_yaw)
                
                if abs(angle_to_goal) > 0.1:
                    twist.angular.z = turning_speed if angle_to_goal > 0 else -turning_speed
                    self.get_logger().info(f"GO_TO_POINT: Aligning to goal. Heading Error: {angle_to_goal:.2f} rad", throttle_duration_sec=2.0)
                else:
                    twist.linear.x = max_speed
                    self.get_logger().info(f"GO_TO_POINT: Moving towards goal. Dist: {dist_to_goal:.2f}m", throttle_duration_sec=2.0)

        elif self.state == "WALL_FOLLOW":
            if self.is_on_mline():
                self.state = "GO_TO_POINT"
                self.get_logger().info("M-LINE REACHED! Path to goal is clear. Returning to Goal Seeking.")
            else:
                # Dynamic Wall following logic (Left or Right)
                side = self.get_parameter('wall_side').get_parameter_value().string_value
                side_mult = 1.0 if side == 'left' else -1.0
                
                # Sensor regions based on side
                s_front = self.regions['front']
                s_side  = self.regions['left']  if side == 'left' else self.regions['right']
                s_fside = self.regions['fleft'] if side == 'left' else self.regions['fright']
                
                if s_front < safety_dist:
                    # Inner Corner / Obstacle in front -> Turn AWAY from wall
                    twist.angular.z = -turning_speed * side_mult
                    dir_name = "RIGHT" if side == 'left' else "LEFT"
                    self.get_logger().info(f"WALL_FOLLOW ({side.upper()}): [INNER CORNER] Obstacle in front ({s_front:.2f}m). Turning {dir_name}.", throttle_duration_sec=1.0)
                elif s_fside < safety_dist or s_side < safety_dist:
                    # Following wall
                    twist.linear.x = max_speed
                    self.get_logger().info(f"WALL_FOLLOW ({side.upper()}): [FOLLOWING] Wall detected on {side.capitalize()} (S:{s_side:.2f}m, FS:{s_fside:.2f}m). Moving forward.", throttle_duration_sec=2.0)
                else:
                    # Outer Corner / Loss of wall -> Turn TOWARD wall
                    twist.linear.x = max_speed * 0.5
                    twist.angular.z = turning_speed * 0.5 * side_mult
                    dir_name = "LEFT" if side == 'left' else "RIGHT"
                    self.get_logger().info(f"WALL_FOLLOW ({side.upper()}): [OUTER CORNER] Wall lost on {side}. Turning {dir_name} to find boundary.", throttle_duration_sec=1.0)

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
