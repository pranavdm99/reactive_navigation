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
        self.declare_parameter('max_speed', 0.21)
        self.declare_parameter('safety_dist', 0.5)
        self.declare_parameter('turning_speed', 0.5)

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
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Bug2 Autonomous Navigation Node Started')

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        
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
        self.state = "GO_TO_POINT"
        self.status = "NAVIGATING"
        # Reset start pos for m-line from current position
        if self.pose:
            self.start_pos = (self.pose[0], self.pose[1])
        self.get_logger().info(f'New Goal Received: {self.goal}')

    def is_on_mline(self):
        if not self.pose or not self.goal or not self.start_pos:
            return False
        
        x0, y0 = self.start_pos
        x1, y1 = self.goal
        curr_x, curr_y, _ = self.pose
        
        # Distance from point to line: |(y1-y0)x - (x1-x0)y + x1y0 - y1x0| / sqrt((y1-y0)^2 + (x1-x0)^2)
        numerator = abs((y1 - y0) * curr_x - (x1 - x0) * curr_y + x1 * y0 - y1 * x0)
        denominator = math.sqrt((y1 - y0)**2 + (x1 - x0)**2)
        dist = numerator / denominator if denominator != 0 else 0
        
        # Check if we are closer to the goal than the hit point
        dist_to_goal = math.sqrt((curr_x - x1)**2 + (curr_y - y1)**2)
        dist_hit_to_goal = 1000.0
        if self.hit_point:
            dist_hit_to_goal = math.sqrt((self.hit_point[0] - x1)**2 + (self.hit_point[1] - y1)**2)
        
        return dist < 0.1 and dist_to_goal < (dist_hit_to_goal - 0.2)

    def control_loop(self):
        if not self.pose or not self.goal or not self.regions:
            return

        twist = Twist()
        curr_x, curr_y, curr_yaw = self.pose
        goal_x, goal_y = self.goal
        
        dist_to_goal = math.sqrt((goal_x - curr_x)**2 + (goal_y - curr_y)**2)
        
        if dist_to_goal < 0.2:
            self.status = "IDLE"
            self.get_logger().info("Goal Reached!")
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
                # Wall following logic (Left-Hand)
                # Keep the wall on the robot's LEFT side
                if self.regions['front'] < safety_dist:
                    # Inner Corner / Obstacle in front
                    twist.angular.z = -turning_speed 
                    self.get_logger().info(f"WALL_FOLLOW: [INNER CORNER] Obstacle in front ({self.regions['front']:.2f}m). Turning RIGHT.", throttle_duration_sec=1.0)
                elif self.regions['fleft'] < safety_dist or self.regions['left'] < safety_dist:
                    # Following wall
                    twist.linear.x = max_speed
                    self.get_logger().info(f"WALL_FOLLOW: [FOLLOWING] Wall detected on Left (L:{self.regions['left']:.2f}m, FL:{self.regions['fleft']:.2f}m). Moving forward.", throttle_duration_sec=2.0)
                else:
                    # Outer Corner / Loss of wall
                    twist.linear.x = max_speed * 0.5
                    twist.angular.z = turning_speed * 0.5
                    self.get_logger().info(f"WALL_FOLLOW: [OUTER CORNER] Wall lost on left. Turning LEFT to find boundary.", throttle_duration_sec=1.0)

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
