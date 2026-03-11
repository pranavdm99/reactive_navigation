import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

# Key mappings for TurtleBot3 (burger)
# Re-architected to be similar to teleop_twist_keyboard
msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : move forward/backward
a/d : turn left/right
s   : force stop

t/g : increase/decrease linear velocity step size
y/h : increase/decrease angular velocity step size

Note: Releasing keys will cause the robot to slow down to a stop.

CTRL-C to quit
"""

move_bindings = {
    'w': (1, 0),
    'x': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
    's': (0, 0),
    ' ': (0, 0),
}

speed_bindings = {
    't': (1.1, 1),
    'g': (0.9, 1),
    'y': (1, 1.1),
    'h': (1, 0.9),
}

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.speed = 0.15
        self.turn = 0.5
        
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        
        self.lin_accel = 0.02
        self.ang_accel = 0.1
        self.lin_decay = 0.03
        self.ang_decay = 0.15
        
        self.max_lin_vel = 0.22
        self.max_ang_vel = 2.84
        
        print(msg)
        print(f"Currently: speed {self.speed}, turn {self.turn}")
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vlimit(self, val, min_val, max_val):
        return max(min(val, max_val), min_val)

    def timer_callback(self):
        key = get_key(self.settings)
        
        target_x = 0.0
        target_z = 0.0
        
        if key in move_bindings.keys():
            target_x = move_bindings[key][0] * self.speed
            target_z = move_bindings[key][1] * self.turn
            verb = "MOVING" if key != 's' and key != ' ' else "STOPPING"
            self.print_feedback(f"{verb}: Key {key} pressed. Target: Lin {target_x:.2f}, Ang {target_z:.2f}")
        elif key in speed_bindings.keys():
            self.speed = self.vlimit(self.speed * speed_bindings[key][0], 0.01, self.max_lin_vel)
            self.turn = self.vlimit(self.turn * speed_bindings[key][1], 0.1, self.max_ang_vel)
            self.print_feedback(f"CONFIG: speed {self.speed:.2f}, turn {self.turn:.2f}")
        elif key == '\x03': # CTRL-C
            rclpy.shutdown()
            sys.exit(0)
        else:
            # No key or unrecognized key -> target is 0, will decay
            if self.target_linear_vel != 0 or self.target_angular_vel != 0:
                self.print_feedback(f"COASTING: No input. Lin: {self.target_linear_vel:.3f}, Ang: {self.target_angular_vel:.3f}")

        # Smoothly transition current target to the desired target
        # Acceleration towards target
        if self.target_linear_vel < target_x:
            self.target_linear_vel = min(target_x, self.target_linear_vel + self.lin_accel)
        elif self.target_linear_vel > target_x:
            self.target_linear_vel = max(target_x, self.target_linear_vel - (self.lin_decay if target_x == 0 else self.lin_accel))
            
        if self.target_angular_vel < target_z:
            self.target_angular_vel = min(target_z, self.target_angular_vel + self.ang_accel)
        elif self.target_angular_vel > target_z:
            self.target_angular_vel = max(target_z, self.target_angular_vel - (self.ang_decay if target_z == 0 else self.ang_accel))
            
        twist = Twist()
        twist.linear.x = self.target_linear_vel
        twist.angular.z = self.target_angular_vel
        self.publisher_.publish(twist)

    def print_feedback(self, info):
        sys.stdout.write(f"\r[STATUS] {info} {' '*20}")
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        twist = Twist()
        node.publisher_.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
