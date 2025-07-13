import rospy
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist
from collections import deque
import os
from datetime import datetime
import json

class CommandInterface:
    def __init__(self):
        rospy.init_node('drone_command_interface')

        # Command tracking
        self.current_cmd = {
            'takeoff': False,
            'land': False,
            'reset': False,
            'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }

        # ROS Publishers
        self.takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=5)
        self.reset_pub = rospy.Publisher('/ardrone/reset', Empty, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd_log_pub = rospy.Publisher('/command_log', String, queue_size=10)

        # Logging setup
        self.log_dir = "command_logs"
        os.makedirs(self.log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = open(f"{self.log_dir}/cmd_log_{timestamp}.csv", "w")
        self.log_file.write("timestamp,cmd_type,linear_x,linear_y,linear_z,angular_x,angular_y,angular_z\n")

    def log_command(self, cmd_type, linear=None, angular=None):
        timestamp = rospy.Time.now().to_sec()

        # Update current command state
        if cmd_type in ['takeoff', 'land', 'reset']:
            self.current_cmd[cmd_type] = True
            # Reset others
            for cmd in ['takeoff', 'land', 'reset']:
                if cmd != cmd_type:
                    self.current_cmd[cmd] = False
        elif cmd_type == 'movement':
            if linear:
                self.current_cmd['linear'].update(linear)
            if angular:
                self.current_cmd['angular'].update(angular)

        # Publish to ROS topic for other nodes
        cmd_msg = {
            'timestamp': timestamp,
            'type': cmd_type,
            'linear': self.current_cmd['linear'],
            'angular': self.current_cmd['angular'],
            'takeoff': self.current_cmd['takeoff'],
            'land': self.current_cmd['land'],
            'reset': self.current_cmd['reset']
        }
        self.cmd_log_pub.publish(String(json.dumps(cmd_msg)))

        # Log to file
        self.log_file.write(
            f"{timestamp},{cmd_type},"
            f"{self.current_cmd['linear']['x']},"
            f"{self.current_cmd['linear']['y']},"
            f"{self.current_cmd['linear']['z']},"
            f"{self.current_cmd['angular']['x']},"
            f"{self.current_cmd['angular']['y']},"
            f"{self.current_cmd['angular']['z']}\n"
        )
        self.log_file.flush()

        # Execute command
        if cmd_type == 'takeoff':
            self.takeoff_pub.publish(Empty())
        elif cmd_type == 'land':
            self.land_pub.publish(Empty())
        elif cmd_type == 'reset':
            self.reset_pub.publish(Empty())
        elif cmd_type == 'movement':
            twist = Twist()
            twist.linear.x = self.current_cmd['linear']['x']
            twist.linear.y = self.current_cmd['linear']['y']
            twist.linear.z = self.current_cmd['linear']['z']
            twist.angular.x = self.current_cmd['angular']['x']
            twist.angular.y = self.current_cmd['angular']['y']
            twist.angular.z = self.current_cmd['angular']['z']
            self.cmd_vel_pub.publish(twist)

    def shutdown(self):
        self.log_file.close()
        print("Command interface shutdown complete")

if __name__ == '__main__':
    ci = CommandInterface()

    # Example: Connect this to your actual input method
    # For testing, we'll just send some commands
    import time
    try:
        ci.log_command('takeoff')
        time.sleep(2)
        ci.log_command('movement', linear={'x': 0.5, 'y': 0.0, 'z': 0.0})
        time.sleep(1)
        ci.log_command('land')
        rospy.spin()
    except KeyboardInterrupt:
        ci.shutdown()
