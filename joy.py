import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty, String
from std_srvs.srv import Empty as EmptySrv
from geometry_msgs.msg import Twist
import time

class HotasXController:
    def __init__(self):
        print("Initializing...")
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=5)
        self.reset_pub = rospy.Publisher('/ardrone/reset', Empty, queue_size=1)
        self.last_trigger_times = {
            'takeoff': 0,
            'land': 0,
            'reset': 0,
            'togglecam': 0,
        }
        rospy.Subscriber('/joy', Joy, self.callback)

        rospy.wait_for_service('/ardrone/togglecam')
        print("Ready !")

    def toggle_camera(self):
        print("Lol toggle")
        try:
            print("A")
            togglecam_service = rospy.ServiceProxy('/ardrone/togglecam', EmptySrv)
            print("B")
            togglecam_service()
            rospy.loginfo("Camera toggled")
            print("Camera toggled !")
        except Exception as e:
            rospy.logerr("Toggle cam Service call failed: %s", e)
            print("Toggle cam Service call failed: %s", e)

    def callback(self, joy):
        current_time = time.time() * 1000  # Get current time in milliseconds
        print(f"Joy callback button3={joy.buttons[3]}, delta time={current_time - self.last_trigger_times['togglecam']} ms")

        # twist = Twist()
        # # # Custom mappings
        # twist.linear.x = joy.axes[1] * 0.5   # Forward/Back
        # twist.linear.y = joy.axes[0] * 0.3   # Left/Right
        # twist.linear.z = (joy.axes[2] + 1) * -0.5  # Throttle (reversed)
        # twist.angular.z = joy.axes[4] * 1.0  # Yaw
        # # twist.angular.z = 0

        if joy.buttons[1] and current_time - self.last_trigger_times['takeoff'] > 1000: # L1
            self.last_trigger_times['takeoff'] = current_time
            print("Takeoff")
            self.takeoff_pub.publish(Empty())
        if joy.buttons[7] and current_time - self.last_trigger_times['land'] > 1000:  # X 6
            self.last_trigger_times['land'] = current_time
            print("Land")
            self.land_pub.publish(Empty())
        if joy.buttons[11] and current_time - self.last_trigger_times['reset'] > 1000:  # ST
            self.last_trigger_times['reset'] = current_time
            print("Reset")
            self.reset_pub.publish(Empty())
        if joy.buttons[3] and current_time - self.last_trigger_times['togglecam'] > 1000:  # ST
            print("Lol 1")
            self.last_trigger_times['togglecam'] = current_time
            print("Lol 2")
            self.toggle_camera()

        # self.pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('hotasx_controller_buttons')
    controller = HotasXController()
    rospy.spin()
