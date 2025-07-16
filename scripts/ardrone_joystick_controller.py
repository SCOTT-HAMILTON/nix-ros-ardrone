#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty, String
from std_srvs.srv import Empty as EmptySrv
from geometry_msgs.msg import Twist
import time

class HotasXController:
    def __init__(self):
        rospy.loginfo("AR Drone joystick conroller Initializing...")
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

    def toggle_camera(self):
        try:
            togglecam_service = rospy.ServiceProxy('/ardrone/togglecam', EmptySrv)
            togglecam_service()
            rospy.loginfo("Camera toggled")
        except Exception as e:
            rospy.logerr("Toggle cam Service call failed: %s", e)

    def callback(self, joy):
        current_time = time.time() * 1000  # Get current time in milliseconds
        # print(f"Joy callback button3={joy.buttons[3]}, delta time={current_time - self.last_trigger_times['togglecam']} ms")

        # twist = Twist()
        # # # Custom mappings
        # twist.linear.x = joy.axes[1] * 0.5   # Forward/Back
        # twist.linear.y = joy.axes[0] * 0.3   # Left/Right
        # twist.linear.z = (joy.axes[2] + 1) * -0.5  # Throttle (reversed)
        # twist.angular.z = joy.axes[4] * 1.0  # Yaw
        # # twist.angular.z = 0

        if joy.buttons[1] and current_time - self.last_trigger_times['takeoff'] > 1000: # L1
            self.last_trigger_times['takeoff'] = current_time
            rospy.loginfo("Takeoff")
            self.takeoff_pub.publish(Empty())
        if joy.buttons[7] and current_time - self.last_trigger_times['land'] > 1000:  # X 6
            self.last_trigger_times['land'] = current_time
            rospy.loginfo("Land")
            self.land_pub.publish(Empty())
        if joy.buttons[11] and current_time - self.last_trigger_times['reset'] > 1000:  # ST
            self.last_trigger_times['reset'] = current_time
            rospy.loginfo("Reset")
            self.reset_pub.publish(Empty())
        if joy.buttons[3] and current_time - self.last_trigger_times['togglecam'] > 1000:  # ST
            self.last_trigger_times['togglecam'] = current_time
            self.toggle_camera()

        # self.pub.publish(twist)

if __name__ == '__main__':
    try:
        rospy.init_node('ardrone_joystick_controller', anonymous=True)
        controller = HotasXController()
        rospy.loginfo("ARDrone Joystick Controller node started.")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ARDrone Joystick Controller node interrupted.")
    except Exception as e:
        rospy.logerr(f"An error occurred in ARDrone Joystick Controller: {e}")
