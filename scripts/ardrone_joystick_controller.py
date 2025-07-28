#!/usr/bin/env mypython

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, Image
from std_msgs.msg import Empty, String
from std_srvs.srv import Empty as EmptySrv
import cv2
import datetime
import os # Import os for path manipulation
import rospy
import sys
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
            'picture': 0,
        }
        self.save_image = False
        rospy.Subscriber('/joy', Joy, self.callback)
        image_topic = rospy.get_param('~image_topic', '/ardrone/front/image_raw')
        rospy.Subscriber(image_topic, Image, self.save_image_callback)

        rospy.wait_for_service('/ardrone/togglecam')

    def save_image_callback(self, msg):
        if self.save_image:
            rospy.loginfo("Received image message. Saving one frame...")
        else:
            return
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        # Get base filename and extension from parameters
        base_filename = rospy.get_param('~base_filename', 'frame')
        extension = rospy.get_param('~extension', 'png')
        # Generate timestamp
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        # Construct the full filename with timestamp
        # Example: frame_20250728_143844.png
        filename = f"{base_filename}_{timestamp}.{extension}"
        output_dir = rospy.get_param('~output_directory', '')
        if output_dir and not os.path.exists(output_dir):
            try:
                os.makedirs(output_dir)
                rospy.loginfo(f"Created output directory: {output_dir}")
            except OSError as e:
                rospy.logerr(f"Error creating directory {output_dir}: {e}")
                # Fallback to current directory if directory creation fails
                output_dir = ''
        full_path = os.path.join(output_dir, filename)
        cv2.imwrite(full_path, cv_image)
        self.save_image = False
        rospy.loginfo(f"Saved image to {full_path}")

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
        if joy.buttons[4] and current_time - self.last_trigger_times['picture'] > 1000:  # ST
            self.last_trigger_times['picture'] = current_time
            self.save_image = True

        # self.pub.publish(twist)

if __name__ == '__main__':
    try:
        rospy.init_node('ardrone_joystick_controller', anonymous=True)

        image_topic = rospy.get_param('~image_topic', '/ardrone/front/image_raw')
        base_filename = rospy.get_param('~base_filename', 'frame')
        extension = rospy.get_param('~extension', 'png')
        output_dir = rospy.get_param('~output_directory', '/tmp/AR-Drone-Pictures')
        rospy.loginfo(f"AR Drone params: image_topic={image_topic}, base_filename={base_filename}, extension={extension}, output_dir={output_dir}")


        controller = HotasXController()
        rospy.loginfo("ARDrone Joystick Controller node started.")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ARDrone Joystick Controller node interrupted.")
    except Exception as e:
        rospy.logerr(f"An error occurred in ARDrone Joystick Controller: {e}")
