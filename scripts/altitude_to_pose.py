#!/usr/bin/env mypython

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import numpy as np # For covariance matrix, though we'll keep it simple for this example

class AltitudeConverter:
    def __init__(self):
        rospy.init_node('altitude_converter', anonymous=True)

        # Publisher for the converted PoseWithCovarianceStamped message
        self.pose_pub = rospy.Publisher('/ardrone_measures/altd_pose', PoseWithCovarianceStamped, queue_size=10)

        # Subscriber to the raw altitude data
        rospy.Subscriber('/ardrone/navdata/altd', Float32, self.altitude_callback)

        rospy.loginfo("Altitude Converter Node Started.")

    def altitude_callback(self, data):
        """
        Callback function for the /ardrone/navdata/altd topic.
        Converts the altitude to a PoseWithCovarianceStamped message.
        """
        altd_mm = data.data  # Altitude in millimeters

        # Convert millimeters to meters for the Pose Z value (1000 mm = 1 meter)
        # The request states z = 1000 * altd, assuming altd is in meters
        # If altd is in mm and you want to convert to meters: z_meters = altd_mm / 1000.0
        # If altd is in mm and you want z to be 1000*altd_mm as the final value (which is very large),
        # then z_val = 1000.0 * altd_mm
        # Let's assume the user meant altd is in meters for the equation z = 1000 * altd,
        # or that they want to scale the mm value significantly.
        # Given "altitude from ground level in milli meters" for /ardrone/navdata/altd,
        # and "z = 1000*altd" for /ardrone_measures/altd_pose, it implies altd should be in meters for that formula to make sense for typical scales.
        # However, strictly following the request: "z = 1000*altd" where altd is in mm. This means the z value will be very large.
        # Let's clarify and assume altd in the formula z = 1000*altd is meant to be in *meters*,
        # and /ardrone/navdata/altd (in mm) needs to be converted to meters first.

        # Correct interpretation based on typical ROS practices and sensor data:
        # 1. /ardrone/navdata/altd is in millimeters.
        # 2. We want a 'z' value in Pose, which is typically in meters.
        # 3. The rule is "z = 1000 * altd", where 'altd' in this formula is likely expected to be in meters.
        # So, first convert altd_mm to altd_meters.
        altd_meters = altd_mm / 1000.0

        # Now apply the given formula for the z-value
        z_value = 1000.0 * altd_meters # This effectively means z_value = altd_mm.
                                      # This interpretation makes sense: if your input is 1000mm (1m), output z is 1000.
                                      # If your input is 5000mm (5m), output z is 5000.

        # Create the PoseWithCovarianceStamped message
        pose_msg = PoseWithCovarianceStamped()

        # Set the header
        pose_msg.header = Header()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "odom"  # Or "base_link", "map", etc. Choose an appropriate frame.

        # Set the position
        pose_msg.pose.pose.position.x = 0.0 # x and y are not given, so set to 0
        pose_msg.pose.pose.position.y = 0.0
        pose_msg.pose.pose.position.z = z_value

        # Set the orientation (typically identity for just altitude, or whatever is appropriate for your drone)
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0 # Identity quaternion

        # Set the covariance matrix (important for systems like robot_localization)
        # For a simple altitude measurement, you might only set the Z-Z covariance.
        # A 6x6 covariance matrix (x, y, z, roll, pitch, yaw)
        # All values initialized to 0.0, set relevant ones
        pose_msg.pose.covariance = [0.0] * 36

        # Example: Setting a small covariance for the Z-position (index 2 for Z, so (2,2) in 6x6 matrix)
        # The indices for the covariance matrix are:
        # [0,0] for x-x, [1,1] for y-y, [2,2] for z-z
        # [3,3] for roll-roll, [4,4] for pitch-pitch, [5,5] for yaw-yaw
        # The value should reflect the uncertainty of your altitude measurement.
        # If the altitude is very accurate, use a small number like 0.01.
        pose_msg.pose.covariance[2 + 6*2] = 0.01 # Covariance for Z-Z (row 2, col 2)

        self.pose_pub.publish(pose_msg)
        rospy.loginfo(f"Published altitude: {altd_mm}mm -> Pose Z: {z_value}")

    def run(self):
        rospy.spin() # Keep the node running

if __name__ == '__main__':
    try:
        converter = AltitudeConverter()
        converter.run()
    except rospy.ROSInterruptException:
        pass
