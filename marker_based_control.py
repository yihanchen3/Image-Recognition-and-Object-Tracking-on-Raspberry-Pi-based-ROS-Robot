import rospy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
import math

def callback(data):
    # Publisher for velocity commands to the robot
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    # Create a new Twist message instance to hold the velocity commands
    twist = Twist()
    
    # Set linear velocity based on the marker's distance (z-axis)
    if data.pose.position.z < 0.05:  # Stop if too close
        twist.linear.x = 0
    elif data.pose.position.z > 1.5:  # Fast speed if far
        twist.linear.x = 0.61
    else:  # Proportional speed(square root of the distance) for smoother control
        twist.linear.x = math.sqrt(data.pose.position.z) / 2

    # Set angular velocity based on the marker's lateral position (x/z ratio)
    if data.pose.position.x / data.pose.position.z > 1:
        twist.angular.z = 1  # Sharp turn if significantly off-center (ratio > 1)
    else:
        twist.angular.z = data.pose.position.x / data.pose.position.z  # Proportional turn
    
    # Send the velocity commands
    pub.publish(twist)

def car_pubandsub():
    # Subscribe to marker data and process with callback
    rospy.Subscriber('/visualization_marker', Marker, callback)
    rospy.spin()  # Keep the node active

if __name__ == '__main__':
    try:
        print("Starting control node...")
        rospy.init_node('marker_based_control', anonymous=True)  # Initialize ROS node
        car_pubandsub()
    except KeyboardInterrupt:
        print("Shutting down control node.")
