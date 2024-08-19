import rospy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
import math

def callback(data):
    # Define the publisher for /cmd_vel
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    twist = Twist()
    
    # Process the data to ensure it is within a reasonable range
    if data.pose.position.z < 0.05:
        twist.linear.x = 0
    elif data.pose.position.z > 1.5:
        twist.linear.x = 0.61
    else:
        twist.linear.x = math.sqrt(data.pose.position.z) / 2

    if data.pose.position.x / data.pose.position.z > 1:
        twist.angular.z = 1
    else:
        twist.angular.z = data.pose.position.x / data.pose.position.z
    
    # Publish the control command
    pub.publish(twist)

def car_pubandsub():
    rospy.Subscriber('/visualization_marker', Marker, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        print("Starting control node...")
        # Initialize the ROS node
        rospy.init_node('zxa_test', anonymous=True)
        car_pubandsub()
    except KeyboardInterrupt:
        print("Shutting down control node.")
