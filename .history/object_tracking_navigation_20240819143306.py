import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv

def detect_navigation():
    # Initialize the ROS node
    rospy.init_node('cv_bridge_test')
    
    # Create a CvBridge object to convert between ROS and OpenCV images
    bridge = CvBridge()
    
    # Subscribe to the camera image topic
    image_sub = rospy.Subscriber('/image_raw', Image, callback, queue_size=1)
    
    # Publisher for sending control commands to the robot
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    
    rospy.spin()

def callback(data):
    try:
        # Convert the ROS image message to an OpenCV image
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    
    # Define the range for the color blue in HSV color space
    lower_blue = (100, 150, 0)
    upper_blue = (140, 255, 255)
    
    # Convert the image from BGR to HSV color space
    hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
    
    # Create a mask for the color blue
    mask = cv.inRange(hsv, lower_blue, upper_blue)
    
    # Find the contours of the masked image
    contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    # Proceed if at least one contour was found
    if contours:
        # Find the largest contour by area
        c = max(contours, key=cv.contourArea)
        x, y, w, h = cv.boundingRect(c)
        
        # Draw a rectangle around the largest contour
        cv.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        
        # Calculate the center of the rectangle
        center_x = x + w//2
        center_y = y + h//2
        
        # Display the detected object center
        cv.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)
        
        # Control the robot based on the object's position
        twist = Twist()
        screen_center_x = cv_image.shape[1] // 2
        
        if center_x < screen_center_x - 50:
            twist.linear.x = 0.2
            twist.angular.z = 0.1
        elif center_x > screen_center_x + 50:
            twist.linear.x = 0.2
            twist.angular.z = -0.1
        else:
            twist.linear.x = 0.2
            twist.angular.z = 0
        
        # Publish the control command
        cmd_pub.publish(twist)
    
    # Display the resulting image
    cv.imshow("Image Window", cv_image)
    cv.waitKey(3)

if __name__ == '__main__':
    try:
        detect_navigation()
    except rospy.ROSInterruptException:
        pass
