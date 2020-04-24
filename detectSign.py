import numpy as np
import rospy
import cv2
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

# CONSTANTS
CYCLE_TIME = 2 # a bit slower to allow turtlebot to actually move and not overwhelm system

# GLOBALS
waypoints = None
subscriber_waypoints = None
subscriber_image = None
subscriber_pose = None
publisher_sign_detection = None
pose_x = None
pose_y = None


'''
Init function to start our detectSign node and instantiate publishers and subscribers
'''
def init():
    global waypoints, subscriber_waypoints, subscriber_image, publisher_sign_detection

    subscriber_image = rospy.Subscriber('/camera/rgb/image_raw', Image, imageCallback)
    subscriber_waypoints = rospy.Subscriber('/parkingBot/waypoints', Float32MultiArray, waypointCallback)
    subscriber_pose = rospy.Subscriber('/odom', Odometry, poseCallback)

    rospy.init_node('signCV')


def imageCallback(data):
    wp = nearWaypoint()
    if not wp is None:
        Bridge = CvBridge()
        try:
            cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
            analyzePicture(cv_img, wp)
        except:
            pass
    return

def waypointCallback(data):
    waypoints = list(data.copy())

def poseCallback(data):
    global pose_x, pose_y

    pose_x = data.pose.pose.position.x
    pose_y = data.pose.pose.position.y

def nearWaypoint():
    global pose_x, pose_y, waypoints

    if pose_x is None or pose_y is None:
        return None

    if waypoints is None:
        return None

    THRESHOLD = 5 # TO BE DETERMINED

    for i in range(len(waypoints)):
        wp_y, wp_x = waypoints[i]
        if np.sqrt((wp_y - pose_y)^2 + (wp_x - pose_x)^2) <= THRESHOLD:
            waypoints = waypoints.pop(i)
            return (wp_y, wp_x)

    return None


'''
analyzePicture
input: np array from takePicture
outPut: what type of sign it is
'''
def analyzePicture(img, wp):
    cv2.imshow(img)
    cv2.waitKey(0)
    print(wp)

    return


def loop():
    global subscriber_pose, subscriber_image, subscriber_waypoints #all the globals

    init()

    while not rospy.is_shutdown():
        start_time = time.time()
        rospy.sleep(max(CYCLE_TIME - (start_time - time.time()), 0))

if __name__ == "__main__":
    loop()
