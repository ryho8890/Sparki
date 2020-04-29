import numpy as np
import rospy
import cv2
import time
from sensor_msgs.msg import Image
from std_msgs.msg import *
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

# CONSTANTS
CYCLE_TIME = 2 # a bit slower to allow turtlebot to actually move and not overwhelm system

# GLOBALS
subscriber_atWP = None
subscriber_image = None
publisher_sign_detection = None
publisher_captured = None

atWP = None


'''
Init function to start our detectSign node and instantiate publishers and subscribers
'''
def init():
    global waypoints
    global subscriber_atWP, subscriber_image, publisher_sign_detection, publisher_captured

    subscriber_image = rospy.Subscriber('/camera/rgb/image_raw', Image, imageCallback)
    subscriber_atW = rospy.Subscriber('/parkingbot/atWP', Bool, atWPCallback)
    publisher_captured = rospy.Publisher('parkingbot/captured', Bool, queue_size=5)

    rospy.init_node('signCV')


def atWPCallback(data):
    global atWP

    if data is None:
        return
    else:
        print('I know Im at a waypoint!')
        atWP = data


def imageCallback(data):
    global atWP, publisher_captured
    #wp = nearWaypoint()(
    print('waiting... {}\n'.format(atWP))
    if not atWP is None and atWP:
        Bridge = CvBridge()
        try:
            print('Capturing an image now..\n')
            #cv_img = Bridge.imgmsg_to_cv2(data, "bgr8")
            msg = Bool()
            msg.data = True
            publisher_captured.publish(msg)
            atWP = None

            img = np.array(data.data)
            analyzePicture(img, wp)
        except e:
            raise e
            print('Capture Failed... :(\n')
            pass
    return


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
