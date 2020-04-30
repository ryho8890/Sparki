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
subscriber_pose = None
publisher_sign_detection = None
publisher_captured = None

atWP = None
pose_x = None
pose_y = None


'''
Init function to start our detectSign node and instantiate publishers and subscribers
'''
def init():
    global waypoints, atWP
    global subscriber_atWP, subscriber_image, publisher_sign_detection, publisher_captured

    subscriber_image = rospy.Subscriber('/camera/rgb/image_raw', Image, imageCallback)
    subscriber_image = rospy.Subscriber('/odom', Odometry, poseCallback)
    subscriber_atW = rospy.Subscriber('/parkingbot/atWP', Bool, atWPCallback)
    publisher_captured = rospy.Publisher('parkingbot/captured', Bool, queue_size=5)
    publisher_sign = rospy.Publisher('/parkingbot/sign', Float32MultiArray, queue_size=5)

    rospy.init_node('signCV')

    atWP = True

def poseCallback(data):
    global pose_x, pose_y

    pose_x = data.pose.pose.position.x
    pose_y = data.pose.pose.position.y

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
            cv_img = Bridge.imgmsg_to_cv2(data, "bgr8")
            msg = Bool()
            msg.data = True
            publisher_captured.publish(msg)
            atWP = None

            img = np.uint8(cv_img)
            analyzePicture(img)
        except:
            print('Capture Failed... :(\n')
            pass
    return


'''
analyzePicture
input: np array from takePicture
outPut: what type of sign it is
'''
def analyzePicture(img):
    global publisher_sign, pose_x, pose_y

    height, width, depth = np.shape(img)

    half_height = height // 2

    # take only top half
    half_img = np.array(img[0:half_height, :, :])
    half_img_hsv = cv2.cvtColor(half_img, cv2.COLOR_BGR2HSV)

    redFiltered = cv2.inRange(half_img_hsv,(0,100,100),(12, 255, 255))

    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 6000
    params.maxArea = 9000
    params.filterByCircularity = False
    params.filterByColor = False
    params.filterByConvexity = False
    params.filterByInertia = False

    blobDetector = cv2.SimpleBlobDetector_create(params)
    keypoints = blobDetector.detect(redFiltered)

    if len(keypoints) > 0:
        print('NO PARKING DETECTED!')
        msg = Float32MultiArray()
        msg.data = [pose_y, pose_x, -1]
        publisher_sign.publish(msg)
    else:
        #repeat for black filter!
        blackFiltered = cv2.inRange(half_img_hsv,(0,0,0),(5, 5, 5))
        cv2.imshow('b', blackFiltered)
        cv2.waitKey(0)

        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = 1600
        params.maxArea = 2500#9000
        params.filterByCircularity = False
        params.filterByColor = False
        params.filterByConvexity = False
        params.filterByInertia = False

        blobDetector = cv2.SimpleBlobDetector_create(params)
        keypoints = blobDetector.detect(blackFiltered)

        digits = {
            '0': [
            [255,255,255,255],
            [255, 0 , 0 ,255],
            [255, 0 , 0 ,255],
            [255, 0 , 0 ,255],
            [255, 0 , 0 ,255],
            [255, 0 , 0 ,255],
            [255, 0 , 0 ,255],
            [255,255,255,255],
            ],
            '1': [
            [0, 255 , 255, 0],
            [0, 255 , 255 ,0],
            [0, 255 , 255 ,0],
            [0, 255 , 255 ,0],
            [0, 255 , 255 ,0],
            [0, 255 , 255 ,0],
            [0, 255 , 255, 0],
            [0, 255 , 255 ,0],
            ],
            '3': [
            [255,255,255,255],
            [255,255,255,255],
            [ 0 , 0 , 0 ,255],
            [255,255,255,255],
            [255,255,255,255],
            [ 0 , 0 , 0 ,255],
            [255,255,255,255],
            [255,255,255,255],
            ],
            '5': [
            [255,255,255,255],
            [255,255,255,255],
            [255, 0 , 0 , 0 ],
            [255,255,255,255],
            [255,255,255,255],
            [ 0 , 0 , 0 ,255],
            [255,255,255,255],
            [255,255,255,255],
            ],
            '6': [
            [255,255,255,255],
            [255,255,255,255],
            [255, 0 , 0 , 0 ],
            [255,255,255,255],
            [255,255,255,255],
            [255, 0 , 0 ,255],
            [255,255,255,255],
            [255,255,255,255],
            ],
        }

        detected = {
            '0': {
                'd': False,
                's': 0
            },
            '1': {
                'd': False,
                's': 0
            },
            '3': {
                'd': False,
                's': 0
            },
            '5': {
                'd': False,
                's': 0
            },
            '6': {
                'd': False,
                's': 0
            },
        }

        for keypoint in keypoints:
            x = int(keypoint.pt[0])
            y = int(keypoint.pt[1])
            s = int(keypoint.size)
            w = s // 2
            #cv2.rectangle(blackFiltered,(x-w,y-w),(x+s,y+s),255,2)

            kp_img = blackFiltered[y-s:y+s, x-w:x+w]

            #h,w = kp_img.shape

            # crop the image
            lr = None
            lc = None
            hc = None
            hr = None

            prev_r = False
            for r in range(2*s):
                for c in range(2*w):
                    if kp_img[r,c] > 100:
                        prev_r = True

                        if lc is None:
                            lc = c
                        else:
                            lc = min(c,lc)

                        if lr is None:
                            lr = r
                        else:
                            lr = min(r, lr)
                    else:
                        if not lc is None:
                            if hc is None:
                                hc = c
                            else:
                                hc = max(hc, c)

                        if prev_r and not lr is None:
                            if hr is None:
                                hr = r
                            else:
                                hr = max(hr, r)
                        prev_r = False

            pot_digit = kp_img[lr:hr, lc:hc]
            scores = {}
            maxK = -1
            maxV = -1
            for k in digits:
                d = np.uint8(digits[k])
                d = cv2.resize(d, (hc-lc, hr-lr))
                score = np.sum(d == pot_digit)
                if score > maxV:
                    maxK = k
                    maxV = score


            detected[maxK]['d'] = True
            detected[maxK]['s'] = maxV

        if detected['0']['d']:
            if detected['3']['d'] and detected['6']['d']:
                # if both detected assume worst one for safety
                print('30 DETECTED!')
                msg = Float32MultiArray()
                msg.data = [pose_y, pose_x, 30]
                publisher_sign.publish(msg)

            elif detected['3']['d']:
                print('30 DETECTED!')
                msg = Float32MultiArray()
                msg.data = [pose_y, pose_x, 30]
                publisher_sign.publish(msg)

            elif detected['6']['d']:
                print('60 DETECTED')
                msg = Float32MultiArray()
                msg.data = [pose_y, pose_x, 60]
                publisher_sign.publish(msg)

        if detected['1']['d'] and detected['5']['d']:
            print('15 DETECTED!')
            msg = Float32MultiArray()
            msg.data = [pose_y, pose_x, 15]
            publisher_sign.publish(msg)

    return


def loop():
    global subscriber_pose, subscriber_image, subscriber_waypoints #all the globals

    init()

    while not rospy.is_shutdown():
        start_time = time.time()
        rospy.sleep(max(CYCLE_TIME - (start_time - time.time()), 0))

if __name__ == "__main__":
    loop()
