import numpy as np
import rospy
import time
import math
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# CONSTANTS
CYCLE_TIME = 0.1
TURN_LEFT = 0
GO STRAIGHT = 1
TURN_RIGHT = 2
STOP = 3

publisher_complete = None
publisher_vel = None
subscriber_pose = None
subscriber_laser = None
state = None
pose_x = None
pose_y = None
pose_w = None

def poseCallback(data):
    global pose_x, pose_y, pose_w

    pose_x = data.pose.pose.position.x
    pose_y = data.pose.pose.position.y
    y,p,r = getRPY(pose.pose.orientation)
    pose_w = y

'''
borrowed from:
https://computergraphics.stackexchange.com/questions/8195/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr
'''
def getRPY(orientation):
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)

    roll = math.atan2(t0, t1)


    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2

    pitch = math.asin(t2)


    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return (yaw, pitch, roll)


def init():
    global subscriber_pose, publisher_complete, publisher_vel
    global state

    state = -1

    publisher_complete = rospy.publisher('/explore/complete', Bool, queue_size=5)
    publisher_vel = rospy.publisher('/cmd_vel', Twist, queue_size=5)
    subscriber_pose = rospy.Subscriber('/odom', Odometry, poseCallback)
    subscriber_laser = None

    rospy.init_node('explore')

def loop():
    global subscriber_pose, publisher_complete, publisher_vel
    global pose_x, pose_y, pose_w

    init()

    turns_taken = 0

    w0 = None
    x0 = None
    y0 = None

    while not rospy.is_shutdown():
        twist_msg = Twist()

        if state == -1:
            state = TURN_LEFT
        elif state == TURN_LEFT:
            if w0 is None:
                w0 = pose_w

            ###### ADJUST TO HANDLE TOLERANCE ######
            if abs(abs(pose_w - w0) - math.pi / 2.0) <= 0.05:
                # publish message to continue to turn left
                pass

            else:
                # publish message to stop
                state = GO_STRAIGHT
                w0 = None

        elif state == TURN_RIGHT:
            if w0 is None:
                w0 = pose_w
                turns_taken += 1

            ###### ADJUST TO HANDLE TOLERANCE ######
            if abs(abs(pose_w - w0) - math.pi / 2.0) <= 0.05:
                # publish message to continue to turn left
                pass

            else:
                # publish message to stop
                state = GO_STRAIGHT
                w0 = None

        elif state == GO_STRAIGHT:
            if x0 is None or y0 is None:
                x0 = pose_x
                y0 = pose_y

            if turns_taken == 0:
                state = TURN_RIGHT
                # go half distance
                

                pass
            elif turns_taken == 1:
                state = TURN_RIGHT
                # go full distance
                pass
            elif turns_taken == 2:
                state = TURN_RIGHT
                # go full distance
                pass
            elif turns_taken == 3:
                state = TURN_RIGHT
                # go full distance
                pass
            elif turns_taken == 4:
                state = STOP
                #go half distance then stop
                pass
            else:
                state = TURN_RIGHT
                # go full distance
        elif state == STOP:
            #publish exploration complete
            pass


        start_time = time.time()



        rospy.sleep(max(CYCLE_TIME - (start_time - time.time()), 0))
