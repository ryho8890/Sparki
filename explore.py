import numpy as np
import rospy
import time
import math
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# CONSTANTS
CYCLE_TIME = 0.05
TURN_LEFT = 0
GO_STRAIGHT = 1
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
    y,p,r = getRPY(data.pose.pose.orientation)
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

    publisher_complete = rospy.Publisher('/explore/complete', Bool, queue_size=5)
    publisher_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    subscriber_pose = rospy.Subscriber('/odom', Odometry, poseCallback)
    subscriber_laser = None

    rospy.init_node('explore')

def loop():
    global subscriber_pose, publisher_complete, publisher_vel
    global pose_x, pose_y, pose_w, state

    init()

    turns_taken = 0

    w0 = None
    x0 = None
    y0 = None

    while not rospy.is_shutdown():
        start_time = time.time()

        twist_msg = Twist()

        if state == -1:
            state = TURN_LEFT
        elif state == TURN_LEFT:
            if w0 is None:
                w0 = pose_w

            sign0 = int(max(np.sign(w0), 0))
            sign1 = int(max(np.sign(pose_w), 0))

            # check if different signed valuesv
            if sign0 and not sign1:
                pose_w += 2 * math.pi


            ###### ADJUST TO HANDLE TOLERANCE ######
            if abs(pose_w - w0) < (math.pi / 2.0):
                # publish message to continue to turn left
                twist_msg.angular.z = 0.1
                publisher_vel.publish(twist_msg)
            else:
                # publish message to stop
                twist_msg.angular.z = 0
                publisher_vel.publish(twist_msg)
                state = GO_STRAIGHT
                w0 = None

        elif state == TURN_RIGHT:
            if w0 is None:
                w0 = pose_w

            sign0 = int(max(np.sign(w0), 0))
            sign1 = int(max(np.sign(pose_w), 0))

            # check if different signed values
            if sign1 and not sign0:
                pose_w -= 2 * math.pi

            if abs(pose_w - w0) < (math.pi / 2.0):
                # publish message to continue to turn left
                twist_msg.angular.z = -0.1
                publisher_vel.publish(twist_msg)
            else:
                # publish message to stop
                twist_msg.angular.z = 0
                publisher_vel.publish(twist_msg)
                state = GO_STRAIGHT
                w0 = None
                turns_taken += 1

        elif state == GO_STRAIGHT:
            if x0 is None or y0 is None:
                x0 = pose_x
                y0 = pose_y

            if turns_taken == 0:
                # go half distance
                if np.sqrt((pose_x - x0)**2 + (pose_y - y0)**2) < 0.75:
                    twist_msg.linear.x = 0.2
                    publisher_vel.publish(twist_msg)
                    #print(np.sqrt((pose_x - x0)**2 + (pose_y - y0)**2))
                else:
                    state = TURN_RIGHT
                    twist_msg.linear.x = 0
                    publisher_vel.publish(twist_msg)
                    x0 = None
                    y0 = None

            elif turns_taken == 1:
                # go full distance
                if np.sqrt((pose_x - x0)**2 + (pose_y - y0)**2) < 6:
                    twist_msg.linear.x = 0.2
                    publisher_vel.publish(twist_msg)
                    #print(np.sqrt((pose_x - x0)**2 + (pose_y - y0)**2))
                else:
                    state = TURN_RIGHT
                    twist_msg.linear.x = 0
                    publisher_vel.publish(twist_msg)
                    x0 = None
                    y0 = None

            elif turns_taken == 2:
                if np.sqrt((pose_x - x0)**2 + (pose_y - y0)**2) < 1.5:
                    twist_msg.linear.x = 0.2
                    publisher_vel.publish(twist_msg)
                    #print(np.sqrt((pose_x - x0)**2 + (pose_y - y0)**2))
                else:
                    state = TURN_RIGHT
                    twist_msg.linear.x = 0
                    publisher_vel.publish(twist_msg)
                    x0 = None
                    y0 = None
            elif turns_taken == 3:
                # go full distance
                if np.sqrt((pose_x - x0)**2 + (pose_y - y0)**2) < 6:
                    twist_msg.linear.x = 0.2
                    publisher_vel.publish(twist_msg)
                    #print(np.sqrt((pose_x - x0)**2 + (pose_y - y0)**2))
                else:
                    state = TURN_RIGHT
                    twist_msg.linear.x = 0
                    publisher_vel.publish(twist_msg)
                    x0 = None
                    y0 = None
            elif turns_taken == 4:
                # go half distance
                if np.sqrt((pose_x - x0)**2 + (pose_y - y0)**2) < 0.75:
                    twist_msg.linear.x = 0.2
                    publisher_vel.publish(twist_msg)
                    #print(np.sqrt((pose_x - x0)**2 + (pose_y - y0)**2))
                else:
                    state = TURN_RIGHT
                    twist_msg.linear.x = 0
                    publisher_vel.publish(twist_msg)
                    x0 = None
                    y0 = None
            else:
                state = STOP
        elif state == STOP:
            #publish exploration complete
            twist_msg.linear.x = 0
            publisher_vel.publish(twist_msg)
            complete_msg = Bool(True)
            publisher_complete.publish(complete_msg)
            return

        rospy.sleep(max(CYCLE_TIME - (start_time - time.time()), 0))


if __name__ == "__main__":
    loop()
