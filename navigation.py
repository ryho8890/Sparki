import rospy
import argparse
import time
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from nav_msgs.msg import Odometry


CYCLE_TIME = 0.100

publisher_navigate = None
publisher_at_waypoint = None
subscriber_cv_complete = None
subscriber_waypoints = None
subscriber_status = None
subscriber_pose = None


waypoints = None
status = None
pose_x = None
pose_y = None
middle_estimate = None

def init():
    global publisher_navigate, subscriber_waypoints

    rospy.init_node('navigation')
    publisher_navigate = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=5)
    subscriber_waypoints = rospy.Subscriber('/parkingbot/waypoints', Int32MultiArray, waypointCallBack)
    subscriber_status = rospy.Subscriber('/move_base/status', GoalStatusArray, statusCallback)
    subscriber_pose = rospy.Subscriber('/odom', Odometry, poseCallback)

def poseCallback(data):
    global pose_x, pose_y

    pose_x = data.pose.pose.position.x
    pose_y = data.pose.pose.position.y

def waypointCallBack(data):
    global waypoints, middle_estimate
    # reshape and put in correct resolution
    waypoints = np.array(data.data).reshape((-1, 2)) * (5 / 100.0)
    # take mean of x's to guesstimate the middle of the lot
    middle_estimate = np.mean(waypoints, 0)[1]

def statusCallback(data):
    global status
    try:
        status = data.status_list[-1].status
    except:
        pass

def getNextWaypoint():
    global pose_x,pose_y, waypoints

    dist_dict = {}

    count = 0
    for wp in waypoints:
        y,x = wp

        dist = np.sqrt((pose_x - x)**2 + (pose_y - y)**2)
        dist_dict[dist] = {
            'wp': wp,
            'i': count
        }

        count += 1

    dists = dist_dict.keys()
    minDist = min(dists)
    nwp = dist_dict[minDist]['wp']
    waypoints.pop(dist_dict[minDist]['i'])

    return nwp


def loop():
    global waypoints, status, middle_estimate
    global publisher_navigate, subscriber_waypoints


    init()

    while not rospy.is_shutdown():
        start_time = time.time()

        # idle until waypoints available
        if waypoints is None:
            rospy.sleep(max(CYCLE_TIME - (start_time - time.time()), 0))
            continue

        # new goal needed
        if status is None or status == 3:

            if len(waypoints) > 0:
                y,x = getNextWaypoint()

                if x > middle_estimate:
                    t = 0
                else:
                    t = np.pi

                goToGoal(x,y,t)

            else:
                # maybe publish that its reached the last waypoint or something idk man im tired AF.
                return


        rospy.sleep(max(CYCLE_TIME - (start_time - time.time()), 0))


def goToGoal(x,y,t):
    global publisher_navigate, subscriber_waypoints

    msg = PoseStamped()

    msg.header.frame_id = 'map'
    msg.pose.position.x = x
    msg.pose.position.y = y
    #msg.pose.position.z = 0

    q = convert2Quaternion(np.deg2rad(t), 0.0, 0.0)

    msg.pose.orientation = Quaternion(*q)

    rospy.sleep(1)
    msg.header.stamp = rospy.Time.now()
    publisher_navigate.publish(msg)


def convert2Quaternion(yaw, pitch, roll):
     qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
     qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
     qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
     qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

     return [qx, qy, qz, qw]

if __name__ == "__main__":
    loop()
