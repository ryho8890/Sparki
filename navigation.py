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
publisher_complete = None
subscriber_cv_complete = None
subscriber_waypoints = None
subscriber_status = None
subscriber_pose = None


waypoints = None
status = None
pose_x = None
pose_y = None
middle_estimate = None
image_captured = None

def init():
    global publisher_navigate, subscriber_waypoints, subscriber_status
    global subscriber_pose, subscriber_captured, publisher_atWP, publisher_complete

    rospy.init_node('navigation')
    publisher_navigate = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=5)
    publisher_atWP = rospy.Publisher('/parkingbot/atWP', Bool, queue_size=5)
    publisher_complete = rospy.Publisher('/navigation/complete', Bool, queue_size=5)
    subscriber_waypoints = rospy.Subscriber('/parkingbot/waypoints', Float32MultiArray, waypointCallBack)
    subscriber_status = rospy.Subscriber('/move_base/status', GoalStatusArray, statusCallback)
    subscriber_pose = rospy.Subscriber('/odom', Odometry, poseCallback)
    subscriber_captured = rospy.Subscriber('/parkingbot/captured', Bool, capturedCallback)

def capturedCallback(data):
    global image_captured

    if data is None:
        return
    else:
        image_captured = data

def poseCallback(data):
    global pose_x, pose_y

    pose_x = data.pose.pose.position.x
    pose_y = data.pose.pose.position.y

def waypointCallBack(data):
    global waypoints, middle_estimate

    if waypoints is None:
        # reshape and put in correct resolution
        waypoints = np.array(data.data).reshape((-1, 2)) # * (5 / 100.0)
        # take mean of x's to guesstimate the middle of the lot
        middle_estimate = -0.8 #np.mean(waypoints, 0)[1]
    else:
        np.vstack((waypoints, np.array(data.data).reshape((-1, 2))))

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
        try:
            y,x = wp
        except:
            wp = [wp]
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

    waypoints = np.delete(waypoints, dist_dict[minDist]['i'], 0)

    return nwp


def loop():
    global waypoints, status, middle_estimate, image_captured
    global publisher_navigate, subscriber_waypoints, publisher_atWP, publisher_complete


    init()


    while not rospy.is_shutdown():
        start_time = time.time()

        # idle until waypoints available
        if waypoints is None:
            rospy.sleep(max(CYCLE_TIME - (start_time - time.time()), 0))
            continue

        # new goal needed
        if status is None or status == 3:

            if status == 3:
                if image_captured is None:
                    msg = Bool()
                    msg.data = True
                    publisher_atWP.publish(msg)
                    print('Goal Reached')
                    image_captured = True

            if not image_captured is None and not image_captured:
                print("continuing")
                continue
            elif not image_captured is None and image_captured:
                print("none")
                image_captured = None
            print("out")
            if len(waypoints) > 0:
                y,x = getNextWaypoint()

                print(x, y, middle_estimate)

                if x > middle_estimate:
                    t = 0
                else:
                    t = np.pi

                goToGoal(x,y,t)

            else:
                # maybe publish that its reached the last waypoint or something idk man im tired AF.
                if len(waypoints) == 0:
                    publisher_complete.publish(Bool(True))
                return


        rospy.sleep(max(CYCLE_TIME - (start_time - time.time()), 0))


def goToGoal(x,y,t):
    global publisher_navigate, subscriber_waypoints

    msg = PoseStamped()

    msg.header.frame_id = 'map'
    msg.pose.position.x = x
    msg.pose.position.y = y
    #msg.pose.position.z = 0

    q = convert2Quaternion(t, 0.0, 0.0)

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
