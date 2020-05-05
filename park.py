import rospy
import time
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from nav_msgs.msg import Odometry


CYCLE_TIME = 0.100

publisher_park = None
subscriber_navigated = None
subscriber_pose = None
subscriber_signs = None
subscriber_waypoints = None

navigation_complete = None
waypoints = None
spot_signs = None
middle_estimate = None
pose_x = None
pose_y = None

def init():
    global publisher_park, subscriber_navigated, subscriber_pose

    rospy.init_node('park')

    publisher_park = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=5)
    subscriber_navigated = rospy.Subscriber('/navigation/complete', Bool, navigationCallBack)
    subscriber_signs = rospy.Subscriber('/parkingbot/sign', Float32MultiArray, signsCallBack)
    subscriber_waypoints = rospy.Subscriber('/parkingbot/waypoints', Float32MultiArray, waypointCallBack)
    subscriber_pose = rospy.Subscriber('/odom', Odometry, poseCallBack)

def waypointCallBack(data):
    global waypoints, middle_estimate

    if waypoints is None:
        waypoints = np.array(data.data).reshape((-1, 2))
        middle_estimate = -0.8
    else:
        np.vstack((waypoints, np.array(data.data).reshape((-1, 2))))

def poseCallBack(data):
    global pose_x, pose_y

    pose_x = data.pose.pose.position.x
    pose_y = data.pose.pose.position.y

def navigationCallBack(data):
    global navigation_complete

    if data is None:
        return
    else:
        navigation_complete = data

def signsCallBack(data):
    global spot_signs

    if spot_signs is None:
        spot_signs = np.array(data.data).reshape((-1, 3))
    else:
        np.vstack((spot_signs, np.array(data.data).reshape((-1, 3))))

def loop():
    global publisher_park, subscriber_navigated, subscriber_pose, subscriber_signs
    global pose_x, pose_y, CYCLE_TIME
    init()
    while not rospy.is_shutdown():
        start_time = time.time()
        if navigation_complete is not None:
            print("park starting")
            x, y, t = findBestSpot()
            goToGoal(x,y,t)
            park()
        rospy.sleep(max(CYCLE_TIME - (start_time - time.time()), 0))

def park():
    global publisher_park

    msg = PoseStamped()

    msg.header.frame_id = 'map'
    msg.pose.position.y = pose_y

    if x > middle_estimate:
        msg.pose.position.x = pose_x + 0.5
    else:
        msg.pose.position.x = pose_x - 0.5

    rospy.sleep(1)
    msg.header.stamp = rospy.Time.now()
    publisher_park.publish(msg)

def findBestSpot():
    global publisher_park, subscriber_navigated, subscriber_pose, subscriber_signs
    global waypoints, spot_signs, pose_x, pose_y

    #there is a waypoint where there is no spot
    flag = 0
    time = 0
    x = 0
    y = 0
    for waypoint in waypoints:
        for spot in spot_signs:
            if spot[1] == waypoint[0] and spot[0] == waypoint[1]:
                flag = 0
                continue
            else:
                flag = 1
        if flag == 1:
            x = waypoint[1]
            y = waypoint[0]
            if x > middle_estimate:
                t = 0
            else:
                t = np.pi
            return (x, y, t)
        else:
            for spot in spot_signs:
                if spot[2] != -1:
                    if spot[2] > time:
                        time = spot[2]
                        x = spot[1]
                        y = spot[0]
            if x > middle_estimate:
                t = 0
            else:
                t = np.pi
            return (x, y, t)
    print('No place to park!')
    return



def goToGoal(x,y,t):
    global publisher_park

    msg = PoseStamped()

    msg.header.frame_id = 'map'
    msg.pose.position.x = x
    msg.pose.position.y = y

    q = convert2Quaternion(np.deg2rad(t), 0.0, 0.0)

    msg.pose.orientation = Quaternion(*q)

    rospy.sleep(1)
    msg.header.stamp = rospy.Time.now()
    publisher_park.publish(msg)


def convert2Quaternion(yaw, pitch, roll):
     qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
     qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
     qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
     qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

     return [qx, qy, qz, qw]

if __name__ == "__main__":
    loop()
