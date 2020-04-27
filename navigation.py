import rospy
import argparse
import time
import numpy as np
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import *
from move_base_msgs.msg import *

CYCLE_TIME = 0.100

publisher_navigate = None
publisher_at_waypoint = None
subscriber_cv_complete = None
subscriber_waypoints = None
subscriber_status = None

waypoints = None

def init():
    global publisher_navigate, subscriber_waypoints

    rospy.init_node('navigation')
    publisher_navigate = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    subscriber_waypoints = rospy.Subscriber('/parkingbot/waypoints', Int32MultiArray, waypointCallBack)

def waypointCallBack(data):
    # reshape and put in correct resolution
    waypoints = np.array(data).reshape((-1, 2)) * (5 / 100.0)

def loop():

    init()

    while not rospy.is_shutdown():
        start_time = time.time()
        if waypoints is None:
            continue

        rospy.sleep(max(CYCLE_TIME - (start_time - time.time()), 0))


    print(waypoints)
    goal_x = float(waypoints[0][0])
    goal_y = float(waypoints[0][1])
    goal_theta = float(0)

    print('Message sent.')
    start_time = time.time()

    if (goal_x and goal_y and goal_theta):
        status = goToGoal(goal_x, goal_y, goal_theta)

    return status

def goToGoal(x,y,t):
    global publisher_navigate, subscriber_waypoints

    msg = PoseStamped()

    msg.header.frame_id = 'map'
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0

    q = convert2Quaternion(np.deg2rad(t), 0.0, 0.0)

    msg.pose.orientation = Quaternion(*q)

    rospy.sleep(1)
    msg.header.stamp = rospy.Time.now()
    publisher_navigate.publish(msg)

    return 1

def convert2Quaternion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

if __name__ == "__main__":
    if not loop():
        raise Exception("FUBAR")
    else:
        print("Message received.")
