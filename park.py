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

publisher_park = None
subscriber_navigated = None
subscriber_pose = None
subscriber_signs = None

navigation_complete = None
pose_x = None
pose_y = None

def init():
    global publisher_park, subscriber_navigated, subscriber_pose

    rospy.init_node('park')
    publisher_park = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=5)
    #check that this is right
    subscriber_navigated = rospy.Subscriber('/navigation/complete', Bool, navigationCallBack)
    #get this
    subscriber_signs = rospy.Subscriber(signsCallBack)
    subscriber_pose = rospy.Subscriber('/odom', Odometry, poseCallBack)

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
    do stuff

def loop():
    global publisher_park, subscriber_navigated, subscriber_pose, subscriber_signs
    global pose_x, pose_y
    init()
    while not rospy.is_shutdown():
        start_time = time.time()
        if navigation_complete is not None:
            #findBestSpot: get x, y, theta of the best spot
            #go to that waypoint
            goToGoal(x,y,t)
            #park
        rospy.sleep(max(CYCLE_TIME - (start_time - time.time()), 0))

def park():
    global publisher_park, subscriber_navigated, subscriber_pose, subscriber_signs
    #turn to face the spot
    #go in

def findBestSpot():
    global publisher_park, subscriber_navigated, subscriber_pose, subscriber_signs

    #find nearest spot with no signs
    #return a waypoint of that spot: x, y, theta

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
