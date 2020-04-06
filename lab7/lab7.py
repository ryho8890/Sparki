import rospy
import argparse
import time
import numpy as np
from geometry_msgs.msg import *

g_CYCLE_TIME = 0.100

pub_goal = None

def loop(args):
    init()
    
    goal_x = float(args.x_goal[0])
    goal_y = float(args.y_goal[0])
    goal_theta = float(args.theta_goal[0])
    
    print('Message sent.')
    start_time = time.time()
        
    if (goal_x and goal_y and goal_theta):
        status = goToGoal(goal_x, goal_y, goal_theta)

    return status
    
def init():
    global pub_goal

    rospy.init_node('Lab7')
    pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

def goToGoal(x,y,t):
    global pub_goal

    msg = PoseStamped()
        
    msg.header.frame_id = 'map'
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0

    q = convert2Quaternion(np.deg2rad(t), 0.0, 0.0)

    msg.pose.orientation = Quaternion(*q)

    rospy.sleep(1)
    msg.header.stamp = rospy.Time.now()
    pub_goal.publish(msg)

    return 1

def convert2Quaternion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="RVIZ Navigation Stack")
    parser.add_argument('-x','--x_goal', nargs=1, default=[0.5], help='Goal X')
    parser.add_argument('-y','--y_goal', nargs=1, default=[0.5], help='Goal Y')
    parser.add_argument('-t','--theta_goal', nargs=1, default=[69], help='Goal Theta')
    
    args = parser.parse_args()

    if not loop(args):
        raise Exception("FUBAR")
    else:
        print("Message received.")
