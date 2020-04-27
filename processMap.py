import numpy as np
import rospy
import time
import math
import cv2
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

# TEMPORARY
import matplotlib as mpl
import matplotlib.pyplot as plt

# CONSTANTS
CYCLE_TIME = 0.1
# Ros players
subscriber_explore = None
subscriber_map = None
publisher_waypoints = None

# others
map = None
explore_complete = None
complete = None

def getWaypoints(height, width):
    global map

    midLine = width // 2

    for r in range(height):
        for c in range(width):
            pass

def exploreCallBack(data):
    global explore_complete

    if data is None:
        return
    else:
        explore_complete = data

def mapCallBack(data):
    global complete, map

    # do nothing until done exploring
    if explore_complete is None or not explore_complete:
        return

    # do nothing is already done it
    if complete is not None and complete:
        return

    map_raw = np.array(data.data)

    width = int(data.info.width)
    height = int(data.info.height)

    map = np.zeros((height, width))

    for r in range(height):
        for c in range(width):
            flat_i = (r * width) + c
            raw_value = map_raw[flat_i]

            # check if unknown
            if raw_value < 0:
                value = 0
            # check if prob of obstacle less than 50%
            elif raw_value < 50:
                value = 0
            # otherwise, confident of obstacle
            else:
                value = 1

            map[r][c] = value


    map = np.rot90(map, k=2)

    complete = True

    getWaypoints(height, width)

def init():
    global subscriber_map, subscriber_explore
    global publisher_waypoints
    global explore_complete

    subscriber_explore = rospy.Subscriber('/explore/complete', Bool, exploreCallBack)
    subscriber_map = rospy.Subscriber('/map', OccupancyGrid, mapCallBack)
    publisher_waypoints = rospy.Publisher('/parkingbot/waypoints', Float32MultiArray, queue_size=5)

    explore_complete = False

    rospy.init_node('waypoints')

def loop():
    global CYCLE_TIME

    init()

    while not rospy.is_shutdown():
        start_time = time.time()
        pass
        rospy.sleep(max(CYCLE_TIME - (start_time - time.time()), 0))

if __name__ == "__main__":
    loop()
