import numpy as np
import rospy
import time
import math
from std_msgs.msg import *
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, MapMetaData

import cv2

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

def getWaypoints():
    global map, publisher_waypoints

    parkingSpotTemplate = np.array([
    [0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
    ])

    height, width = map.shape

    th, tw = parkingSpotTemplate.shape

    wps = []

    line_found = None
    for r in range(height):
        for c in range(width):
            hr = r + th
            hc = c + tw

            if hr > height or hc > width:
                continue

            window = map[r:hr, c:hc]

            score = max(np.sum(window == parkingSpotTemplate) / float(tw * th),
                        np.sum(window == np.rot90(parkingSpotTemplate, 2)) / float(tw * th))

            if score > 0.82:
                wp_x = int(c + (1.5 * tw))
                wp_y = int(r + (th / 2.0))
                wps.append([wp_y, wp_x])


    print('publishing...')
    msg = Int32MultiArray()
    msg.data = list(np.int32(wps).flatten())
    publisher_waypoints.publish(msg)
    print('shutting down.')

    return


def exploreCallBack(data):
    global explore_complete
    if data is None:
        return
    else:
        explore_complete = data

def mapCallBack(data):
    global explore_complete, map, complete
    # do nothing until done exploring
    #if explore_complete is None or not explore_complete:
    #     return

    # do nothing if already done it
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

            if raw_value < 0:
                value = -1
            elif raw_value < 50:
                value = 0
            else:
                value = 1

            map[r][c] = value

    map = np.rot90(map, k=2)

    complete = True

def init():
    global subscriber_map, subscriber_explore
    global publisher_waypoints
    global explore_complete
    subscriber_explore = rospy.Subscriber('/explore/complete', Bool, exploreCallBack)
    subscriber_map = rospy.Subscriber('/map', OccupancyGrid, mapCallBack)
    publisher_waypoints = rospy.Publisher('/parkingbot/waypoints', Int32MultiArray, queue_size=5)

    explore_complete = False

    rospy.init_node('waypoints')

def loop():
    global CYCLE_TIME, complete

    init()
    while not rospy.is_shutdown():
        start_time = time.time()
        if complete:
            getWaypoints()
            return
        rospy.sleep(max(CYCLE_TIME - (start_time - time.time()), 0))

if __name__ == "__main__":
    loop()
