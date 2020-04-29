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
origin_x = None
origin_y = None
resolution = None

def getWaypoints():
    global map, publisher_waypoints, origin_x, origin_y, resolution

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
                wps.append([r,c])


    middle_estimate = np.mean(wps, 0)[1]

    for i in range(len(wps)):
        y,x = tuple(wps[i])

        if x > middle_estimate:
            mult = -0.4
        else:
            mult = 1.4

        wp_x = ((x + (mult * tw)) * resolution) + origin_x
        wp_y = ((y - (th / 2.0)) * resolution) + origin_y

        wps[i] = [wp_y, wp_x]



    print('publishing...')
    msg = Float32MultiArray()
    msg.data = list(np.array(wps).flatten())
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
    global explore_complete, map, complete, origin_x, origin_y, resolution
    # do nothing until done exploring
    if explore_complete is None or not explore_complete:
         return

    # do nothing if already done it
    if complete is not None and complete:
        return

    map_raw = np.array(data.data)

    width = int(data.info.width)
    height = int(data.info.height)

    origin = data.info.origin
    origin_x = origin.position.x
    origin_y = origin.position.y
    resolution = data.info.resolution



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

    #map = np.rot90(map, k=2)

    complete = True

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
