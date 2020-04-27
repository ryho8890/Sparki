import numpy as np
import rospy
import time
import math
from std_msgs.msg import *
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, MapMetaData

# CONSTANTS
CYCLE_TIME = 0.1
# Ros players
subscriber_explore = None
subscriber_map = None
publisher_waypoints = None

# others
map = None
waypoints = []
nodes = []
explore_complete = None

def foundInWaypoints(waypoints, point):
    for i in waypoints:
        if i == point:
            return 1
    return 0

def getWaypoints(height, width):
    global map
    for y in range(height):
        for x in range(width):
            if map[y][x] == 1:
                nodes.append([x, y])
    for i in nodes:
        x = i[0]
        y = i[1]
        #left side spots
        if map[y][130] == 1: #if a line detected
            count = 0
            row = y+1
            while map[row][130] == 0: #count spaces until next line detected
                count = count + 1
                row = row + 1
            if count != 0 and foundInWaypoints(waypoints, [142, y + count/2]) == 0:
                waypoints.append([142, y + count/2]) #add the midpoint between the two lines
        #same thing on right side
        if map[y][200] == 1:
            count = 0
            row = y+1
            while map[row][200] == 0:
                count = count + 1
                row = row + 1
            if count != 0 and foundInWaypoints(waypoints, [188, y + count/2]) == 0:
                waypoints.append([188, y + count/2])
    #print(waypoints)
    pass

def exploreCallBack(data):
    global explore_complete
    if data is None:
        return
    else:
        explore_complete = data

def mapCallBack(data):
    global explore_complete, map
    # do nothing until done exploring
    # if explore_complete is None or not explore_complete:
    #     return
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
        if waypoints != None:
            for i in waypoints:
                point = Float32MultiArray()
                d = MultiArrayDimension()
                d.label = "x"
                d.size = 1
                d.stride = 1
                point.layout.dim.append(d)

                point.layout.data_offset = 1
                point.data = [i[0], i[1]]
        rospy.sleep(max(CYCLE_TIME - (start_time - time.time()), 0))

if __name__ == "__main__":
    loop()
