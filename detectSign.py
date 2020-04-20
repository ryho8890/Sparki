import numpy as np
import rospy
import cv2
import time

# CONSTANTS
CYCLE_TIME = 0.1

# GLOBALS
waypoints = None
subscriber_waypoints = None
subscriber_image = None
publisher_sign_detection = None


'''
Init function to start our detectSign node and instantiate publishers and subscribers
'''
def init():
    
    pass

'''
Ryan!
/odom

nearWaypoint
input(pose, waypoints)

pseudo-code:
for wp in wps:
    if within couple cm's:
        return true
    else
        continue
'''

'''
Nick!

takePicture
input: None
returns: np array of whatever ros gives us

pseudo-code:
 TBD
'''

'''
analyzePicture
input: np array from takePicture
outPut: what type of sign it is
'''

'''
Nick!

loop

main ros loop
while ros isnt shutdown
    get pose
    check if near any waypoint
    if yes:
        call take pic
    else:
        pass (do nothing)
'''
def loop():
    global x #all the globals

    init()

    while not rospy.is_shutdown():
        start_time = time.time()
