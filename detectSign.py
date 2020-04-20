'''
Init function to start our detectSign node and instantiate publishers and subscribers
'''

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
