import rospy
import json
import copy
import time
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16
from math import pi, cos, sin, floor, ceil

# Load obstables img
ob_img=mpimg.imread('obstacles.png')

# MAP TESTING
'''
og_x = -1
og_y = -1
og = True
'''
initial_start = True
on_start = True

# GLOBALS
render_buffer = 0
ping_distance = 0
pose2D_sparki_odometry = None #Pose2D message object, contains x,y,theta members in meters and radians
#TODO: Track servo angle in radians
servo_angle = None
#TODO: Track IR sensor readings (there are five readings in the array: we've been using indices 1,2,3 for left/center/right)
ir_readings = {
        'L': -1,
        'C': -1,
        'R': -1
        }
#TODO: Create data structure to hold map representation
MAP_RES = 1 # cm

MAP_WID = 200 / MAP_RES # redund for 1cm res, but can be altered to support other resolutions
MAP_HEI = 200 / MAP_RES

world_map = [[0 for i in range(ceil(MAP_HEI))] for j in range(ceil(MAP_WID))]


# TODO: Use these variables to hold your publishers and subscribers
publisher_motor = None # rospy.Publisher('/sparki/motor_command', Float32MultiArray)
publisher_odom = None #rospy.Publisher('/sparki/set_odometry', Pose2D)
publisher_ping = None #rospy.Publisher('/sparki/ping_command', Empty)
publisher_servo = None #rospy.Publisher('/sparki/set_servo', Int16) 
subscriber_odometry = None #rospy.Subscriber('/sparki/odometry', Pose2D, callback=callback_update_odometry)
subscriber_state = None #rospy.Subscriber('/sparki/state', String, callback=callback_update_state) 

# rendering publisher
publisher_render = None

# CONSTANTS 
IR_THRESHOLD = 300 # IR sensor threshold for detecting black track. Change as necessary.
CYCLE_TIME = 0.1 # In seconds
RENDER_LIMIT = 1 # in seconds


def main():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, publisher_render
    global subscriber_odometry, subscriber_state
    global IR_THRESHOLD, CYCLE_TIME, RENDER_LIMIT
    global pose2D_sparki_odometry, servo_angle, ir_readings, ping_distance
    global render_buffer
    global servo_angle

    #TODO: Init your node to register it with the ROS core
    init()

    servos = list(range(0, 90, 15))

    while not rospy.is_shutdown():
        #TODO: Implement CYCLE TIME
        start_time = time.time()

        
        # push ping command 
        publisher_ping.publish(Empty())
        
        # MOTORS
        motor_left = 1.0
        motor_right = 1.0

        # LOOP CLOSURE
        if ir_readings['L'] < IR_THRESHOLD and ir_readings['C'] < IR_THRESHOLD and ir_readings['R'] < IR_THRESHOLD:
            # close the loop, do relevant things
            if not initial_start:
                if not on_start:
                    if servos:
                        new_servo_angle = servos.pop(0)
                        servo_msg = Int16(new_servo_angle)
                        publisher_servo.publish(servo_msg)
                        servo_angle = new_servo_angle
                
                    display_map()

                    on_start = True
        else:
            on_start = False
            initial_start = False


        # FOLLOW LINE
        if ir_readings['C'] < IR_THRESHOLD:
            motor_left = 1.0
            motor_right = 1.0
        else:
            if ir_readings['L'] < IR_THRESHOLD:
                motor_left = -1.0
                motor_right = 1.0
            if ir_readings['R'] < IR_THRESHOLD:
                motor_left = 1.0
                motor_right = -1.0

        motor_msg = Float32MultiArray()
        motor_msg.data = [float(motor_left), float(motor_right)]
        try:
            publisher_motor.publish(motor_msg)
        except:
            print(motor_left)
            print(motor_right)
            print(motor_msg)
            print(motor_msg.data)
        


        if render_buffer >= RENDER_LIMIT:
            render_buffer = 0
            publisher_render.publish(Empty())

        #TODO: Implement line following code here
        #      To create a message for changing motor speed, use Float32MultiArray()
        #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))

        

        #TODO: Implement loop closure here
        #if :
        #    rospy.loginfo("Loop Closure Triggered")

        #TODO: Implement CYCLE TIME
        render_buffer += CYCLE_TIME
        rospy.sleep(max(CYCLE_TIME - (start_time - time.time()), 0))
        


def init():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, publisher_render
    global subscriber_odometry, subscriber_state
    global pose2D_sparki_odometry, ping_distance, servo_angle
    global render_buffer

    render_buffer = 0
    ping_distance = 0

    #TODO: Set up your publishers and subscribers
    publisher_motor = rospy.Publisher('/sparki/motor_command', Float32MultiArray, queue_size=10)
    publisher_odom = rospy.Publisher('/sparki/set_odometry', Pose2D, queue_size=10)
    publisher_ping = rospy.Publisher('/sparki/ping_command', Empty, queue_size=10)
    publisher_servo = rospy.Publisher('/sparki/set_servo', Int16, queue_size=10)
    subscriber_odometry = rospy.Subscriber('/sparki/odometry', Pose2D, callback=callback_update_odometry)
    subscriber_state = rospy.Subscriber('/sparki/state', String, callback=callback_update_state)

    # rendering publisher
    publisher_render = rospy.Publisher('/sparki/render_sim', Empty, queue_size=10)

    # init the node
    rospy.init_node('Lab4')

    #TODO: Set up your initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    pose2D_sparki_odometry = Pose2D()

    #TODO: Set sparki's servo to an angle pointing inward to the map (e.g., 45)
    initial_servo_msg = Int16(70)
    publisher_servo.publish(initial_servo_msg)
    servo_angle = 70


def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2D_sparki_odometry
    global og_x, og_y, og
    #TODO: Copy this data into your local odometry variable
    pose2D_sparki_odometry = data
    '''
    if og_x == -1 and og_y == -1:
        og_x = floor(100 * data.x)
        og_y = floor(100 * data.y)
        print(og_x)
        print(og_y)
        print('---')


    if not og and abs(floor(100 * data.x) - og_x) < 3 and abs((floor(100 * data.y)) - og_y) < 3:
        print(data.x)
        print(data.y)

        display_map()
        og = True
    elif not abs(floor(100 * data.x) - og_x) < 3 and abs((floor(100 * data.y)) - og_y) < 3:
        og = False
    '''

def track_path(x, y):
    global world_map

    x = floor(100 * x)
    y = floor(100 * y)

    for i in range(x-3,x+4):
        for j in range(y-3, y+4):
            world_map[i][j] = 2

def callback_update_state(data):
    state_dict = json.loads(data.data) # Creates a dictionary object from the JSON string received from the state topic
    #TODO: Load data into your program's local state variables
    sensors = state_dict['light_sensors'][1:4]
    ir_readings['L'] = sensors[0]
    ir_readings['C'] = sensors[1]
    ir_readings['R'] = sensors[2]

    servo_angle = int(state_dict['servo'])

    try:
        ping_distance = float(state_dict['ping'])
        if ping_distance >= 0:
            x_r, y_r = convert_ultrasonic_to_robot_coords(ping_distance)
            x_w, y_w = convert_robot_coords_to_world(x_r, y_r)
            populate_map_from_ping(x_w, y_w)
            
    except:
        pass

def convert_ultrasonic_to_robot_coords(x_us):
    #TODO: Using US sensor reading and servo angle, return value in robot-centric coordinates
    #x_r, y_r = 0., 0.

    x_r = 100 * x_us * cos((2 * pi * servo_angle) / 180.0 )
    y_r = 100 * x_us * sin((2 * pi * servo_angle) / 180.0 )

    return x_r, y_r

def convert_robot_coords_to_world(x_r, y_r):
    #TODO: Using odometry, convert robot-centric coordinates into world coordinates
    #x_w, y_w = 0., 0.
    
    t_w = pose2D_sparki_odometry.theta
    x_w = pose2D_sparki_odometry.x * 100
    y_w = pose2D_sparki_odometry.y * 100

    x_w += (x_r * cos(t_w)) - (y_r * sin(t_w))
    y_w += (x_r * sin(t_w)) + (y_r * cos(t_w)) 

    return x_w, y_w

def populate_map_from_ping(x_ping, y_ping):
    global world_map
    #TODO: Given world coordinates of an object detected via ping, fill in the corresponding part of the map
    i = floor(x_ping)
    j = floor(y_ping)

    if world_map[i][j] != 2:
        world_map[i][j] = 1

def display_map():
    global world_map
    #TODO: Display the map
    hits = []
    for i in range(ceil(MAP_WID)):
        for j in range(ceil(MAP_HEI)):
            if world_map[i][j] == 1:
                x = i
                y = j
                xs = [x,x+1,x+1,x]
                ys = [y,y,y+1,y+1]
                hits.append({'x': xs, 'y':ys})

    plt.figure()
    #plt.imshow(ob_img)
    plt.axis('equal')
    for hit in hits:
        plt.fill(hit['x'], hit['y'], "b")

    plt.show()
            

def ij_to_cell_index(i,j):
    #TODO: Convert from i,j coordinates to a single integer that identifies a grid cell
    c = MAP_WID * j + i 
    return c

def cell_index_to_ij(cell_index):
    #TODO: Convert from cell_index to (i,j) coordinates
    j = cell_index / MAP_WID
    i = cell_index % MAP_WID
    return i, j


def cost(cell_index_from, cell_index_to):
    #TODO: Return cost of traversing from one cell to another
    i0, j0 = cell_index_to_if(cell_index_from)
    i1, j1 = cell_index_to_if(cell_index_to)

    cost = abs(i0 - i1) + abs(j0 - j1) # manhattan distance

    return cost

if __name__ == "__main__":
    main()


