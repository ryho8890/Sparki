import rospy
import json
import copy
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16


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

    #TODO: Init your node to register it with the ROS core
    init()

    while not rospy.is_shutdown():
        #TODO: Implement CYCLE TIME
        start_time = time.time()

        
        # push ping command 
        publisher_ping.publish(Empty())
        
        # Line Following
        motor_left = None
        motor_right = None

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
        motor_msg.data = [motor_left, motor_right]
        
        publisher_motor.publish(motor_msg)
        


        if render_buffer >= RENDER_LIMIT:
            render_buffer = 0
            publisher_render.publish(Empty())

        #TODO: Implement line following code here
        #      To create a message for changing motor speed, use Float32MultiArray()
        #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))

        

        #TODO: Implement loop closure here
        if False:
            rospy.loginfo("Loop Closure Triggered")

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
    initial_servo_msg = Int16(45)
    publisher_servo.publish(initial_servo_msg)
    servo_angle = 45


def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2D_sparki_odometry
    #TODO: Copy this data into your local odometry variable
    pose2D_sparki_odometry = data
    print(data.x)

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
    except:
        pass

def convert_ultrasonic_to_robot_coords(x_us):
    #TODO: Using US sensor reading and servo angle, return value in robot-centric coordinates
    x_r, y_r = 0., 0.
    return x_r, y_r

def convert_robot_coords_to_world(x_r, y_r):
    #TODO: Using odometry, convert robot-centric coordinates into world coordinates
    x_w, y_w = 0., 0.

    return x_w, y_w

def populate_map_from_ping(x_ping, y_ping):
    #TODO: Given world coordinates of an object detected via ping, fill in the corresponding part of the map
    pass

def display_map():
    #TODO: Display the map
    pass

def ij_to_cell_index(i,j):
    #TODO: Convert from i,j coordinates to a single integer that identifies a grid cell
    return 0

def cell_index_to_ij(cell_index):
    #TODO: Convert from cell_index to (i,j) coordinates
    return 0, 0


def cost(cell_index_from, cell_index_to):
    #TODO: Return cost of traversing from one cell to another
    return 0

if __name__ == "__main__":
    main()


