#!/usr/bin/python
# CSCI 3302 Sparki Simulator Version 1.1
# Hastily authored by Brad Hayes (bradley.hayes@colorado.edu)
# If getting ImageTk errors, run: sudo apt-get install python-imaging-tk

import math
import rospy
import sys
import json
import argparse
import time
import copy
from PIL import Image, ImageTk
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int16, String, Float32MultiArray, Empty
import numpy as np

if sys.version_info[0] == 2:
  import Tkinter as tk # Python 2
else:
  import tkinter as tk # Python 3

g_namespace = ""


# Driver Constants
CYCLE_TIME = 0.05 #0.05 # 50ms cycle time
MAP_RESOLUTION = 0.0015 # meters per pixel
MAP_SIZE_X = 1200 # Default map size in pixels
MAP_SIZE_Y = 800 # Default map size in pixels
SPARKI_SIZE_RADIUS = 0.08 # 0.08m radius == 6.29in diameter
SPARKI_ULTRASONIC_MAX_DIST = .75 # 0.75m max range for ultrasonic sensor

# ***** SERVO POSITIONS ***** #
SPARKI_SERVO_LEFT = 80
SPARKI_SERVO_CENTER = 0
SPARKI_SERVO_RIGHT = -80
SPARKI_SPEED = 0.08 #0.0278 # 100% speed in m/s
SPARKI_AXLE_DIAMETER = 0.085 # Distance between wheels, meters
SPARKI_WHEEL_RADIUS = 0.03 # Radius of wheels, meters
MAP_PADDING = round(SPARKI_SIZE_RADIUS / MAP_RESOLUTION)


# ROBOT STATE VARS
g_pose = None # Pose2D
g_motors = None # [Float, Float] each Float in range [-1,1]
g_ir_sensors = None # 5 Int Array in 0-1000 intensity
g_us_sensor = None # Float (in centimeters)
g_servo_angle = None
g_world_starting_pose = None # Pose2D
g_ping_requested = None # Bool: include US distance in next state broadcast
g_world_obstacles = None # Grid to indicate presence of obstacles [0,255]. 0 is free, otherwise occupied
g_world_surface = None # Grayscale image of world surface [0,255] (e.g., line following map or equivalent). 0 is empty, otherwise has line

# Publishers and Subscribers
g_pub_state = None
g_pub_odom = None
g_sub_motors = None
g_sub_ping = None
g_sub_odom = None
g_sub_servo = None
g_sub_render = None

# Visualization
g_tk_window = None
g_tk_label = None
g_render_requested = None
g_render_image_base = None

def recv_motor_command(msg):
    global g_motors
    if len(msg.data) != 2:
        rospy.logerr("Sparki given motor commands with too many entries! (Got %d expected 2)" % len(msg.data))
        g_motors = [0,0] # Stop robot
        return
    g_motors = [msg.data[0], msg.data[1]]

def recv_ping(msg):
    global g_ping_requested
    g_ping_requested = True

def recv_render(msg):
    global g_render_requested
    g_render_requested = True

def set_odometry(msg):
    global g_pose
    g_pose = copy.copy(msg)
    g_pose.theta = -g_pose.theta # Negate theta

def set_servo(msg):
    global g_servo_angle
    new_angle = msg.data

    g_servo_angle = math.radians(max(SPARKI_SERVO_RIGHT, min(SPARKI_SERVO_LEFT,-new_angle))) # Negate theta

def get_ir_reading():
    # Find the IR readings 'underneath' the Sparki robot and return them
    # Find pixels halfway between the center and front of the robot
    global g_pose, g_world_surface, SPARKI_SIZE_RADIUS, MAP_RESOLUTION, MAP_SIZE_X, MAP_SIZE_Y
    center_point = [g_pose.x, g_pose.y]
    sparki_sensor_center_point_offset = 0.5 * SPARKI_SIZE_RADIUS
    forward_point = (center_point[0] + math.cos(g_pose.theta)*sparki_sensor_center_point_offset, \
                     center_point[1] + math.sin(g_pose.theta)*sparki_sensor_center_point_offset)

    # Find map locations that IR sensors are 'over' in (x,y) [meters]
    left_line_ir_point = (forward_point[0] - math.sin(g_pose.theta)*SPARKI_SIZE_RADIUS/2, forward_point[1] + math.cos(g_pose.theta)*SPARKI_SIZE_RADIUS/2)
    left_ir_point = (forward_point[0] - math.sin(g_pose.theta)*SPARKI_SIZE_RADIUS/4, forward_point[1] + math.cos(g_pose.theta)*SPARKI_SIZE_RADIUS/4)
    center_ir_point = (forward_point[0], forward_point[1])
    right_ir_point = (forward_point[0] + math.sin(g_pose.theta)*SPARKI_SIZE_RADIUS/4, forward_point[1] - math.cos(g_pose.theta)*SPARKI_SIZE_RADIUS/4)
    right_line_ir_point = (forward_point[0] + math.sin(g_pose.theta)*SPARKI_SIZE_RADIUS/2, forward_point[1] - math.cos(g_pose.theta)*SPARKI_SIZE_RADIUS/2)

    # Convert to pixel locations
    pixel_coords = [0] * 5
    for i, coord in enumerate([left_line_ir_point, left_ir_point, center_ir_point, right_ir_point, right_line_ir_point]):
      pixel_coords[i] = (int(coord[0] / MAP_RESOLUTION), int(coord[1] / MAP_RESOLUTION))
      if pixel_coords[i][0] < 0 or pixel_coords[i][0] >= MAP_SIZE_X or pixel_coords[i][1] < 0 or pixel_coords[i][1] >= MAP_SIZE_Y:
        rospy.logerr("IR Sensor is off the map! Coords: %s" % (str(pixel_coords[i])))
        return [1000] * 5

    # Look up pixel coords on world map
    ir_readings = [1000] * 5
    for i, coord in enumerate(pixel_coords):
      ir_readings[i] = int(1000 * (255. - g_world_surface[coord[1],coord[0]]) / 255.)

    return ir_readings

def get_ping_reading():
    # Return distance from robot of an object in the line-of-sight of ultrasonic sensor
    # Returns -1 if nonesparki-sim.zip
    global g_servo_angle, g_world_obstacles, g_pose, MAP_RESOLUTION, MAP_SIZE_X, MAP_SIZE_Y, SPARKI_ULTRASONIC_MAX_DIST

    # Cast a ray until hitting map boundary or SPARKI_ULTRASONIC_MAX_DIST meters away
    offset = [math.cos(g_pose.theta + g_servo_angle), math.sin(g_pose.theta + g_servo_angle)]

    # Sample 500 points along the sensor line
    num_increments = 500
    for incr in range(1,num_increments):
      distance = (float(incr) / num_increments) * SPARKI_ULTRASONIC_MAX_DIST # Distance from robot being checked
      test_point_meters = [g_pose.x + distance * offset[0], g_pose.y + distance * offset[1]]
      test_point_pixels = [int(test_point_meters[0] / MAP_RESOLUTION), int(test_point_meters[1] / MAP_RESOLUTION)]
      if test_point_pixels[0] < 0 or test_point_pixels[1] < 0 or test_point_pixels[0] >= MAP_SIZE_X or test_point_pixels[1] >= MAP_SIZE_Y:
        # print("Sampled off map: %s" % (str(test_point_pixels)))
        return -1 # Sampling off the map
      if g_world_obstacles[test_point_pixels[1],test_point_pixels[0]] > 0:
        return distance * SPARKI_ULTRASONIC_MAX_DIST # Obstacle found

    return -1


def load_img_to_bool_matrix(img_filename):
    global MAP_SIZE_X, MAP_SIZE_Y

    if img_filename is None:
        grid = np.zeros([800,1200])
        return grid

    img = Image.open(img_filename)

    MAP_SIZE_X = img.width
    MAP_SIZE_Y = img.height

    grid = np.zeros([img.height, img.width])
    for y in range(img.height):
        for x in range(img.width):
            pixel = img.getpixel((x,y))
            grid[y,x] = 255 - pixel[0] # Dark pixels have high values to indicate being occupied/having something interesting

    return grid

def init(args):
    # Initialize all publishers, subscribers, state variables
    global g_namespace, g_pose, g_ir_sensors, g_us_sensor, g_world_starting_pose, g_world_obstacles, g_world_surface, g_motors, g_ping_requested, g_servo_angle
    global g_pub_state, g_pub_odom, g_sub_motors, g_sub_ping, g_sub_odom, g_sub_servo, g_render_requested, g_sub_render
    global g_tk_window, g_tk_label, g_render_image_base

    g_namespace = args.namespace

    rospy.init_node("sparki_simulator_%s" % g_namespace)

    g_ir_sensors = [0]*5
    g_us_sensor = -1
    g_motors = [0.,0.]
    g_servo_angle = 0
    g_ping_requested = False
    g_world_starting_pose = Pose2D()
    g_world_starting_pose.x, g_world_starting_pose.y, g_world_starting_pose.theta = args.startingpose[0], args.startingpose[1], args.startingpose[2]
    g_pose = copy.copy(g_world_starting_pose)

    g_world_obstacles = load_img_to_bool_matrix(args.obstacles)
    g_world_surface = load_img_to_bool_matrix(args.worldmap)

    if (g_world_obstacles.size != g_world_surface.size):
      g_world_obstacles = load_img_to_bool_matrix(None)
      g_world_surface = load_img_to_bool_matrix(None)
      rospy.logerr("Obstacle map and surface map have different dimensions. Resetting to blank!")

    MAP_SIZE_X = g_world_obstacles.shape[1]
    MAP_SIZE_Y = g_world_obstacles.shape[0]

    g_pub_odom = rospy.Publisher("/%s/odometry" % g_namespace, Pose2D, queue_size=10)
    g_pub_state = rospy.Publisher('/%s/state' % g_namespace, String, queue_size=10)

    g_sub_motors = rospy.Subscriber('/%s/motor_command' % g_namespace, Float32MultiArray, recv_motor_command)
    g_sub_ping = rospy.Subscriber('/%s/ping_command' % g_namespace, Empty, recv_ping)
    g_sub_odom = rospy.Subscriber('/%s/set_odometry' % g_namespace, Pose2D, set_odometry)
    g_sub_servo = rospy.Subscriber('/%s/set_servo' % g_namespace, Int16, set_servo)
    g_sub_render = rospy.Subscriber('/%s/render_sim' % g_namespace, Empty, recv_render)

    g_tk_window = tk.Tk()
    img = ImageTk.PhotoImage('RGB', (MAP_SIZE_X, MAP_SIZE_Y))
    g_tk_label = tk.Label(g_tk_window, image = img)
    g_tk_label.pack(fill="both", expand="yes")

    g_render_requested = True
    g_render_image_base = Image.new('RGB', (MAP_SIZE_X, MAP_SIZE_Y), color = 'white')
    for y_coord in range(0,MAP_SIZE_Y):
      for x_coord in range(0, MAP_SIZE_X):
        # Add line-following diagram as shades of black-to-red
        if g_world_surface[y_coord, x_coord] > 0:
          g_render_image_base.putpixel((x_coord, y_coord), (255-int(g_world_surface[y_coord, x_coord]/5),0,0))

        # Add objects as shades of black-to-green
        if g_world_obstacles[y_coord, x_coord] > 0:
          g_render_image_base.putpixel((x_coord, y_coord), (0,255-int(g_world_surface[y_coord, x_coord]/5),0))

def update_and_publish_state(pub):
  global g_pose, g_ir_sensors, g_us_sensor, g_world_starting_pose, g_world_obstacles, g_motors, g_ping_requested, g_servo_angle

  state = {}
  state['servo'] = math.degrees(-g_servo_angle) # Un-Negate theta
  state['light_sensors'] = get_ir_reading()

  if g_ping_requested is True:
    state['ping'] = get_ping_reading()
    g_ping_requested = False

  pub.publish(json.dumps(state))


def update_and_publish_odometry(pub, time_delta):
  global SPARKI_SPEED, SPARKI_AXLE_DIAMETER, MAP_SIZE_X, MAP_SIZE_Y
  global g_motors, g_pose
  left_wheel_dist = (g_motors[0] * time_delta * SPARKI_SPEED)
  right_wheel_dist = (g_motors[1] * time_delta * SPARKI_SPEED)

  g_pose.x += math.cos(g_pose.theta) * (left_wheel_dist+right_wheel_dist)/2.
  g_pose.y += math.sin(g_pose.theta) * (left_wheel_dist+right_wheel_dist)/2.
  g_pose.theta += (right_wheel_dist - left_wheel_dist) / SPARKI_AXLE_DIAMETER

  g_pose.x = min(MAP_SIZE_X*MAP_RESOLUTION, max(g_pose.x, 0))
  g_pose.y = min(MAP_SIZE_Y*MAP_RESOLUTION, max(g_pose.y, 0))

  pub_pose = copy.copy(g_pose)
  pub_pose.y = MAP_SIZE_Y * MAP_RESOLUTION - pub_pose.y
  pub_pose.theta = -pub_pose.theta # Negate theta
  pub.publish(pub_pose)

def render_robot_and_scene():
    global SPARKI_SIZE_RADIUS, g_tk_window, g_tk_label, g_render_image_base

    render_img = copy.copy(g_render_image_base)

    robot_pixel_coords = np.array( [int(g_pose.x / MAP_RESOLUTION), int(g_pose.y / MAP_RESOLUTION)] )

    # Add robot as blue
    robot_y_range = [int(robot_pixel_coords[1] - SPARKI_SIZE_RADIUS/MAP_RESOLUTION - 1), int(robot_pixel_coords[1] + SPARKI_SIZE_RADIUS/MAP_RESOLUTION + 1)]
    robot_x_range = [int(robot_pixel_coords[0] - SPARKI_SIZE_RADIUS/MAP_RESOLUTION - 1), int(robot_pixel_coords[0] + SPARKI_SIZE_RADIUS/MAP_RESOLUTION + 1)]
    for y_coord in range(robot_y_range[0], robot_y_range[1]):
      for x_coord in range(robot_x_range[0], robot_x_range[1]):
          if (np.linalg.norm(robot_pixel_coords - np.array((x_coord, y_coord))) < SPARKI_SIZE_RADIUS/MAP_RESOLUTION):
            render_img.putpixel((x_coord,y_coord), (0,0,128))

    # Add servo/ultrasonic direction as line extending halfway out sparki's radius
    sparki_pixel_radius = int(SPARKI_SIZE_RADIUS / MAP_RESOLUTION)
    for pct in range(0, 100):
      dist = pct/100. * sparki_pixel_radius / 2
      for width in range(-5,6,1):
          pixel_coord = [ int(robot_pixel_coords[0] + width*math.cos(g_pose.theta) + math.cos(g_pose.theta + g_servo_angle) * dist), int(robot_pixel_coords[1] + width*math.sin(g_pose.theta) + math.sin(g_pose.theta + g_servo_angle) * dist) ]
          render_img.putpixel( pixel_coord, (255, 255, 0) )

    # Add small indicator for front of robot
    front_of_robot_pixel_coord = [ int(robot_pixel_coords[0] + math.cos(g_pose.theta)* sparki_pixel_radius), int(robot_pixel_coords[1] + math.sin(g_pose.theta)*sparki_pixel_radius) ]
    for y_coord in range(front_of_robot_pixel_coord[1] - 3, front_of_robot_pixel_coord[1] + 3 + 1):
      for x_coord in range(front_of_robot_pixel_coord[0] - 3, front_of_robot_pixel_coord[0] + 3 + 1):
        if x_coord < 0 or y_coord < 0 or x_coord >= MAP_SIZE_X or y_coord >= MAP_SIZE_Y:
          continue
        render_img.putpixel( (x_coord, y_coord), (0, 255, 255) )

    # Display on screen
    img = ImageTk.PhotoImage(render_img) #render_img.resize((320,240), Image.ANTIALIAS))
    g_tk_label.configure(image=img)
    g_tk_label.image = img
    g_tk_window.update_idletasks()
    g_tk_window.update()


def launch_simulator(args):
    global g_pub_odom, g_pub_state, g_render_requested
    init(args)

    last_time = time.time()
    while not rospy.is_shutdown():
        cycle_start = time.time()
        # Update and Publish Odometry
        update_and_publish_odometry(g_pub_odom, time.time() - last_time)
        last_time = time.time()
        # Update and Publish Sensors
        update_and_publish_state(g_pub_state)
        if g_render_requested is True:
          render_start_time = time.time()
          render_robot_and_scene()
          render_end_time = time.time()
          cycle_start += render_end_time - render_start_time
          last_time += render_end_time - render_start_time
          g_render_requested = False
        if CYCLE_TIME-(time.time()-cycle_start) < 0:
          rospy.loginfo("CYCLE TIME over by: %f" % (CYCLE_TIME-(time.time()-cycle_start)))
        rospy.sleep(max(0,CYCLE_TIME-(time.time()-cycle_start)))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Sparki Simulation Environment")
    # 800, 656 for bottom line on default line follow map
    default_x, default_y = 800 * MAP_RESOLUTION, 656 * MAP_RESOLUTION
    parser.add_argument('-n','--namespace', type=str, nargs='?', default='sparki', help='Prepended string for all topics')
    parser.add_argument('-p','--startingpose', nargs=3, default=[default_x, default_y, 0.], help='Starting x, y, theta of Sparki in world coords')
    parser.add_argument('-w','--worldmap', nargs='?', type=str, default='line-follow.png', help='Black and white image showing the ground texture of the world')
    parser.add_argument('-o','--obstacles', nargs='?', type=str, default='obstacles.png', help='Black and white image showing the obstacle locations')
    args = parser.parse_args()

    launch_simulator(args)


# fast
# 1.2
# 1.2
# 1.2075016736984252
# 1.2150062084197997
# 1.222491431236267
# 1.229973292350769
# 1.2374610900878904
# 1.2449562907218932
# 1.252441906929016
# 1.2599137902259825
# 1.2674100637435912
# 1.274889850616455
# 1.2823749303817749
# 1.2898581147193908
# 1.2973525643348693
# 1.304836642742157
# 1.3123588800430297
# 1.3198492527008057
# 1.3273340821266175
# 1.334822452068329
# 1.3423040986061097
# 1.3498061656951905
# 1.3573034405708313
# 1.364794957637787
# 1.3722801804542542
# 1.3797752380371093
# 1.3872865319252015
# 1.3947653889656066
# 1.4022520780563354
# 1.4097269296646118
# 1.4172179102897644
# 1.424715256690979
# 1.4322029113769532
# 1.4396865606307985
# 1.4471811890602113
# 1.4546920180320742
# 1.4621889710426332
# 1.4696675062179567
# 1.4771594166755677
# 1.4846476435661318
# 1.492123031616211
# 1.4995986700057984
# 1.507081174850464
# 1.514547371864319
# 1.5220337390899659
# 1.52953999042511
# 1.5370263934135437
# 1.5444950938224793
# 1.5519875049591065
# 1.5594680786132813
# 1.5669530510902405
# 1.5669530510902405
# 1.5669530510902405
# 1.5739762829411632
# 1.5810023007499119
# 1.5810023007499119
# 1.5810023007499119
# 1.5866975009184117
# 1.592406837558053
# 1.5981015202064146
# 1.6037988176933689
# 1.6094977494544436
# 1.6151971170219503
# 1.6151971170219503
# 1.6199519840748735
# 1.6247239908283695
# 1.6247239908283695
# 1.6247239908283695
# 1.6271963918766705
# 1.6296720655866632
# 1.632166106517527
# 1.6346483136934333
# 1.6371309714531956
# 1.639616415718384
# 1.639616415718384
# 1.639616415718384
# 1.6394977192882243
# 1.6393796371231324
# 1.6392616373113003
# 1.6391431236376899
# 1.6390248412300827
# 1.638906790088479
# 1.6387887135640213
# 1.638669989494754
# 1.6385517268293668
# 1.6384331945416633
# 1.638315301337818
# 1.6381972987056688
# 1.638079457959722
# 1.6379614429181775
# 1.6378432767076356
# 1.6377252966380234
# 1.6376070559711096
# 1.6374891785610401
# 1.6373711477257198
# 1.6372529922323829
# 1.6372529922323829
# 1.6372529922323829
# 1.634559013734397
# 1.6318681114947058
# 1.6291811350239676
# 1.6264888555472992
# 1.6237982364778274
# 1.6237982364778274
# 1.6237982364778274
# 1.6188534798145213
# 1.6138843674205503
# 1.6089250823631036
# 1.6039799477253158
# 1.5990393015344984
# 1.5940984191096297
# 1.5891214873685684
# 1.5841802978394335
# 1.5792364861123316
# 1.5743037537622249
# 1.5693577450584482
# 1.564403869760771
# 1.5594621368933186
# 1.5545114507553308
# 1.5495651113238826
# 1.5446013141960606
# 1.5446013141960606
# 1.5387408490666257
# 1.5328533985773372
# 1.5270003159712198
#







#slow
# 1.2
# 1.2
# 1.2013899589061736
# 1.202777631139755
# 1.2041637126445768
# 1.205550092411041
# 1.2069362004280086
# 1.2083257947921748
# 1.2097146865844721
# 1.2111018617153162
# 1.2124875389099115
# 1.2138778292179102
# 1.2152643548011774
# 1.2166530808925624
# 1.2180423438549037
# 1.2194307186603541
# 1.2208159849166866
# 1.2222054400920863
# 1.2235940004825587
# 1.2249834689140315
# 1.226369093084335
# 1.2277598871231075
# 1.2291466579437251
# 1.2305407726287838
# 1.2319264962196346
# 1.2333203855514523
# 1.2347097943305967
# 1.2360957565307615
# 1.23748394575119
# 1.2388733876705167
# 1.2402570565700528
# 1.2416490834236142
# 1.2430365501880642
# 1.2444255016326902
# 1.2458121398925779
# 1.2471997657299039
# 1.2485900096416471
# 1.2499802933216093
# 1.251366494131088
# 1.2527553196430203
# 1.2541438004970547
# 1.2555385382175441
# 1.256927542686462
# 1.2583168520450587
# 1.2597033577442165
# 1.2610892868041987
# 1.2624765348434444
# 1.2638626362323757
# 1.2652505338191982
# 1.2666389086246486
# 1.2680230945110316
# 1.2694124834060663
# 1.2707994199275965
# 1.2721854285240168
# 1.273572749471664
# 1.2749587249279017
# 1.276345104694366
# 1.2777314712047572
# 1.2791184541225429
# 1.2805040451526637
# 1.2818971722602839
# 1.2832853349685664
# 1.284676566457748
# 1.2860662204742428
# 1.2874549266815183
# 1.2888411407470701
# 1.290229707765579
# 1.2916206940174102
# 1.293007106924057
# 1.2944006449699401
# 1.2957872766017913
# 1.2971743456840514
# 1.2985613484859466
# 1.299956417608261
# 1.3013483781814574
# 1.302735838317871
# 1.304123371362686
# 1.3055117395401
# 1.3069061524868009
# 1.308293181800842
# 1.3096787927150724
# 1.3110645361900326
# 1.312456695604324
# 1.3138450704097744
# 1.3152353143215176
# 1.3166257438182827
# 1.3180127267360684
# 1.3193984702110286
# 1.3207918690681453
# 1.322181801462173
# 1.3235717338562005
# 1.3249614210128777
# 1.3263500012874596
# 1.327742160701751
# 1.329129720258712
# 1.3305180818080895
# 1.3319095055103294
# 1.3333032954215995
# 1.3346925517559043
# 1.336084956407546
# 1.337474451351165
# 1.3388613945007315
# 1.3402517444610587
# 1.3416385749340047
# 1.343028818845748
# 1.3444149136066428
# 1.3458019893169395
# 1.3471881768703453
# 1.3485757894515984
# 1.3499626862049094
# 1.351351114034652
# 1.3527360886573783
# 1.3541234096050254
# 1.3555123013973227
# 1.356902479028701
# 1.358289693927764
# 1.3596807398319235
# 1.3610693399906149
# 1.3624582715511313
# 1.363845161676406
# 1.365233344268798
# 1.3666219179153434
# 1.368008662223815
# 1.3693954529285421
# 1.3707858095169059
# 1.3721699688911428
# 1.3735588474273672
# 1.3749422379493703
# 1.3763329458236684
# 1.3777177149772633
# 1.3791108023166645
# 1.3804980039596546
# 1.3818869951724995
# 1.3832738190174092
# 1.3846601059913624
# 1.3860469762325276
# 1.3874366302490224
# 1.3888203654289235
# 1.3902055985450734
# 1.3915927736759175
# 1.3929814136028278
# 1.3943717304229724
# 1.3957607348918901
# 1.3971516813755023
# 1.3985405002593982
# 1.3999356290340412
# 1.401326237487792
# 1.4027192718029011
# 1.4041112124919881
# 1.405506579875945
# 1.4068968039035785
# 1.4082828854084002
# 1.4096687614440906
# 1.4110555985450732
# 1.4124458557128894
# 1.4138339190006244
# 1.4152209880828845
# 1.4166102974414814
# 1.4179990765571582
# 1.4193884720802294
# 1.4207759123325334
# 1.4221636044502244
# 1.4235489833831774
# 1.4249385379791246
# 1.426327960014342
# 1.4277145783901202
# 1.4291014154911028
# 1.430490983343123
# 1.4318765080928788
# 1.4332644587039933
# 1.4346476768970475
# 1.436036489152907
# 1.4374283039093003
# 1.4388143522739396
# 1.4402001885414109
# 1.4415889809131608
# 1.4429798876285538
# 1.4443712450504287
# 1.445763417720793
# 1.447153396511076
# 1.448538033103941
# 1.4499315380096418
# 1.4513175267219527
# 1.452706007575987
# 1.4541002415657025
# 1.455495231151579
# 1.456882883501051
# 1.4582694289684277
# 1.4596557822227458
# 1.461041910123823
# 1.4624356735229471
# 1.4638233391284923
# 1.4652086053848248
# 1.4665975303173047
# 1.4679837311267834
# 1.4693719534873944
# 1.4707507375240307
# 1.4721380452156048
# 1.4735244846343976
# 1.47491224966049
# 1.4763014794826488
# 1.4776955345153788
# 1.4790883832454662
# 1.4804850563526135
# 1.4818750285148603
# 1.483262037944792
# 1.4846543431758865
# 1.486046343517302
# 1.4874404714584335
# 1.4888264071464523
# 1.4902131912231429
# 1.4916054964542373
# 1.492995534896849
# 1.4943875882625566
# 1.4957745579242692
# 1.4971607322216018
# 1.4985548734188063
# 1.4999518646717056
# 1.501343599891661
# 1.5027297277927383
# 1.5041172807216627
# 1.5055009893894178
# 1.5068897950172406
# 1.5082712302684766
# 1.5096579613208754
# 1.5110436981677993
# 1.5124318940162642
# 1.5138202423095686
# 1.5152057140350323
# 1.5165907947063428
# 1.5179805680274945
# 1.5193695857524854
# 1.5207630840301496
# 1.522154872274397
# 1.5235432603359205
# 1.5249320394515973
# 1.5263236553668957
# 1.52771346182823
# 1.5291026319980603
# 1.5304929024219494
# 1.5318737941741924
# 1.533264574956892
# 1.5346456986904127
# 1.53602457551956
# 1.5374140439510329
# 1.5387986805438978
# 1.5401828067779524
# 1.5415721559047681
# 1.5429661512851698
# 1.5443540091037733
# 1.545736982059477
# 1.5471156865596754
# 1.5485055659294111
# 1.5498944577217084
# 1.551283163928984
# 1.5526720026969894
# 1.5540583426952346
# 1.5554457034111007
# 1.5568298495292647
# 1.55821829061508
# 1.559609197330473
# 1.559609197330473
# 1.559609197330473
# 1.559609197330473
# 1.5609878275368385
# 1.5623675329479831
# 1.5637508399653206
# 1.5651297142365745
# 1.5651297142365745
# 1.5651297142365745
# 1.5665000698082923
# 1.5678691108494762
# 1.5678691108494762
# 1.5678691108494762
# 1.5678691108494762
# 1.5678691108494762
# 1.569194954566789
# 1.5705216546411618
# 1.5718509174433293
# 1.5731774969933752
# 1.5745084661662772
# 1.5745084661662772
# 1.5745084661662772
# 1.5758063912202487
# 1.577106562248663
# 1.577106562248663
# 1.577106562248663
# 1.5783722820357777
# 1.5796355319077569
# 1.5808985824956296
# 1.582162031651715
# 1.582162031651715
# 1.582162031651715
# 1.5833858959193041
# 1.584612625338829
# 1.5858383841150452
# 1.5870645054206898
# 1.5870645054206898
# 1.5882637578567598
# 1.5894710033184698
# 1.5906726559590783
# 1.5918760542029875
# 1.5918760542029875
# 1.5918760542029875
# 1.5930306977845725
# 1.5941858706120255
# 1.5953414569128128
# 1.5964933881093244
# 1.5964933881093244
# 1.5964933881093244
# 1.597600199180116
# 1.5986998405180552
# 1.5998005812501002
# 1.600902416115992
# 1.600902416115992
# 1.600902416115992
# 1.6019444959612978
# 1.6019444959612978
# 1.6029562905152133
# 1.6039689369762122
# 1.6049822852924789
# 1.6059957836605614
# 1.6059957836605614
# 1.6059957836605614
# 1.6069442562325895
# 1.6078926653161356
# 1.60884557301212
# 1.609792621628195
# 1.6107438060653825
# 1.611692559800688
# 1.611692559800688
# 1.611692559800688
# 1.6125745029783591
# 1.6134556546243013
# 1.6143370252045515
# 1.6152199577968838
# 1.6152199577968838
# 1.6160656395931778
# 1.616913487296921
# 1.616913487296921
# 1.616913487296921
# 1.6176841167935532
# 1.6184576973530755
# 1.6192292822563183
# 1.6200020549623744
# 1.6200020549623744
# 1.6200020549623744
# 1.6206971763799691
# 1.6213953596114623
# 1.6220916433228363
# 1.6227868676864514
# 1.623481564036607
# 1.624176193969975
# 1.624176193969975
# 1.624176193969975
# 1.6247938215590265
# 1.6254083403090258
# 1.6260226298629512
# 1.62663756292893
# 1.62725196120407
# 1.6278669412846278
# 1.6284835521834031
# 1.629100662612083
# 1.6297155780475945
# 1.6303311281799258
# 1.6303311281799258
# 1.6303311281799258
# 1.6308660206839085
# 1.6314017224742947
# 1.6319307947329802
# 1.632463282282085
# 1.632997306966371
# 1.633529277895539
# 1.6340609332539082
# 1.6345943140719676
# 1.6351266336559698
# 1.6351266336559698
# 1.6351266336559698
# 1.6355738577673178
# 1.6360246153268885
# 1.63647346598012
# 1.63692152904465
# 1.637369968782037
# 1.6378177450615508
# 1.6378177450615508
# 1.6378177450615508
# 1.6381798397652048
# 1.6385408377198625
# 1.6389016025074108
# 1.6392636592134617
# 1.6396257729159172
# 1.6399875152781613
# 1.6403481522555903
# 1.6407095906370102
# 1.6410714383562444
# 1.6414333655250122
# 1.6417957106674133
# 1.6421583839709313
# 1.6425205978488855
# 1.6428837875743718
# 1.643245018696139
# 1.643245018696139
# 1.643245018696139
# 1.6435181806325512
# 1.6437912916495176
# 1.6440644065833645
# 1.644339329810349
# 1.6446128573222691
# 1.6448876943778838
# 1.645161285865518
# 1.6454344974157493
# 1.6457080118714014
# 1.6459816608066153
# 1.6462551909297891
# 1.6465285904902816
# 1.6468025540815576
# 1.6470749770331932
# 1.6473481729159025
# 1.6476216625646452
# 1.647894546402546
# 1.648168301093532
# 1.6484425545339607
# 1.648716460677657
# 1.6489896500322323
# 1.6492628746387317
# 1.6495365810215254
# 1.6498099518582279
# 1.6498099518582279
# 1.6498099518582279
# 1.649994814434202
# 1.6501800699457794
# 1.650363942710816
# 1.6505474858318905
# 1.6507310192834086
# 1.6509154124463805
# 1.6510998029722006
# 1.6512836994716025
# 1.6514681691119735
# 1.6516533481461515
# 1.651837989201383
# 1.6520219982192574
# 1.6522058657099907
# 1.6522058657099907
# 1.6522058657099907
# 1.6522998540931004
# 1.6523937028742186
# 1.6524872818476417
# 1.6525810026602676
# 1.6526751673356357
# 1.6527688724877818
# 1.6528624286416485
# 1.6529561078421422
# 1.6530498116519614
# 1.6531434004691146
# 1.6532372653581549
# 1.6533312103393634
# 1.653425026457195
# 1.6535187222130532
# 1.6536125190908668
# 1.6537061204364039
# 1.6537061204364039
# 1.6537061204364039
# 1.6537092440352748
# 1.6537123695763107
# 1.653715491935184
# 1.653718620688262
# 1.6537217460799005
# 1.6537248705004566
# 1.6537279919330665
# 1.6537311138736273
# 1.6537342382941833
# 1.6537373620275118
# 1.6537373620275118
# 1.6536950102912396
# 1.65365272370683
# 1.6536105817474527
# 1.6535683030296766
# 1.6535683030296766
# 1.6535683030296766
# 1.6534352152809493
# 1.6533019885222684
# 1.653169454274361
# 1.6530360199528726
# 1.6529022707960557
# 1.6527689691370344
# 1.6526352339446877
# 1.6525018910270044
# 1.6525018910270044
# 1.6525018910270044
# 1.6522790496265025
# 1.6520560196518297
# 1.6518328256069743
# 1.6516102153962753
# 1.6513869457086734
# 1.6511637633831167
# 1.6509408676476842
# 1.6507170641992934
# 1.6507170641992934
# 1.6507170641992934
# 1.650405043168667
# 1.6500917304925404
# 1.6497795560138326
# 1.6494682709380817
# 1.6491570722699882
# 1.6488460374784866
# 1.6485340253865834
# 1.6482232915320956
# 1.6479115624997598
# 1.6475998438959345
# 1.6472871316040485
# 1.6469765154427494
# 1.6466641115368132
# 1.6463524361368167
# 1.6460401871687491
# 1.6460401871687491
# 1.6460401871687491
# 1.6456405654278734
# 1.6452411593552074
# 1.6448418372595432
# 1.6444421697130303
# 1.6440427102004538
# 1.6436416112277712
# 1.6436416112277712
# 1.6436416112277712
# 1.6431558044578871
# 1.6426688706875345
# 1.6421840541011896
# 1.6416977209339674
# 1.6412111303654036
# 1.6407249525665586
# 1.6407249525665586
# 1.6401960718989226
# 1.6396680663020549
# 1.6391405852432845
# 1.6391405852432845
# 1.6391405852432845
# 1.638529360396051
# 1.637915434937077
# 1.6373054495714738
# 1.63669558669582
# 1.636083449006821
# 1.6354718829375858
# 1.634861022643773
# 1.6342499523671898
# 1.6342499523671898
# 1.6342499523671898
# 1.633557505711819
# 1.6328661476534483
# 1.632173334833632
# 1.631483223713643
# 1.630792647465845
# 1.6301010617917377
# 1.6294089021301215
# 1.6287187646199022
# 1.6280277793235345
# 1.6273387535017667
# 1.6266486291866626
# 1.6266486291866626
# 1.6266486291866626
# 1.625880948337287
# 1.62511677634553
# 1.6243500414577694
# 1.6235825072691092
# 1.622812674173733
# 1.622043361723897
# 1.62127611719015
# 1.620508410675149
# 1.6197404365043422
# 1.6189735916210448
# 1.6189735916210448
# 1.6181705946790965
# 1.6173641222256407
# 1.616559533622449
# 1.6157538608442072
# 1.6149490069641417
# 1.6141423153689205
# 1.6141423153689205
# 1.6141423153689205
# 1.613266629395824
# 1.6123892091837222
# 1.611519743850536
# 1.6106421518901268
# 1.6097665371297432
# 1.6088894729812053
# 1.6080145787259157
# 1.607136115457021
# 1.6062621218330988
# 1.605383658564204
# 1.604506749408041
# 1.6036282651942306
# 1.6027497516575386
# 1.6018737934005403
# 1.6018737934005403
# 1.6018737934005403
# 1.6009237768659177
# 1.5999750926577878
# 1.5990276368998477
# 1.5980811295777158
# 1.5971355481095872
# 1.5961879929917053
# 1.5952431341413353
# 1.5942925259634229
# 1.5933439727297616
# 1.5923979034946458
# 1.5914533969204332
# 1.590508687109976
# 1.590508687109976
# 1.590508687109976
# 1.589497488111423
