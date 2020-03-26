'''
 IMPORTANT: Read through the code before beginning implementation!
 Your solution should fill in the various "TODO" items within this starter code.
'''
import rospy
import time
import copy
import math
import random
import argparse
import numpy as np
from PIL import Image
import numpy as np
from pprint import pprint
from math import floor, ceil
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16
import matplotlib.pyplot as plt

g_CYCLE_TIME = .100

# Parameters you might need to use which will be set automatically
MAP_SIZE_X = None
MAP_SIZE_Y = None
MAP_X_CELLS = None
MAP_Y_CELLS = None
# Default parameters will create a 4x4 grid to test with
g_MAP_SIZE_X = 2. # 2m wide
g_MAP_SIZE_Y = 1.5 # 1.5m tall
g_MAP_RESOLUTION_X = 0.2 #0.5 # Each col represents 50cm
g_MAP_RESOLUTION_Y = 0.1 #0.375 # Each row represents 37.5cm
g_NUM_X_CELLS = int(g_MAP_SIZE_X // g_MAP_RESOLUTION_X) # Number of columns in the grid map
g_NUM_Y_CELLS = int(g_MAP_SIZE_Y // g_MAP_RESOLUTION_Y) # Number of rows in the grid map
COORD_ADJ = 0.0015

# Map from Lab 4: values of 0 indicate free space, 1 indicates occupied space
g_WORLD_MAP = [0] * g_NUM_Y_CELLS*g_NUM_X_CELLS # Initialize graph (grid) as array

# Source and Destination (I,J) grid coordinates
g_dest_coordinates = (3,3)
g_src_coordinates = (0,0)

#GLOBALS
render_buffer = 0
pose2D_sparki_odometry = None

#Constants
CYCLE_TIME = 0.1 # In seconds
RENDER_LIMIT = 1 # in seconds

#Publishers
publisher_motor = None
publisher_odom = None
publisher_render = None
subscriber_odometry = None


def create_test_map(map_array):
  # Takes an array representing a map of the world, copies it, and adds simulated obstacles
  num_cells = len(map_array)
  # accomodate for 2d map storage
  num_cells *= num_cells

  new_map = copy.copy(map_array)
  # Add obstacles to up to sqrt(n) vertices of the map
  for i in range(int(math.sqrt(len(map_array)))):
    random_cell = random.randint(0, num_cells-1)
    i,j = vertex_index_to_ij(random_cell)
    try:
        new_map[i][j] = 1
    except:
        print(random_cell)
        raise Exception(random_cell)

  return new_map

def _load_img_to_intensity_matrix(img_filename):
  '''
  Helper function to read the world image containing obstacles
  YOu should not modify this
  '''
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

def vertex_index_to_ij(vertex_index):
  '''
  vertex_index: unique ID of graph vertex to be convered into grid coordinates
  Returns COL, ROW coordinates in 2D grid
  '''
  global g_NUM_X_CELLS
  return vertex_index % g_NUM_X_CELLS, vertex_index // g_NUM_X_CELLS

def ij_to_vertex_index(i,j):
  '''
  i: Column of grid map
  j: Row of grid map

  returns integer 'vertex index'
  '''
  global g_NUM_X_CELLS
  return j*g_NUM_X_CELLS + i

def ij_coordinates_to_xy_coordinates(i,j):
  '''
  i: Column of grid map
  j: Row of grid map

  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return (i+0.5)*g_MAP_RESOLUTION_X, (j+0.5)*g_MAP_RESOLUTION_Y

def xy_coordinates_to_ij_coordinates(x,y):
  '''
  i: Column of grid map
  j: Row of grid map

  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return int(x // g_MAP_RESOLUTION_X), int(y // g_MAP_RESOLUTION_Y)

# **********************************
# *      Core Dijkstra Functions   *
# **********************************

def get_travel_cost(vertex_source, vertex_dest):
  # Returns the cost of moving from vertex_source (int) to vertex_dest (int)
  # INSTRUCTIONS:
  '''
      This function should return 1 if:
        vertex_source and vertex_dest are neighbors in a 4-connected grid (i.e., N,E,S,W of each other but not diagonal) and neither is occupied in g_WORLD_MAP (i.e., g_WORLD_MAP isn't 1 for either)

      This function should return 1000 if:
        vertex_source corresponds to (i,j) coordinates outside the map
        vertex_dest corresponds to (i,j) coordinates outside the map
        vertex_source and vertex_dest are not adjacent to each other (i.e., more than 1 move away from each other)
  '''
  global g_WORLD_MAP, g_NUM_X_CELLS, g_NUM_Y_CELLS

  i0, j0 = vertex_index_to_ij(vertex_source)
  i1, j1 = vertex_index_to_ij(vertex_dest)
  try:
      if g_WORLD_MAP[j1][i1] == 1:
          return 10000#np.inf
      elif abs(i0 - i1) == 1 and abs(j0 - j1) == 1:
          return 10000#np.inf
      elif i0 < 0 or i0 >= g_NUM_X_CELLS or j0 < 0 or j0 > g_NUM_Y_CELLS:
          return 10000#np.inf
      elif i1 < 0 or i1 >= g_NUM_X_CELLS or j1 < 0 or j1 > g_NUM_Y_CELLS:
          return 10000#np.inf
      else:
          return 1
  except:
      print(vertex_source, vertex_dest)
      print(i0, j0)
      print(i1, j1)
      print(g_WORLD_MAP.shape)
      raise Exception('oof')

def run_dijkstra(source_vertex):
  '''
  source_vertex: vertex index to find all paths back to
  returns: 'prev' array from a completed Dijkstra's algorithm run

  Function to return an array of ints corresponding to the 'prev' variable in Dijkstra's algorithm
  The 'prev' array stores the next vertex on the best path back to source_vertex.
  Thus, the returned array prev can be treated as a lookup table:  prev[vertex_index] = next vertex index on the path back to source_vertex
  '''
  global g_NUM_X_CELLS, g_NUM_Y_CELLS, g_WORLD_MAP

  # Array mapping vertex_index to distance of shortest path from vertex_index to source_vertex.
  #dist = [0] * g_NUM_X_CELLS * g_NUM_Y_CELLS

  # Queue for identifying which vertices are up to still be explored:
  # Will contain tuples of (vertex_index, cost), sorted such that the min cost is first to be extracted (explore cheapest/most promising vertices first)
  #src = ij_to_vertex_index(source_vertex)
  #Q_cost = [src]  #[np.inf] * g_NUM_X_CELLS * g_NUM_Y_CELLS

  # Array of ints for storing the next step (vertex_index) on the shortest path back to source_vertex for each vertex in the graph
  #prev = [-1] * g_NUM_X_CELLS * g_NUM_Y_CELLS
  maxNode = ij_to_vertex_index(g_NUM_X_CELLS - 1, g_NUM_Y_CELLS - 1)

  nodeRange = list(range(0,maxNode+1))

  nodeInfo = {
      n: {
            'known' : False,
            'cost' : np.inf,
            'path' : -1,
            #'obstacle': False
         } for n in nodeRange
  }

  nodeInfo[source_vertex] = {
    'known' : True,
    'cost' : 0,
    'path' : -1,
    #'obstacle': False
  }

  costs = np.array([np.inf] * len(nodeRange))
  costs[source_vertex] = 0

  # Insert your Dijkstra's code here. Don't forget to initialize Q_cost properly!
  while np.sum(nodeInfo[n]['known'] for n in nodeRange) != len(nodeRange):
      # do things
      minCostIndex = np.where(costs == min(costs))[0][0]
      #curNode = nodeInfo[minCostIndex]

      curI, curJ = vertex_index_to_ij(minCostIndex)

      neighbors = []

      lowI = max(0, curI - 1)
      highI = min(g_NUM_X_CELLS - 1, curI + 1)

      lowJ = max(0, curJ - 1)
      highJ = min(g_NUM_Y_CELLS - 1, curJ + 1)

      for i in range(lowI, highI + 1):
          for j in range(lowJ, highJ + 1):
              index = ij_to_vertex_index(i,j)
              if index != minCostIndex:
                  neighbors.append(index)

      for neighbor in neighbors:
          if not nodeInfo[neighbor]['known']:
              c = get_travel_cost(minCostIndex, neighbor)

              #if c > 10000:
              #nodeInfo[neighbor]['obstacle'] = True

              c += nodeInfo[minCostIndex]['cost']

              if costs[neighbor] > c:
                  costs[neighbor] = c
                  nodeInfo[neighbor]['cost'] = c
                  nodeInfo[neighbor]['path'] = minCostIndex

      costs[minCostIndex] = np.inf
      nodeInfo[minCostIndex]['known'] = True

  # Return results of algorithm run
  return nodeInfo

def reconstruct_path(prev, source_vertex, dest_vertex):
  '''
  Given a populated 'prev' array, a source vertex_index, and destination vertex_index,
  allocate and return an integer array populated with the path from source to destination.
  The first entry of your path should be source_vertex and the last entry should be the dest_vertex.
  If there is no path between source_vertex and dest_vertex, as indicated by hitting a '-1' on the
  path from dest to source, return an empty list.
  '''
  global g_NUM_X_CELLS, g_NUM_Y_CELLS

  nodeInfo = prev
  maxCost = g_NUM_X_CELLS * g_NUM_Y_CELLS

  if nodeInfo[dest_vertex]['cost'] > maxCost:
      return []

  final_path = [dest_vertex]

  # TODO: Insert your code here
  parent = nodeInfo[dest_vertex]['path']
  while parent != -1:

      #if nodeInfo[parent]['obstacle']:
        #  return []

      final_path.insert(0, parent)
      parent = nodeInfo[parent]['path']

  return final_path

def render_map(map_array):
  '''
  TODO-
    Display the map in the following format:
    Use " . " for free grid cells
    Use "[ ]" for occupied grid cells

    Example:
    For g_WORLD_MAP = [0, 0, 1, 0,
                       0, 1, 1, 0,
                       0, 0, 0, 0,
                       0, 0, 0, 0]
    There are obstacles at (I,J) coordinates: [ (2,0), (1,1), (2,1) ]
    badThe map should render as:
      .  .  .  .
      .  .  .  .
      . [ ][ ] .
      .  . [ ] .


    Make sure to display your map so that I,J coordinate (0,0) is in the bottom left.
    (To do this, you'll probably want to iterate from row 'J-1' to '0')
  '''
  mapViz = []
  for r in map_array:
      rowStr = ''
      for p in r:
          if p == 0:
              v = ' . '
          elif p == 1:
              v = '[ ]'
          elif p == 2:
              v = ' x '
          #v = ' . ' if p == 0 else '[ ]'
          rowStr += v
      mapViz.insert(0, rowStr)

  for r in mapViz:
      print(r)

def part_1():
  global g_WORLD_MAP, g_NUM_X_CELLS, g_NUM_Y_CELLS

  g_NUM_X_CELLS = 4
  g_NUM_Y_CELLS = 4

  s = 2
  d = 13

  # TODO: Initialize a grid map to use for your test -- you may use create_test_map for this, or manually set one up with obstacles
  g_WORLD_MAP = []
  for i in range(4):
      g_WORLD_MAP.append([0] * 4)

  g_WORLD_MAP = create_test_map(g_WORLD_MAP)

  info = run_dijkstra(s)
  path = reconstruct_path(info,s,d)

  pathStr = ''
  first = True

  for p in path:
      i,j = vertex_index_to_ij(p)
      g_WORLD_MAP[j][i] = 2 if not g_WORLD_MAP[j][i] else 1
      if not first:
          pathStr += '-> {} '.format(p)
      else:
          pathStr += '{} '.format(p)
          first = False


  i,j = vertex_index_to_ij(s)
  #g_WORLD_MAP[i][j] = -1
  i,j = vertex_index_to_ij(d)
  #g_WORLD_MAP[i][j] = -1
  pprint(info)
  print(pathStr)
  render_map(g_WORLD_MAP)
  # Use render_map to render your initialized obstacle map

  # TODO: Find a path from the (I,J) coordinate pair in g_src_coordinates to the one in g_dest_coordinates using run_dijkstra and reconstruct_path

  '''
  TODO-
    Display the final path in the following format:
    Source: (0,0)
    Goal: (3,1)
    0 -> 1 -> 2 -> 6 -> 7
  '''

def part_2(args):
  global g_dest_coordinates
  global g_src_coordinates
  global g_WORLD_MAP, g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y, g_MAP_SIZE_X, g_MAP_SIZE_Y
  global g_NUM_X_CELLS, g_NUM_Y_CELLS

  g_src_coordinates = (float(args.src_coordinates[0]), float(args.src_coordinates[1]))
  g_dest_coordinates = (float(args.dest_coordinates[0]), float(args.dest_coordinates[1]))

  # pixel_grid has intensity values for all the pixels
  # You will have to convert it to the earlier 0 and 1 matrix yourself
  pixel_grid = _load_img_to_intensity_matrix(args.obstacles)

  '''
  TODO -
  1) Compute the g_WORLD_MAP -- depending on the resolution, you need to decide if your cell is an obstacle cell or a free cell.
  2) Run Dijkstra's to get the plan
  3) Show your plan/path on the image
  Feel free to add more helper functions
  '''

  #### Your code goes here ####
  mean_i = np.mean(pixel_grid)

  MAP_X = int(1.8 * 1000)#int(MAP_SIZE_X)
  RES_X = int(g_MAP_RESOLUTION_X*1000)

  MAP_Y = int(1.2 * 1000)
  RES_Y = int(g_MAP_RESOLUTION_Y*1000)

  NUM_X = MAP_X // RES_X
  NUM_Y = MAP_Y // RES_Y

  g_NUM_X_CELLS = NUM_X
  g_NUM_Y_CELLS = NUM_Y
  g_MAP_RESOLUTION_X = RES_X
  g_MAP_RESOLUTION_Y = RES_Y

  g_WORLD_MAP = np.zeros((NUM_Y, NUM_X))

  xos = []
  yos = []

  xs = []
  ys = []

  check = {}
  obstacle = {}
  count = {}
  vertices = {}

  for r in range(MAP_Y):
      for c in range(MAP_X):
          a = xy_coordinates_to_ij_coordinates(c,r)

          if a not in check:
              v = ij_to_vertex_index(a[0], a[1])
              vertices[v] = a
              check[a] = True
              count[a] = 1
              obstacle[a] = False
          else:
              count[a] += 1
          if r < len(pixel_grid) and c < len(pixel_grid[0]):
              if pixel_grid[r][c] > mean_i:
                  obstacle[a] = True
                  xos.append(c)
                  yos.append(r)
              else:
                  xs.append(c)
                  ys.append(r)


  for a in obstacle:
      c,r = a
      x,y = ij_coordinates_to_xy_coordinates(a[0], a[1])

      if obstacle[a]:
          v = 1
      else:
          v = 0

      g_WORLD_MAP[r][c] = v


  src_x, src_y = g_src_coordinates[0]*1000, g_src_coordinates[1]*1000
  dest_x, dest_y = g_dest_coordinates[0]*1000, g_dest_coordinates[1]*1000

  i,j = xy_coordinates_to_ij_coordinates(src_x, src_y)
  src = ij_to_vertex_index(i,j)

  i,j = vertex_index_to_ij(src)
  x,y = ij_coordinates_to_xy_coordinates(i,j)

  i,j = xy_coordinates_to_ij_coordinates(dest_x, dest_y)
  dest = ij_to_vertex_index(i,j)

  nodeInfo = run_dijkstra(src)
  path = reconstruct_path(nodeInfo, src, dest)

  print(path)

  for p in path:
      i,j = vertex_index_to_ij(p)
      x,y = ij_coordinates_to_xy_coordinates(i,j)
      plt.scatter([x], [y], color='blue')

  im = plt.imread(args.obstacles)
  plt.imshow(im)
  plt.scatter([src_x], [src_y], color='green')
  plt.scatter([dest_x], [dest_y], color = 'red')

  plt.show()

def loadWorld(args):
    global g_dest_coordinates
    global g_src_coordinates
    global g_WORLD_MAP, g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y, g_MAP_SIZE_X, g_MAP_SIZE_Y
    global g_NUM_X_CELLS, g_NUM_Y_CELLS

    fn = args.obstacles
    g_src_coordinates = (float(args.src_coordinates[0]), float(args.src_coordinates[1]))
    g_dest_coordinates = (float(args.dest_coordinates[0]), float(args.dest_coordinates[1]))

    pixel_grid = _load_img_to_intensity_matrix(fn)

    mean_i = np.mean(pixel_grid)

    print(np.shape(pixel_grid))

    MAP_X = int(1.2 * 1000)
    RES_X = int(g_MAP_RESOLUTION_X*1000)

    MAP_Y = int(0.8 * 1000)
    RES_Y = int(g_MAP_RESOLUTION_Y*1000)

    NUM_X = MAP_X // RES_X
    NUM_Y = MAP_Y // RES_Y

    g_NUM_X_CELLS = NUM_X
    g_NUM_Y_CELLS = NUM_Y
    g_MAP_RESOLUTION_X = RES_X
    g_MAP_RESOLUTION_Y = RES_Y

    g_WORLD_MAP = np.zeros((NUM_Y, NUM_X))

    check = {}
    obstacle = {}

    for r in range(MAP_Y):
        for c in range(MAP_X):
            a = xy_coordinates_to_ij_coordinates(c,r)

            if a not in check:
                v = ij_to_vertex_index(a[0], a[1])
                check[a] = True
                obstacle[a] = False

            if r < len(pixel_grid) and c < len(pixel_grid[0]):
                if pixel_grid[r][c] > mean_i:
                    obstacle[a] = True


    for a in obstacle:
        c,r = a
        x,y = ij_coordinates_to_xy_coordinates(a[0], a[1])

        if obstacle[a]:
            v = 1
        else:
            v = 0

        g_WORLD_MAP[r][c] = v


def getErrors(dest_x, dest_y):
    global pose2D_sparki_odometry

    pose_theta = pose2D_sparki_odometry.theta

    src_x = pose2D_sparki_odometry.x
    src_y = pose2D_sparki_odometry.y

    adj_x = float(dest_x - src_x)
    adj_y = float(dest_y - src_y)

    #diff = float(adj_y / adj_x)

    errorsDict = {
        'b': np.arctan2(adj_y, adj_x) - pose_theta,
        'd': np.sqrt((src_x - dest_x)**2 + (src_y - dest_y)**2)
    }

    return errorsDict

def getDestCoords(d):
    i,j = vertex_index_to_ij(d)
    x,y = ij_coordinates_to_xy_coordinates(i,j)

    x = x*COORD_ADJ
    y = y*COORD_ADJ

    return x,y

def loop(args):
      global publisher_motor, publisher_odom, publisher_render
      global subscriber_odometry
      global CYCLE_TIME, RENDER_LIMIT
      global pose2D_sparki_odometry
      global render_buffer
      global g_WORLD_MAP, g_src_coordinates, g_dest_coordinates, COORD_ADJ

      FACE = 0
      APPROACH = 1
      UPDATE_GOAL = 2

      state = FACE

      init(args)

      # Djikstra code
      #########################################

      src_x, src_y = g_src_coordinates[0]/COORD_ADJ, g_src_coordinates[1]/COORD_ADJ
      dest_x, dest_y = g_dest_coordinates[0]/COORD_ADJ, g_dest_coordinates[1]/COORD_ADJ

      print(src_x, src_y)

      i,j = xy_coordinates_to_ij_coordinates(src_x, src_y)
      src = ij_to_vertex_index(i,j)

      i,j = xy_coordinates_to_ij_coordinates(dest_x, dest_y)
      dest = ij_to_vertex_index(i,j)

      nodeInfo = run_dijkstra(src)
      path = reconstruct_path(nodeInfo, src, dest)

      for p in path:
          i,j = vertex_index_to_ij(p)
          x,y = ij_coordinates_to_xy_coordinates(i,j)
          plt.scatter([x], [800 - y], color='blue')

      im = plt.imread('empty_world.png')
      print(im.shape)
      print('^')
      im = plt.imread(args.obstacles)
      plt.imshow(im)
      plt.scatter([src_x], [800 - src_y], color='green')
      plt.scatter([dest_x], [800 - dest_y], color = 'red')
      plt.show()

      ##########################################

      #dest_x_m, dest_y_m = g_dest_coordinates
      d = path.pop(0)
      dest_x_m, dest_y_m = getDestCoords(d)

      #print(dest_x_m, dest_y_m)

      while not rospy.is_shutdown():
          start_time = time.time()

          # loop functionality

          kinematicErrors = getErrors(dest_x_m, dest_y_m)

          if state == FACE:
              if abs(kinematicErrors['b']) >= np.deg2rad(1):

                  motor_right = 0.5 if kinematicErrors['b'] < 0 else -0.5
                  motor_left = 0.5 if kinematicErrors['b'] > 0 else -0.5

                  motor_msg = Float32MultiArray()
                  motor_msg.data = [float(motor_left), float(motor_right)]
                  try:
                      publisher_motor.publish(motor_msg)
                  except:
                      raise Exception(motor_left, motor_right)
              else:
                  state = APPROACH

          elif state == APPROACH:
              #while True:
                  #pass
              if abs(kinematicErrors['d']) >= 0.02: # m or cm???
                  # distance error gets fixed
                  #print(kinematicErrors['d'])

                  motor_right = 0.5
                  motor_left = 0.5

                  motor_msg = Float32MultiArray()
                  motor_msg.data = [float(motor_left), float(motor_right)]
                  try:
                      publisher_motor.publish(motor_msg)
                  except:
                      raise Exception(motor_left, motor_right)
              else:
                  state = UPDATE_GOAL

          elif state == UPDATE_GOAL:
              if path:
                  print('Waypoint reached.')
                  d = path.pop(0)
                  dest_x_m, dest_y_m = getDestCoords(d)
                  state = FACE
              else:
                  state = -1
          else:
             # its arrived
             while True:
                 pass

             print('Sparki has arrived!')

          if render_buffer >= RENDER_LIMIT:
              render_buffer = 0
              publisher_render.publish(Empty())

          render_buffer += CYCLE_TIME
          rospy.sleep(max(CYCLE_TIME - (start_time - time.time()), 0))

def init(args):
    global publisher_motor, publisher_odom, publisher_render
    global subscriber_odometry
    global pose2D_sparki_odometry
    global render_buffer
    global g_src_coordinates, g_dest_coordinates

    loadWorld(args)

    render_buffer = 0

    publisher_motor = rospy.Publisher('/sparki/motor_command', Float32MultiArray, queue_size=10)
    publisher_odom = rospy.Publisher('/sparki/set_odometry', Pose2D, queue_size=10)
    publisher_render = rospy.Publisher('/sparki/render_sim', Empty, queue_size=10)
    subscriber_odometry = rospy.Subscriber('/sparki/odometry', Pose2D, callback=callback_update_odometry)

    # init the node
    rospy.init_node('Lab6')

    #TODO: Set up your initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    pose2D_sparki_odometry = Pose2D()
    pose2D_sparki_odometry = Pose2D(g_src_coordinates[0], g_src_coordinates[1], 0)

    #pose2D_sparki_odometry = Pose2D(1.2 / COORD_ADJ, 0.8 / COORD_ADJ, 0)
    publisher_odom.publish(pose2D_sparki_odometry)

def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2D_sparki_odometry
    #TODO: Copy this data into your local odometry variable
    pose2D_sparki_odometry = copy.copy(data)

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Dijkstra on image file")
  parser.add_argument('-s','--src_coordinates', nargs=2, default=[1.2, 0.2], help='Starting x, y location in world coords')
  parser.add_argument('-g','--dest_coordinates', nargs=2, default=[0.3, 0.7], help='Goal x, y location in world coords')
  parser.add_argument('-o','--obstacles', nargs='?', type=str, default='obstacles_test1.png', help='Black and white image showing the obstacle locations')
  args = parser.parse_args()

  loop(args)
raise Exception

  #part_1()
  #part_2(args)
