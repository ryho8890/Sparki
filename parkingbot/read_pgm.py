import re
import numpy
import math
from matplotlib import pyplot

'''
def read_pgm(filename, byteorder='>'):
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return numpy.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))
'''

def foundInWaypoints(waypoints, point):
    for i in waypoints:
        if i == point:
            return 1
    return 0

def waypoints(map):
    nodes = []
    waypoints=[]
    for y in range(len(map)):
        for x in range(len(map[y])):
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
    return waypoints

if __name__ == "__main__":
    image = read_pgm("map.pgm", byteorder='<')

    #This is the matrix with the hex values
    #print(image)
    map = numpy.zeros([len(image), len(image[0])])
    for y in range(len(image)):
        for x in range(len(image[y])):
            if image[y][x] == 0:
                map[y][x] = 1
            elif image[y][x] == 205:
                map[y][x] = -1
            else:
                map[y][x] = 0

    #this is the matix converted to be 1s and 0s
    print(map)
    wp = waypoints(map)
    pyplot.imshow(image, pyplot.cm.gray)
    for p in wp:
        pyplot.scatter(p[0], p[1], color='blue')
    pyplot.show()
