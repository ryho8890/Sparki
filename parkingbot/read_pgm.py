import re
import numpy
import math

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
        if (map[y][x+1] == 0 and map[y+1][x] == 0 and x > 135 and x < 140) or (x > 192 and x < 197 and map[y][x-1] == 0 and map[y+1][x] == 0):
            y = y + 8
            if x < 140:
                x = x + 7
            if x > 192:
                x = x - 7
            if map[y][x] != -1:
                waypoints.append([x, y])
    return waypoints

if __name__ == "__main__":
    from matplotlib import pyplot
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
