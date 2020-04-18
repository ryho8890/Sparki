import re
import numpy

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


if __name__ == "__main__":
    from matplotlib import pyplot
    image = read_pgm("map.pgm", byteorder='<')

    #This is the matrix with the hex values
    print(image)
    map = numpy.zeros([len(image), len(image[0])])
    for y in range(len(image)):
        for x in range(len(image[y])):
            if image[y][x] == 0:
                map[y][x] = 0
            else:
                map[y][x] = 1

    #this is the matix converted to be 1s and 0s
    print(map)
    pyplot.imshow(image, pyplot.cm.gray)
    pyplot.show()
