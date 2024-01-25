import numpy as np
import math

from constants import L

# constants
L = 5 # size of the box

# re-write
def clockwiseangle_and_distance(point):
    """
    Finds angle between point and ref vector ffor sortinf points in rotor
    in order of angles
    """
    refvec = (0,1)
    origin = (0.5, 0.5)
    # Vector between point and the origin: v = p - o
    vector = [point[0]-origin[0], point[1]-origin[1]]

    # Length of vector: ||v||
    lenvector = math.hypot(vector[0], vector[1])

    # If length is zero there is no angle
    if lenvector == 0:
        return -math.pi, 0

    # Normalize vector: v/||v||
    normalized = [vector[0]/lenvector, vector[1]/lenvector]
    dotprod  = normalized[0]*refvec[0] + normalized[1]*refvec[1]     # x1*x2 + y1*y2
    diffprod = refvec[1]*normalized[0] - refvec[0]*normalized[1]     # x1*y2 - y1*x2

    angle = math.atan2(diffprod, dotprod)

    # Negative angles represent counter-clockwise angles so we need to subtract them
    # from 2*pi (360 degrees)
    if angle < 0:
        return 2*math.pi+angle
    # I return first the angle because that's the primary sorting criterium
    return angle


def angle_to_xy(magnitude, angle):
    """
    Takes an angle in radians as input as returns x and y poistion for the corresponding angle
    using r as v_mag, a predifined value which is the magnitude of the velocity.
    """
    # get x using x = cos(angle) and y = sin(angle)
    x = magnitude * math.cos(angle)
    y = magnitude * math.sin(angle)

    return [x, y]

def distance_fun(pos1, pos2):
    """
    Calculate the distance between the points
    """
    # get the two arrays as np arrays, easier to do calculations
    pos1 = np.array(pos1)
    pos2 = np.array(pos2)

    # get the distance
    distance = pos2 - pos1

    # distance is the same as the magnitude
    dist = np.sqrt(distance.dot(distance))

    return dist

def centroid(pts):
    'Location of centroid.'

    # check if the last point is the same as the first, if nots so 'close' the polygon
    if pts[0] != pts[-1]:
        pts = pts + pts[:1]

    # get the x and y points
    x = [c[0] for c in pts]
    y = [c[1] for c in pts]

    # initialise the x and y centroid to 0 and get the area of the polygon
    sx = sy = 0
    a = area(pts)

    for i in range(len(pts) - 1):
        sx += (x[i] + x[i+1])*(x[i]*y[i+1] - x[i+1]*y[i])
        sy += (y[i] + y[i+1])*(x[i]*y[i+1] - x[i+1]*y[i])

    return [sx/(6*a), sy/(6*a)]

def area(pts):
    'Area of cross-section.'

    if pts[0] != pts[-1]:
      pts = pts + pts[:1]

    x = [ c[0] for c in pts ]
    y = [ c[1] for c in pts ]
    s = 0

    for i in range(len(pts) - 1):
        s += x[i]*y[i+1] - x[i+1]*y[i]

    return s/2

def rescale(magnitude, vector):
    """
    Changes the length of a  given vector to that of the magnitude given.
    """
    # make the vector a numpy array
    vec = np.array(vector)

    # get the magnitude
    mag = np.sqrt(vec.dot(vec))

    # multiply to rescale and make it a list
    new_vec = (magnitude / mag) * vec

    return new_vec



def per_boun_distance(i, j):
    """
    Calculates the minimum distance  between two particles in a box with periodic
    boundries.
    """
    # calculate the minimum x distance
    in_distance_x = j[0] - i[0]
    out_distance_x = L - in_distance_x
    distance_x = min(in_distance_x, out_distance_x)


    # calculate the minimum y distance
    in_distance_y = j[1] - i[1]
    out_distance_y = L - in_distance_y
    distance_y = min(in_distance_y, out_distance_y)

    return np.array([distance_x, distance_y])
