
import math
import numpy as np
import pandas as pd


def dfPoint2LonLat(df, net):
    df["lon"] = df.apply(lambda row: net.convertXY2LonLat(row["x"], row['y'])[0], axis=1)
    df["lat"] = df.apply(lambda row: net.convertXY2LonLat(row["x"], row['y'])[1], axis=1)
    return df.drop(columns=["x","y"])

def dfLonLat2XY(df, net):
    df["x"] = df.apply(lambda row: net.convertLonLat2XY(row["lon"], row['lat'])[0], axis=1)
    df["y"] = df.apply(lambda row: net.convertLonLat2XY(row["lon"], row['lat'])[1], axis=1)
    return df.drop(columns=["lon","lat"])

def distance2d(point1, point2):
    """
    Calculates the 2D Euclidean distance between two points.

    Args:
        point1 (tuple): Coordinates of the first point (x, y).
        point2 (tuple): Coordinates of the second point (x, y).

    Returns:
        float: 2D Euclidean distance between the two points.

    """
    return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5


def polyLength(polygon):
    return sum([distance2d(a, b) for a, b in zip(polygon[:-1], polygon[1:])])


def distancePointToLine(point, line):
    offset, dist = lineOffsetWithMinimumDistanceToPoint(point, *(line))
    return dist

def distancePointToPolygon(point, polygon):
    mindist = np.inf
    for i in range(len(polygon) - 1):
         mindist = np.min(mindist, distancePointToLine(point, polygon[i]))
    return mindist

def lineOffsetWithMinimumDistanceToPoint(point, line_start_point, line_end_point):
    """Return the offset from line (line_start, line_end) and distance from the point to that point
    where the distance to point is minimal"""
    p = point
    p1 = line_start_point
    p2 = line_end_point
    d = distance2d(p1, p2)
    u = ((p[0] - p1[0]) * (p2[0] - p1[0])) + ((p[1] - p1[1]) * (p2[1] - p1[1]))
    if u==0:
        return 0,distance2d(p,p1)
    elif  u < 0:
        return 0,distance2d(p,p1)
    elif u >=d*d:
        return d,distance2d(p,p2)
    else:
        return u/d,distance2d(p,(((d-u/d)*p1[0]+(u/d)*p2[0])/d, ((d-u/d)*p1[1]+(u/d)*p2[1])/d)) 


def polygonOffsetWithMinimumDistanceToPoint(point, polygon):
    """Return the offset and the distance from the polygon start where the distance to the point is minimal"""
    p = point
    s = polygon
    distlist = []
    offsetlist = []
    offset = 0
    
    for i in range(len(s) - 1):
        offset, distance = lineOffsetWithMinimumDistanceToPoint(p, s[i], s[i + 1])
        distlist.append(distance)
        offsetlist.append(offset)
    minindex = distlist.index(min(distlist))
    offset = 0
    for i in range(minindex):
        offset+=distance2d(s[i],s[i+1])
    
    if offsetlist[minindex] > distance2d(s[minindex],s[minindex+1]):
        return("errrror")
    #print(offset+distance2d(s[minindex],s[minindex+1]))
    u = offsetlist[minindex]
    d = distance2d(s[minindex], s[minindex+1])
    p1 = s[minindex]
    p2 = s[minindex+1]
    offsetposition = (((1-u/d)*p1[0]+(u/d)*p2[0]), ((1-u/d)*p1[1]+(u/d)*p2[1]))
    return offsetlist[minindex] + offset, offsetposition
        

def offsetBearing(polygon, offset):
    """
    Calculates the bearing angle at a specified offset along a polygon.

    Args:
        polygon (list): List of points representing the polygon.
        offset (float): Offset along the polygon to calculate the bearing angle.

    Returns:
        float: Bearing angle at the specified offset.

    """
    sum = 0

    if offset > polyLength(polygon):
        #polygon = coordlist
        # Check if offset is greater than polygon length
        print(f"offset = {offset}, polygon length = {polyLength(polygon)}")
        raise ValueError("offset is greater than polygon length.")
    
    if offset < 0:
        raise ValueError("expected the offset to be a positive number")


    for a, b in zip(polygon[:-1], polygon[1:]):
        sum += distance2d(a, b)

        if sum > offset:
            break

    return calculate_bearing_angle(a, b)




def calculate_bearing_angle(point1, point2):
    """
    Calculates the bearing angle between two points in a 2D space.
    Args:
        point1: Tuple representing the coordinates of the first point (x1, y1).
        point2: Tuple representing the coordinates of the second point (x2, y2).
    Returns:
        Bearing angle in degrees.
    """
    x1, y1 = point1
    x2, y2 = point2

    dx = x2 - x1
    dy = y2 - y1

    angle = math.atan2(dx,dy)
    bearing = math.degrees(angle)

    # Adjusting the bearing to be between 0 and 360 degrees
    if bearing < 0:
        bearing += 360
    return (bearing)

def road_distance(currentedge, currentoffset, lastedge , lastoffset, lastedgelength):
    """
    Calculates the road distance between two points on a road network.

    Args:
        currentedge (int): ID of the current edge.
        currentoffset (float): Offset along the current edge.
        lastedge (int): ID of the last edge.
        lastoffset (float): Offset along the last edge.
        lastedgelength (float): Length of the last edge.

    Returns:
        float: Road distance between the two points.

    """
    if currentedge == lastedge:
        # Points are on the same edge
        w = max(0, currentoffset - lastoffset)
    else:
        # Points are on different edges
        w = (lastedgelength - lastoffset) + currentoffset
    
    return w


def getBoundingBox(coordList):
    minX = 1e400
    minY = 1e400
    maxX = -1e400
    maxY = -1e400
    for x, y in coordList:
        minX = min(x, minX)
        minY = min(y, minY)
        maxX = max(x, maxX)
        maxY = max(y, maxY)
    return minX, minY, maxX, maxY