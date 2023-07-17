import pandas as pd
import sumolib
from sumolib import geomhelper
import math
from shapely.geometry import LineString
import time
import numpy as np


def combineRawShapesSumo(edge, fromedge=None, edge_reverse=False, from_reverse =False):
    """
    Combines the raw shapes of Sumo edges, optionally considering the starting point from another edge.

    Args:
        edge (SumoEdge): Current Sumo edge.
        fromedge (SumoEdge, optional): Starting edge. Defaults to None.

    Returns:
        list: Combined raw shape of Sumo edges.

    """
         
    shape = edge.getRawShape()
    if edge_reverse==True:
        shape = list(reversed(shape))


    if fromedge is not None:
        if from_reverse==False or edge_reverse==False:
            p = fromedge.getRawShape()[-1]
        else:
            p = fromedge.getRawShape()[0]

        if p != shape[0]:
            shape = [p] + shape

    return shape

def getRawShape(edge, net, edge_reverse=False, from_reverse =False, fromedge=None):
    """
    Retrieves the raw shape of an edge and converts it to a LineString geometry.

    Args:
        edge (SumoEdge): Current Sumo edge.
        net (SumoNet): Sumo network.
        fromedge (SumoEdge, optional): Starting edge. Defaults to None.

    Returns:
        LineString: LineString geometry representing the raw shape of the edge.

    """
    _shape = list()
    for point in combineRawShapesSumo(edge, fromedge, edge_reverse, from_reverse):
        _shape.append(net.convertXY2LonLat(point[0], point[1]))
    return LineString(_shape)


