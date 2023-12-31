import pandas as pd
import sumolib
from sumolib import geomhelper
import math
from shapely.geometry import LineString
import time
import numpy as np
import geotools
import rtree
import pyproj
import osmnx as ox


#########  class Node  #####################

class Node:
    def __init__(self, coord=None, id=None):
        self.coord = coord
        self.id = id
        self.outgoing = []  # list of edges
        self.incoming = []  # list of edges


    def addIncoming(self, e):
        self.incoming.append(e)

    def addOutgoing(self, e):
        self.outgoing.append(e)

    def getID(self):
        return self.id
    
    def getIncoming(self):
        return self.incoming.copy()
    
    def getOutgoing(self):
        return self.outgoing.copy()
    
    def getCoord(self):
        return self.coord



class Edge:
    def __init__(self, id=None, fromnode=None, tonode=None,
                 speed=None, length=None, shape=None):
        self.fromnode = fromnode
        self.tonode = tonode
        self.speed = speed
        self.length = length
        self.shape = shape
        self.id = id
        self.incoming = []  # list of edges
        self.outgoing = []  # list of edges

    def getIncoming(self):
        return self.incoming.copy()
    
    def getOutgoing(self):
        return self.outgoing.copy()

    def addIncoming(self, e):
        self.incoming.append(e)

    def addOutgoing(self, e):
        self.outgoing.append(e)

    def getID(self):
        return self.id
    
    def getShape(self):
        return self.shape.copy()
    
    def getSpeed(self):
        return self.speed
    
    def getLength(self):
        return self.length
    
    def getToNode(self):
        return self.tonode
    
    def getFromNode(self):
        return self.fromnode
    
    def getBoundingBox(self):
        return geotools.getBoundingBox(self.shape)
    



class Net:
    def __init__(self):
        self.outgoing = dict()
        self.incoming = dict()
        self.edges = dict()
        self.nodes = dict()
        self.geoproj = None
        self._location = dict()
        self._rtree = None
        self._edgeidlist = []

    def getNodes(self):
        return list(self.nodes.values())
    
    def getEdges(self):
        return list(self.edges.values())
    
    def getNode(self, n):
        return self.nodes[n]
    
    def getEdge(self, n):
        return self.edges[n]

    def importFromSumoNet(self, snet):

        #set geoproj
        self.geoproj = snet.getGeoProj()

        #set _location
        self._location  = snet._location

        # create dictionary for nodes
        for sn in snet.getNodes():
            nid = sn.getID()
            n = Node(id=nid, coord=sn.getCoord())
            self.nodes[nid] = n
  

        # create dictionary for edges
        for se in snet.getEdges():
            eid       = se.getID()
            efromnode = self.nodes[se.getFromNode().getID()]
            etonode   = self.nodes[se.getToNode().getID()]
            espeed    = se.getSpeed()
            elength   = se.getLength()
            eshape    = se.getRawShape()
            e         = Edge(id=eid, fromnode=efromnode, tonode=etonode,
                             speed=espeed, length=elength, shape=eshape)
            self.edges[eid] = e

            
        for e in self.edges.values():
            se = snet.getEdge(e.getID())
            for seout in se.getOutgoing().keys():
                e.addOutgoing(self.edges[seout.getID()])
            for sein in se.getIncoming().keys():
                e.addIncoming(self.edges[sein.getID()])

        
        for n in self.nodes.values():
            sn = snet.getNode(n.getID())
            for seout in sn.getOutgoing():
                n.addOutgoing(self.edges[seout.getID()])
            for sein in sn.getIncoming():
                n.addIncoming(self.edges[sein.getID()])

        
        self._edgeidlist = list(self.edges.keys())
        self._rtree = self._initRTree()


    def importFromOSM(self, osmfile):
        import pyrosm
        import networkx as nx
        from geotools import calculateUTMZone
        

        # create networkx graph from osm
        osm = pyrosm.OSM(osmfile)
        nodes, edges= osm.get_network(network_type="driving", nodes=True)
        G = osm.to_graph(nodes, edges, graph_type="networkx")
        #G = nx.classes.digraph.DiGraph(G)

        # set self._location and self.geoproj
        minlon = nodes["lon"].min()-.1
        maxlon = nodes["lon"].max()+.1
        minlat = nodes["lat"].min()-.1
        maxlat = nodes["lat"].max()+.1
        zone = calculateUTMZone(minlon, maxlon, minlat, maxlat)
        self._location["projParameter"] = f"+proj=utm +zone={zone} +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
        self._location["origBoundary"] = f"{minlon},{minlat},{maxlon},{maxlat}"
        
        self.geoproj = pyproj.Proj(self._location["projParameter"])#,preserve_units=True)
        x1,y1 = self.geoproj(minlon, minlat)
        x2,y2 = self.geoproj(maxlon, maxlat)

        self._location["convBoundary"] = f"{0},{0},{x2-x1},{y2-y1}"
        self._location["netOffset"] = f"{-x1},{-y1}"

        #graph simplification
        G = ox.simplify_graph(G)
       

        # create dictionary for nodes
        for node in G.nodes():
            p = G.nodes[node]["geometry"]
            lon = p.coords.xy[0][0]
            lat = p.coords.xy[0][0]
            x,y = self.convertLonLat2XY(lon,lat)
            n = Node(id=node, coord=(x,y))
            self.nodes[node] = n

        # create dictionary for edges
        for edge in G.edges:
            espeed  = G.edges[edge[0],edge[1],0]["maxspeed"]
            if(isinstance(espeed,list)):
                espeed = None #max([int(s) if s!=None else 30 for s in espeed])
            else:
                espeed = float(espeed) if espeed !=None else None
            G.edges[edge[0],edge[1],0]["maxspeed"] = espeed

        G = ox.add_edge_speeds(G)
       # G = ox.add_edge_travel_times(G)
        for edge in G.edges:
            eid = f"{edge[0]}_{edge[1]}"
            efromnode = self.nodes[edge[0]]
            etonode   = self.nodes[edge[1]]
            espeed    = G.edges[edge]["speed_kph"]*1000/3600 #km/h to m/s
            geometry = G.edges[edge]["geometry"]
            eshape = []
            for point in geometry.coords:
                eshape.append(self.convertLonLat2XY(*point))   
            elength = geotools.polyLength(eshape)
            e  = Edge(id=eid, fromnode=efromnode, tonode=etonode,
                            speed=espeed, length=elength, shape=eshape)
            self.edges[eid] = e



        for nodeid in self.nodes.keys():
            node = self.nodes[nodeid]
            for item in G.out_edges(nodeid):
                edgeid = f"{item[0]}_{item[1]}"
                node.addOutgoing(self.edges[edgeid])
            for item in G.in_edges(nodeid):
                edgeid = f"{item[0]}_{item[1]}"
                node.addIncoming(self.edges[edgeid])

        for edgeid in self.edges.keys():
            edge = self.edges[edgeid]
            fromnode = edge.getFromNode()
            tonode = edge.getToNode()
            
            for item in tonode.getOutgoing():
                edge.addOutgoing(self.edges[item.getID()])

            for item in fromnode.getIncoming():
                edge.addIncoming(self.edges[item.getID()])

        self._edgeidlist = list(self.edges.keys())
        self._rtree = self._initRTree()

        self.G = G



    def getLocationOffset(self):
        """ offset to be added after converting from geo-coordinates to UTM"""
        return list(map(float, self._location["netOffset"].split(",")))

    def convertLonLat2XY(self, lon, lat, rawUTM=False):
        x, y = self.geoproj(lon, lat)
        if rawUTM:
            return x, y
        else:
            x_off, y_off = self.getLocationOffset()
            return x + x_off, y + y_off


    def convertXY2LonLat(self, x, y, rawUTM=False):
        if not rawUTM:
            x_off, y_off = self.getLocationOffset()
            x -= x_off
            y -= y_off
        return self.geoproj(x, y, inverse=True)


    def _initRTree(self):
        result = rtree.index.Index()
        result.interleaved = True
        for ri in range(len(self._edgeidlist)):
            edge = self.edges[self._edgeidlist[ri]]
            result.add(ri, edge.getBoundingBox())
        return result
    

    def getNeighboringEdges(self, x, y, r=0.1):
        edges = []
      
        for i in self._rtree.intersection((x - r, y - r, x + r, y + r)):
            e = self.edges[self._edgeidlist[i]]
            d = sumolib.geomhelper.distancePointToPolygon(
                (x, y), e.getShape())
            if d < r:
                edges.append((e, d))
        return edges


########################################################################
#################3  end of the network class   #########################
########################################################################

def combineShapesSumo(edge, fromedge=None, edge_reverse=False, from_reverse =False):
    """
    Combines the raw shapes of Sumo edges, optionally considering the starting point from another edge.

    Args:
        edge (SumoEdge): Current Sumo edge.
        fromedge (SumoEdge, optional): Starting edge. Defaults to None.

    Returns:
        list: Combined raw shape of Sumo edges.

    """
         
    shape = edge.getShape()
    if edge_reverse==True:
        shape = list(reversed(shape))


    if fromedge is not None:
        if from_reverse==False or edge_reverse==False:
            p = fromedge.getShape()[-1]
        else:
            p = fromedge.getShape()[0]

        if p != shape[0]:
            shape = [p] + shape

    return shape

def getGeoShape(edge, net, fromedge=None, edge_reverse=False, from_reverse =False):
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
    for point in combineShapesSumo(edge, fromedge=fromedge, edge_reverse=edge_reverse, from_reverse=from_reverse):
        _shape.append(net.convertXY2LonLat(point[0], point[1]))
    return LineString(_shape)


