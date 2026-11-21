#import networkx as nx
#import numpy as np
import pandas as pd

import network
from network import combineShapesSumo,getGeoShape
from geotools import distance2d,offsetBearing,polyLength,road_distance, polygonOffsetWithMinimumDistanceToPoint
from engine import use_rust, rust


def rust_network(net):
    """The network as the Rust engine reads it (made one time for each Net). Edge index = position in net._edgeidlist."""
    if getattr(net, "_rust_net", None) is None:
        ids = net._edgeidlist
        edge_index = {e: i for i, e in enumerate(ids)}
        nodes = list(net.nodes.values())
        node_index = {id(n): i for i, n in enumerate(nodes)}
        edges = [net.edges[e] for e in ids]
        net._rust_net = rust.Network(
            [[(float(p[0]), float(p[1])) for p in e.shape] for e in edges],
            [float(e.speed) for e in edges],
            [node_index[id(e.fromnode)] for e in edges],
            [node_index[id(e.tonode)] for e in edges],
            [[edge_index[o.getID()] for o in n.outgoing] for n in nodes],
            [[edge_index[o.getID()] for o in n.incoming] for n in nodes])
        net._rust_edge_index = edge_index
    return net._rust_net




class _Hyp:
    """A hypothesis of the beam search. path = (edge, reverse, length, previous) or None;
    matched = (matched point dict, previous) or None. The previous items are shared."""
    __slots__ = ("cost", "edge", "rev", "offset", "path", "matched")

    def __init__(self, cost, edge, rev, offset, path, matched):
        self.cost, self.edge, self.rev, self.offset, self.path, self.matched = cost, edge, rev, offset, path, matched


def _unlink(node):
    """The items of a linked path or matched list, first to last."""
    out = []
    while node is not None:
        out.append(node[:-1] if len(node) > 2 else node[0])
        node = node[-1]
    return out[::-1]


class MapMatcher:
    
    def __init__(self, net, MAX_GPS_ERROR=60, 
                 MAX_MAP_ERROR=40,
                 MAP_ONE_WAY_FIX=True,
                 U_TURN_ON_ONEWAY=False,
                 LOOP=True,
                 MAX_SPEED=100,
                 DIFF_GPS_ERROR=10,
                 MAX_RUNNING_TIME=5,
                 engine=None,
                 debug=False,
                 BEAM_WIDTH=5):
        
        if not isinstance(net, network.Net):
            raise ValueError("network.Net expected")
            
        self.net = net
        self.radius = MAX_GPS_ERROR + MAX_MAP_ERROR
        self.matchdf = None
        self.routedf = None
        
        self.MAX_GPS_ERROR = MAX_GPS_ERROR #meters
        self.MAX_MAP_ERROR = MAX_MAP_ERROR #meters
        self.MAP_ONE_WAY_FIX = MAP_ONE_WAY_FIX
        self.U_TURN_ON_ONEWAY = U_TURN_ON_ONEWAY
        self.LOOP = LOOP
        self.MAX_SPEED = MAX_SPEED #m/s
        self.MAX_RUNNING_TIME = MAX_RUNNING_TIME  #s
        self.DIFF_GPS_ERROR = DIFF_GPS_ERROR #meters
        self.path= []  # item: {"edge":,"reverese":,"length":}
        self.MINSPEED_BEARING = 1 #m/s
        self.engine = engine  # "auto", "rust" or "python"; see engine.py
        self.debug = debug    # True: write new_sample.csv and print the path
        self.BEAM_WIDTH = BEAM_WIDTH  # the number of hypotheses that the matcher keeps
        
        
    
    def reset(self):
        self.matchdf = None
        self.routedf = None
        self.path = []
        
        
     
    
    def decision_stay_change_nodecide(self,remind_offset, speed, maxspeed, deltaTime):
        """
        Determines the decision (stay, change, or no decision) based on edge length, offset, speed, and time.

        Args:

            remind_offset (float): the length of rest of the edge after offset.
            speed (float): Current speed.
            deltaTime (float): Time interval.

        Returns:
            str: Decision (stay, change, or no decision).

        """

        if remind_offset < (speed * deltaTime) - self.DIFF_GPS_ERROR:  # RADIUS
            return "CHANGE"
        elif remind_offset >= (maxspeed * deltaTime) + self.DIFF_GPS_ERROR:  # RADIUS
            return "STAY"
        else:
            return "NODECISION"
    

    
    
    def first_point_matching(self,x,y):
        
        #x, y = self.net.convertLonLat2XY(lon, lat)
        edges = self.net.getNeighboringEdges(x, y, self.radius)
        edges = [edge for edge,dist in edges]
        reversedict = {edge:False for edge in edges}
        return set(edges),reversedict
    
    
    
    
    def cost_calculate(self, bearing_error, match_point_distance, air_distance_error, road_distance_error, reverse):
        """
        Calculates the cost for a match point based on different error values.

        Args:
            bearing_error (float): Bearing error for the match point.
            match_point_distance (float): Distance to the match point.
            air_distance_error (float): Air distance error for the match point.
            road_distance_error (float): Road distance error for the match point.

        Returns:
            float: Cost value for the match point.

        """
        cost = 0
        if reverse == True:
            cost+= 100000
        cost+= 1*bearing_error + 30*match_point_distance  + 10*air_distance_error  +5*road_distance_error
        return  cost

    
    def outgoinglist(self, edge, reverse, path=None):
        """
        Retrieves a list of outgoing edge IDs from a given edge.

        Args:
            edge (SumoEdge): Current Sumo edge.

        Returns:
            list: List of outgoing edge IDs.

        """
        #out = list(edge.getOutgoing().keys())
        if self.MAP_ONE_WAY_FIX: 
            if reverse==True:
                node = edge.getFromNode()
            else:
                node = edge.getToNode()

            out = node.getOutgoing() + node.getIncoming()
            if (not self.U_TURN_ON_ONEWAY) and (edge in out):
                out.remove(edge)
            
            if path is None:
                path = [item["edge"] for item in self.path]

            if( not self.LOOP):
                for item in path:
                    if item in out:
                        out.remove(item)
            reversedict={edge:True for edge in node.getIncoming()}

            reversedict.update({edge:False for edge in node.getOutgoing()})
        else:
            node = edge.getToNode()
            out = node.getOutgoing()
            if path is None:
                path = [item["edge"] for item in self.path]
            if( not self.LOOP):
                for item in path:
                    if item in out:
                        out.remove(item)
            reversedict={edge:False for edge in out}


        return set(out),reversedict


        #print({edge:False for edge in tonode.getOutgoing()})
        #return tonode.getOutgoing(), {edge:False for edge in tonode.getOutgoing()}

    
    
    
    
    def match(self, sample_gps):
        """
        Matches GPS points to road network edges with a beam search.

        The matcher keeps at most BEAM_WIDTH hypotheses (a path, the last matched point and the
        total cost). For each point, it extends each hypothesis to its candidate edges (stay on the
        edge, change to an outgoing edge, or both, from the speed and the rest of the edge). A
        candidate farther than MAX_GPS_ERROR + MAX_MAP_ERROR from the point is not used. The
        BEAM_WIDTH children with the lowest total cost stay, at most one for each (edge, reverse).
        When no hypothesis can continue, the match starts again from the edges near the point (a
        gap). When no edge is near the point, the matcher skips the point.

        Args:
            sample_gps (DataFrame): columns x, y, timestamp, speed, bearing, type.

        Returns:
            int: 1, with the result in self.matchdf and self.path (0 when sample_gps is not a DataFrame).
        """
        if not isinstance(sample_gps, pd.DataFrame):
            return 0
        if self.debug:
            sample_gps.to_csv("new_sample.csv", index=False)
        if use_rust(self.engine):
            return self._match_rust(sample_gps)
        beam = []
        for index, row in sample_gps.iterrows():
            beam = self.step(beam, index, row)
        best = beam[0] if beam else _Hyp(0.0, None, False, None, None, None)
        self.path = [{"edge": e, "reverse": r, "length": l} for e, r, l in _unlink(best.path)]
        self.matchdf = pd.DataFrame(_unlink(best.matched))
        if self.debug:
            self.show_path()
        return 1

    def _edge_index(self, edge):
        if getattr(self, "_index", None) is None:
            self._index = {e: i for i, e in enumerate(self.net._edgeidlist)}
        return self._index[edge.getID()]

    def step(self, beam, index, row):
        """One point of the beam search: returns the new beam (sorted by total cost)."""
        children = []
        for h in beam:
            remind_offset = h.path[2] - h.offset
            decision = self.decision_stay_change_nodecide(remind_offset, row.speed, h.edge.getSpeed(), 1)
            if decision == "STAY":
                edges, reversedict = {h.edge}, {h.edge: h.rev}
            else:
                path = [p[0] for p in _unlink(h.path)]
                edges, reversedict = self.outgoinglist(h.edge, h.rev, path)
                if decision == "NODECISION":
                    edges.add(h.edge)
                    reversedict[h.edge] = h.rev
            children += self._extend(h, index, row, decision, edges, reversedict)
        if not children:
            # no hypothesis can continue: start again from the edges near the point (a gap)
            base = beam[0] if beam else _Hyp(0.0, None, False, None, None, None)
            start = _Hyp(base.cost, None, False, None, base.path, base.matched)
            edges, reversedict = self.first_point_matching(row.x, row.y)
            children = self._extend(start, index, row, "CHANGE", edges, reversedict)
            if not children:
                return beam  # no edge near the point: skip it
        children.sort(key=lambda c: c.cost)
        out, seen = [], set()
        for c in children:
            if (c.edge, c.rev) not in seen:
                seen.add((c.edge, c.rev))
                out.append(c)
                if len(out) == self.BEAM_WIDTH:
                    break
        return out

    def _extend(self, h, index, row, decision, edges, reversedict):
        """The children of hypothesis h on the candidate edges (in the sequence of net._edgeidlist)."""
        out = []
        x, y = row.x, row.y
        for edge in sorted(edges, key=self._edge_index):
            rev = reversedict[edge]
            if h.edge is not None and edge != h.edge:
                temp_edge, temp_reverse = h.edge, h.rev
            elif h.edge is not None and h.path[3] is not None:   # stay: the edge before it
                temp_edge, temp_reverse = h.path[3][0], h.path[3][1]
            else:
                temp_edge, temp_reverse = None, False
            shape = combineShapesSumo(edge, temp_edge, rev, temp_reverse)
            edge_length = polyLength(shape)
            offset, matchpoint = polygonOffsetWithMinimumDistanceToPoint([x, y], shape)
            dist = distance2d(matchpoint, (x, y))
            if dist > self.radius:
                continue
            matched_bearing = offsetBearing(shape, offset)
            dist_bearing = min(abs(row.bearing - matched_bearing), 360 - abs(row.bearing - matched_bearing))
            if h.matched is not None:
                last = h.matched[0]
                lastedge = self.net.getEdge(last["edgeid"])
                lastoffset, last_edge_length = last["offset"], last["edge_length"]
                last_sample, last_matched = (last["x_sample"], last["y_sample"]), (last["x"], last["y"])
            else:
                lastedge, lastoffset, last_edge_length = edge, offset, 0
                last_sample, last_matched = (x, y), matchpoint
            cost_air = abs(distance2d(last_sample, (x, y)) - distance2d(last_matched, matchpoint))
            predict_distance = float(row.speed) * 1
            matched_road_distance = road_distance(edge, offset, lastedge, lastoffset, last_edge_length)
            rd = abs(matched_road_distance - predict_distance)
            cost = self.cost_calculate(dist_bearing, dist, cost_air, rd, rev)
            rec = {"index": index, "timestamp": row.timestamp, "x": matchpoint[0], "y": matchpoint[1],
                   "bearing": row.bearing, "rd": rd, "dist": dist,
                   "from_edge": temp_edge.getID() if temp_edge is not None else None,
                   "predict_distance": predict_distance, "speed": row.speed,
                   "matched_road_distance": matched_road_distance, "y_sample": y, "x_sample": x,
                   "cost_air": cost_air, "decitsion": decision, "matchbearing": matched_bearing,
                   "edgeid": edge.getID(), "offset": offset, "distbearing": dist_bearing,
                   "edge_length": edge_length, "type": row["type"], "edge_reverse": rev,
                   "from_edge_reverse": temp_reverse}
            if edge == h.edge:
                path = h.path
            else:
                frm = h.path
                length = polyLength(combineShapesSumo(edge, frm[0] if frm else None, rev,
                                                      frm[1] if frm else False))
                path = (edge, rev, length, h.path)
            out.append(_Hyp(h.cost + cost, edge, rev, offset, path, (rec, h.matched)))
        return out

    def _match_rust(self, sample_gps):
        """match() with the Rust engine: the same matchdf and path as the Python code."""
        rnet = rust_network(self.net)
        ids = self.net._edgeidlist
        m = rust.Matcher(rnet, self.MAX_GPS_ERROR, self.MAX_MAP_ERROR, self.MAP_ONE_WAY_FIX,
                         self.U_TURN_ON_ONEWAY, self.LOOP, self.DIFF_GPS_ERROR, beam_width=self.BEAM_WIDTH)
        m.match_trace(sample_gps.index.astype("int64").tolist(),
                      sample_gps["x"].astype(float).tolist(), sample_gps["y"].astype(float).tolist(),
                      sample_gps["speed"].astype(float).tolist(), sample_gps["bearing"].astype(float).tolist())
        self.path = [{"edge": self.net.getEdge(ids[e]), "reverse": rev, "length": length}
                     for e, rev, length in m.path()]
        r = m.result()
        if not r["pos"]:
            self.matchdf = pd.DataFrame([])
            return 1
        rows = sample_gps.iloc[r["pos"]]
        names = ("STAY", "CHANGE", "NODECISION")
        self.matchdf = pd.DataFrame({
            "index": r["label"], "timestamp": rows["timestamp"].to_numpy(), "x": r["x"], "y": r["y"],
            "bearing": rows["bearing"].to_numpy(), "rd": r["rd"], "dist": r["dist"],
            "from_edge": [ids[e] if e >= 0 else None for e in r["from_edge"]],
            "predict_distance": r["predict_distance"], "speed": rows["speed"].to_numpy(),
            "matched_road_distance": r["matched_road_distance"], "y_sample": rows["y"].to_numpy(),
            "x_sample": rows["x"].to_numpy(), "cost_air": r["cost_air"],
            "decitsion": [names[d] for d in r["decision"]], "matchbearing": r["matchbearing"],
            "edgeid": [ids[e] for e in r["edge"]], "offset": r["offset"], "distbearing": r["distbearing"],
            "edge_length": r["edge_length"], "type": rows["type"].to_numpy(),
            "edge_reverse": r["edge_reverse"], "from_edge_reverse": r["from_edge_reverse"]})
        return 1

    def save_routematch(self, routematchfile=None):
        """
        Saves the route matching results to a file and returns the route dataframe.

        Args:
            routematchfile (str): File path to save the route matching results (optional).

        Returns:
            pandas.DataFrame: Route dataframe containing the route matching results.

        """
        # Create a new dataframe with unique from_edge and edgeid combinations
        self.routedf = self.matchdf[["from_edge", "edgeid","edge_reverse","from_edge_reverse"]].drop_duplicates()
        self.routedf = self.routedf.set_index(["from_edge", "edgeid","edge_reverse","from_edge_reverse"])

        # Calculate departure, arrival, stop time, and travel time for each from_edge and edgeid combination
        self.routedf["departure"] = self.matchdf.groupby(["from_edge", "edgeid"], dropna=False)["timestamp"].first()
        self.routedf["arrival"] = self.matchdf.groupby(["from_edge", "edgeid"], dropna=False)["timestamp"].last()
        self.routedf["stop_time"] = self.matchdf.groupby(["from_edge", "edgeid"], dropna=False)["speed"].apply(lambda col: sum([1 if x == 0 else 0 for x in col]))
        self.routedf["travel_time"] = self.routedf.apply(lambda row: int(row.arrival - row.departure), axis=1)
        self.routedf.reset_index(inplace=True)

        # Get the shape for each edgeid and from_edge combination
        
        self.routedf["shape"] = self.routedf.apply(lambda x: getGeoShape(self.net.getEdge(x.edgeid),
                                                             self.net,
                                                             edge_reverse=x.edge_reverse,
                                                             from_reverse=x.from_edge_reverse,
                                                             fromedge=self.net.getEdge(x.from_edge) if isinstance(x.from_edge, str) else None), axis=1)

        if routematchfile is not None:
            self.routedf.to_csv(routematchfile, index=False)
        return self.routedf

    

    
    def save_pointmatch(self, pointmatchfile=None):
        """
        Saves the point matching results to a file and returns the match dataframe.

        Args:
            pointmatchfile (str): File path to save the point matching results (optional).

        Returns:
            pandas.DataFrame: Match dataframe containing the point matching results.

        """
        if pointmatchfile is not None:
            self.matchdf.to_csv(pointmatchfile, index=False)
        return self.matchdf

    def show_path(self):
        print([item["edge"].getID() for item in self.path])


# Example usage
# Create a directed graph


# Create a map matcher instance

# Define a sequence of observations

# Perform map matching
