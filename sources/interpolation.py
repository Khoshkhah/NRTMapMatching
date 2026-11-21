from geotools import distance2d,calculate_bearing_angle, polyLength, polygonOffsetWithMinimumDistanceToPoint
import heapq
import math
import pandas as pd
import numpy as np
from engine import use_rust, rust

def Bezier_3(p0,p1,p2,p3,t):
    b0 = (1-t)**3
    b1 = 3*((1-t)**2)*t
    b2 = 3*(1-t)*(t**2)
    b3 = t**3   
    p = b0*p0 + b1*p1 + b2*p2 + b3*p3
    return p


def Bezier_2(p0,p1,p2,t):
    b0 = (1-t)**2
    b1 = 2*(1-t)*t
    b2 = (t**2)    
    p=b0*p0 + b1*p1 + b2*p2
    return p


def derivative_Bezier_3(p0,p1,p2,p3,t):
    q0 = 3*(p1-p0)
    q1 = 3*(p2-p1)
    q2 = 3*(p3-p2)  
    derivative = Bezier_2(q0,q1,q2,t)
    
    return derivative
   
    
def derivative2Bearing(p):
    if p[0]!= 0:
        slope = p[1]/p[0]
    else:
        slope = np.inf
    
    angle = np.arctan(slope) * 180/ np.pi
    bearing = 90 - angle
    
    if p[0] < 0 or (p[0]==0 and p[1]<0):
        bearing += 180 
    return bearing



def bezierInterpolation(a,b, sample_rate=1):
    items = []
    delta_time = b["timestamp"] - a["timestamp"]
    point_a = np.array([a['x'], a["y"]]) 
    point_b = np.array([b['x'], b['y']]) 
    
    if a["stopindex"]>0 and b["stopindex"]>0:
        time = a["timestamp"]
        while time  <= b["timestamp"]:

            if time==a["timestamp"] or time==b["timestamp"]:
                types = "origin"
            else:
                types = "extra"
            items.append({'x':point_a[0], 'y':point_a[1],  "timestamp":int(time),
                    "speed":0, "bearing":a["bearing"], "type":types, "stopindex":a["stopindex"]})
            
            time+=sample_rate
            if time > b['timestamp'] and (time -sample_rate)!= b['timestamp']:
                time = b['timestamp']

        return pd.DataFrame(items)
    
    # Cubic Hermite curve as a Bezier: the control points are at gap time x velocity / 3 from the
    # end points (velocity = speed in the direction of the bearing). The old formula multiplied a
    # distance-based length by the speed again, so the control points were at 60-90 % of the gap at
    # city speeds, and the curve made loops (Monaco 30 s: mean error 25 m, now 20 m).
    v_a = a["speed"] * np.array((np.sin(a["bearing"] * np.pi / 180.0), np.cos(a["bearing"] * np.pi / 180.0)))
    v_b = b["speed"] * np.array((np.sin(b["bearing"] * np.pi / 180.0), np.cos(b["bearing"] * np.pi / 180.0)))
    p0 = point_a + (delta_time * v_a) / 3.0
    p1 = point_b - (delta_time * v_b) / 3.0

    time = 0
    zero_point = np.array((0,0))

    while time + a["timestamp"] <= b["timestamp"]:
        
        t = time/delta_time
        p_t = Bezier_3(point_a,p0,p1,point_b,t)
        

        if t==0:
            types = "origin"
            p_t_1 = Bezier_3(point_a,p0,p1,point_b,t+(sample_rate/delta_time))
            speed_t = distance2d(p_t, p_t_1)/sample_rate

        elif t==1:
            types = "origin"
            speed_t = None
        else:
            types = "extra"
            p_t_1 = Bezier_3(point_a,p0,p1,point_b,t+(sample_rate/delta_time))
            speed_t = distance2d(p_t, p_t_1)/sample_rate
            
        v_t = derivative_Bezier_3(point_a,p0,p1,point_b,t)
            
    
        bearing_t = calculate_bearing_angle(zero_point, v_t)
        items.append({"x":p_t[0], 'y':p_t[1], "timestamp":int(time + a["timestamp"]),
                    "speed":speed_t, "bearing":bearing_t, "type":types,"stopindex":0})

        
        time = time + sample_rate
        if time > delta_time and (time -sample_rate)!= delta_time:
            time = delta_time
            
    df = pd.DataFrame(items)
    return df




def interpolateTrajectory(traj, sample_rate=1, engine=None, net=None, ROAD_MIN_GAP=6,
                          ROAD_DETOUR_FACTOR=2, ROAD_DETOUR_M=100):
    """Points at each sample_rate seconds between the points of traj (columns x, y, timestamp,
    speed, bearing, stopindex).

    Without net: a cubic Bezier curve between each two points.
    With net (a network.Net): road interpolation. In a gap of ROAD_MIN_GAP seconds or more, the
    points go along the shortest road path between the two points, at constant speed. If that path
    is longer than ROAD_DETOUR_FACTOR x the straight distance + ROAD_DETOUR_M metres (GPS noise or
    a wrong edge), or if no path is found, the gap gets the Bezier curve. Shorter gaps and stops
    also get the Bezier curve.
    """
    if net is not None:
        return roadInterpolation(traj, net, sample_rate, ROAD_MIN_GAP, ROAD_DETOUR_FACTOR, ROAD_DETOUR_M,
                                 engine)
    if len(traj) >= 2 and use_rust(engine):
        label, x, y, ts, speed, bearing, origin, stop = rust.interpolate(
            traj["x"].astype(float).tolist(), traj["y"].astype(float).tolist(),
            traj["timestamp"].astype(float).tolist(), traj["speed"].astype(float).tolist(),
            traj["bearing"].astype(float).tolist(), traj["stopindex"].astype("int64").tolist(),
            float(sample_rate))
        return add_confidence(pd.DataFrame({"x": x, "y": y, "timestamp": ts, "speed": speed, "bearing": bearing,
                             "type": ["origin" if o else "extra" for o in origin], "stopindex": stop},
                            index=label))
    interpol_list = []
    time = traj.iloc[0]["timestamp"]
    a = dict(traj.iloc[0])
    for index,row in traj[1:].iterrows():  # to the last point (before: traj[1:-1] left out the last segment)
            b=dict(row.copy())
            mydict = bezierInterpolation(a,b, sample_rate=sample_rate)            
            interpol_list.append(mydict)
            a = b.copy()
            if a["stopindex"]>0:
                a["x"] = mydict.iloc[-1]["x"]
                a["y"] = mydict.iloc[-1]["y"]

            time = row["timestamp"]

    last_speed = traj["speed"].iloc[-1]
    lastindex = interpol_list[-1].index[-1]
    interpol_list[-1].at[lastindex,"speed"] = last_speed
    df = pd.concat(interpol_list).reset_index(drop=True)
    df = df.dropna()
    return add_confidence(df)



# ---------------------------------------------------------------------------------------------
# Road interpolation: the points of a long gap go along the shortest road path
# ---------------------------------------------------------------------------------------------

SNAP_RADIUS = 50.0  # metres: the edges near a GPS point that can carry the road path
SNAP_COUNT = 3      # the nearest edges of each GPS point that the road path can start or end on


class RoadPath:
    """The shortest road path between two points on a network.Net (Dijkstra on the edge lengths)."""

    def __init__(self, net):
        self.net = net
        self.index = {e: i for i, e in enumerate(net._edgeidlist)}
        self.length = {e: polyLength(net.edges[e].getShape()) for e in net._edgeidlist}

    def snap(self, p):
        """The SNAP_COUNT nearest edges: (distance, edge index, edge, offset, point on the edge)."""
        out = []
        for e, _d in self.net.getNeighboringEdges(p[0], p[1], SNAP_RADIUS):
            off, q = polygonOffsetWithMinimumDistanceToPoint(list(p), e.getShape())
            out.append((distance2d(p, q), self.index[e.getID()], e, off, q))
        # the two directions of a road are at the same distance; the last bit of the distance can
        # differ between Python and Rust, so the sort uses the distance in micrometres, then the edge index
        out.sort(key=lambda c: (math.floor(c[0] * 1e6 + 0.5), c[1]))
        return out[:SNAP_COUNT]

    def node_path(self, ea, eb, limit):
        """(length, edges) of the shortest path from the end of ea to the start of eb, or None."""
        src, dst = ea.getToNode(), eb.getFromNode()
        if src is dst:
            return 0.0, []
        dist, prev, heap, n = {src.getID(): 0.0}, {}, [(0.0, 0, src)], 1
        while heap:
            d, _, u = heapq.heappop(heap)
            if d > dist.get(u.getID(), math.inf) or d > limit:
                continue
            if u is dst:
                edges = []
                while u.getID() in prev:
                    e = prev[u.getID()]
                    edges.append(e)
                    u = e.getFromNode()
                return d, edges[::-1]
            for e in u.getOutgoing():
                v, nd = e.getToNode(), d + self.length[e.getID()]
                if nd < dist.get(v.getID(), math.inf):
                    dist[v.getID()], prev[v.getID()] = nd, e
                    n += 1
                    heapq.heappush(heap, (nd, n, v))
        return None

    def polyline(self, a, b):
        """The road polyline from near a to near b (without the legs to a and b), or None."""
        air = distance2d(a, b)
        limit = max(3 * air, air + 300)
        best = None
        for da, _ia, ea, oa, qa in self.snap(a):
            for db, _ib, eb, ob, qb in self.snap(b):
                if ea is eb and ob >= oa:
                    length, mid = ob - oa, None
                else:
                    r = self.node_path(ea, eb, limit)
                    if r is None:
                        continue
                    length, mid = (self.length[ea.getID()] - oa) + r[0] + ob, r[1]
                score = length + 2 * (da + db)
                if best is None or score < best[0]:
                    best = (score, ea, oa, qa, eb, ob, qb, mid)
        if best is None:
            return None
        _, ea, oa, qa, eb, ob, qb, mid = best
        if mid is None:
            core = [qa] + _cut(ea.getShape(), oa, ob) + [qb]
        else:
            core = [qa] + _cut(ea.getShape(), oa, math.inf)
            for e in mid:
                core += list(e.getShape())[1:]
            core += _cut(eb.getShape(), 0, ob) + [qb]
        poly = [tuple(p) for p in core]
        poly = [p for i, p in enumerate(poly) if i == 0 or p != poly[i - 1]]
        return poly if len(poly) > 1 else None


def _cut(shape, lo, hi):
    """The shape points after the offset lo, up to the offset hi."""
    pts, acc = [], 0.0
    for p0, p1 in zip(shape[:-1], shape[1:]):
        d = distance2d(p0, p1)
        if acc + d > lo and acc < hi and acc + d <= hi:
            pts.append(p1)
        acc += d
    return pts


def _along(poly, s):
    """The point and the bearing at the distance s along a polyline."""
    acc = 0.0
    for a, b in zip(poly[:-1], poly[1:]):
        d = distance2d(a, b)
        if d > 0 and acc + d >= s:
            f = (s - acc) / d
            return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])), calculate_bearing_angle(a, b)
        acc += d
    a, b = poly[-2], poly[-1]
    return tuple(b), calculate_bearing_angle(a, b)


def roadInterpolation(traj, net, sample_rate=1, min_gap=6, detour_factor=2, detour_m=100, engine=None):
    """interpolateTrajectory(traj, net=net): see there."""
    # plain Python floats, not numpy numbers: numpy calculates x ** 0.5 with sqrt, Python and Rust with
    # pow, and a difference of one bit can change the nearest edge at a tie
    X, Y, T, S, B = (traj[c].astype(float).tolist() for c in ("x", "y", "timestamp", "speed", "bearing"))
    ST = traj["stopindex"].astype("int64").tolist()
    if use_rust(engine):
        from mapmatching import rust_network
        label, x, y, ts, speed, bearing, origin, stop = rust.interpolate_road(
            rust_network(net), X, Y, T, S, B, ST,
            float(sample_rate), float(min_gap), float(detour_factor), float(detour_m))
        return add_confidence(pd.DataFrame({"x": x, "y": y, "timestamp": ts, "speed": speed, "bearing": bearing,
                             "type": ["origin" if o else "extra" for o in origin], "stopindex": stop}))
    if getattr(net, "_road_path", None) is None:
        net._road_path = RoadPath(net)
    road = net._road_path
    rows = []
    for i in range(len(X) - 1):
        a, b, dt = (X[i], Y[i]), (X[i + 1], Y[i + 1]), T[i + 1] - T[i]
        poly = None
        if not (ST[i] > 0 and ST[i + 1] > 0) and dt >= min_gap:
            poly = road.polyline(a, b)
            if poly is not None and polyLength(poly) > detour_factor * distance2d(a, b) + detour_m:
                poly = None
        if poly is None:
            ka = {"x": X[i], "y": Y[i], "timestamp": T[i], "speed": S[i], "bearing": B[i], "stopindex": ST[i]}
            kb = {"x": X[i + 1], "y": Y[i + 1], "timestamp": T[i + 1], "speed": S[i + 1], "bearing": B[i + 1],
                  "stopindex": ST[i + 1]}
            seg = bezierInterpolation(ka, kb, sample_rate=sample_rate)
            for r in seg[seg["timestamp"] < T[i + 1]].itertuples():
                rows.append((r.x, r.y, int(r.timestamp), r.speed, r.bearing, r.type, int(r.stopindex)))
            continue
        length = polyLength(poly)
        k = 0
        while k * sample_rate < dt:
            q, br = _along(poly, length * (k * sample_rate) / dt)
            if k == 0:
                q = a   # the GPS point stays; it gets the bearing of the road
            rows.append((q[0], q[1], int(T[i] + k * sample_rate), length / dt, br, "origin" if k == 0 else "extra", 0))
            k += 1
    rows.append((X[-1], Y[-1], int(T[-1]), S[-1], B[-1], "origin", int(ST[-1])))
    return add_confidence(pd.DataFrame(rows, columns=["x", "y", "timestamp", "speed", "bearing", "type", "stopindex"]))


def add_confidence(df):
    """The column confidence: 1 at a real GPS point (type origin), down to 0 in the middle of a gap.
    For a point at the time t between the real points t0 and t1: 1 - min(t - t0, t1 - t) / ((t1 - t0) / 2).
    MapMatcher(INTERP_WEIGHT=...) uses it to give the guessed points less weight."""
    t = df["timestamp"].to_numpy(dtype=float)
    o = np.unique(t[(df["type"] == "origin").to_numpy()])
    if len(o) == 0:
        df["confidence"] = 1.0
        return df
    i1 = np.clip(np.searchsorted(o, t, side="left"), 0, len(o) - 1)
    i0 = np.clip(np.searchsorted(o, t, side="right") - 1, 0, len(o) - 1)
    t0, t1 = o[i0], o[i1]
    half = (t1 - t0) / 2
    with np.errstate(invalid="ignore", divide="ignore"):
        c = 1 - np.minimum(t - t0, t1 - t) / half
    df["confidence"] = np.where(half > 0, np.clip(c, 0.0, 1.0), 1.0)
    return df
