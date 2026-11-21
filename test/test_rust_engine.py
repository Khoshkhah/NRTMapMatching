"""The Rust engine (rust/nrt-matcher) gives the same results as the Python code.

Each stage runs on real trips of data/gps_inside_island.zip on the network data/iland.net:
cleaningData, richdata, interpolateTrajectory and MapMatcher.match. The match must give the same
edge for each point; the numbers must agree to a small tolerance.

Ties: when two edges have exactly the same cost, the Python code takes the edge that comes first
in a Python set. That sequence follows memory addresses, so it can change from one run to the next.
The Rust engine takes the edge with the lowest index in Net._edgeidlist. So that the comparison is
exact, the Python reference here takes the same edge at a tie (`tie_min`); the test counts the ties.

MapMatcher.match writes
new_sample.csv into the current folder, so the test runs in a temporary folder.

    pytest test/test_rust_engine.py              # 40 trips
    python test/test_rust_engine.py 400          # more trips, with a summary
"""
import contextlib
import io
import os
import sys
import tempfile
import zipfile

import numpy as np
import pandas as pd
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "sources"))

import engine  # noqa: E402

pytestmark = pytest.mark.skipif(engine.rust is None, reason="nrt_matcher is not installed")

TOL = 1e-6  # metres and degrees
TEXT_COLS = ["edgeid", "from_edge", "decitsion", "type", "edge_reverse", "from_edge_reverse", "index", "timestamp"]


def load():
    import sumolib
    import network
    net = network.Net()
    net.importFromSumoNet(sumolib.net.readNet(os.path.join(ROOT, "data/iland.net"), withInternal=False))
    with zipfile.ZipFile(os.path.join(ROOT, "data/gps_inside_island.zip")) as zf:
        gps = pd.read_csv(zf.open("gps_inside_island.csv"),
                          usecols=["trip_id_extended", "timestamp", "lat", "lon", "speed", "bearing"])
    return net, gps


def trips(gps, n):
    ids = gps["trip_id_extended"].unique()
    step = max(1, len(ids) // n)
    for t in ids[::step][:n]:
        yield t, gps[gps["trip_id_extended"] == t].sort_values("timestamp").drop_duplicates()


def frames_equal(a, b):
    """'' when equal (numbers to TOL), else the first difference."""
    if list(a.columns) != list(b.columns) or len(a) != len(b):
        return f"shape {a.shape} != {b.shape}"
    if not np.array_equal(a.index.to_numpy(), b.index.to_numpy()):
        return "index"
    for c in a.columns:
        x, y = a[c].to_numpy(), b[c].to_numpy()
        if c in TEXT_COLS or x.dtype == object:
            bad = [i for i in range(len(x)) if not (x[i] == y[i] or (pd.isna(x[i]) and pd.isna(y[i])))]
        else:
            bad = np.flatnonzero(~np.isclose(x.astype(float), y.astype(float), rtol=0, atol=TOL, equal_nan=True))
        if len(bad):
            i = bad[0]
            return f"column {c}: {len(bad)} rows, first row {i}: {x[i]!r} != {y[i]!r}"
    return ""


def run(f):
    """(result, error text); the Python code prints much, so stdout is muted."""
    with contextlib.redirect_stdout(io.StringIO()):
        try:
            return f(), ""
        except Exception as e:  # both engines must fail in the same way
            return None, f"{type(e).__name__}: {e}"


def tie_min(net, ties):
    """`min` for mapmatching.py: at a cost tie, the edge with the lowest index (the rule of the Rust engine)."""
    index = {e: i for i, e in enumerate(net._edgeidlist)}

    def m(*a, key=None):
        if key is None:
            return min(*a)
        best = min(key(i) for i in a[0])
        tied = [i for i in a[0] if key(i) == best]
        if len(tied) > 1:
            ties.append(tied[0]["timestamp"])
        return min(tied, key=lambda i: index[i["edge"].getID()])
    return m


def compare_trip(net, s):
    """Run each stage with both engines. Returns {stage: difference text} for the stages that differ."""
    from cleandata import cleaningData, richdata
    from interpolation import interpolateTrajectory
    from mapmatching import MapMatcher
    out = {}

    cp, ep = run(lambda: cleaningData(s.copy(), net, engine="python"))
    cr, er = run(lambda: cleaningData(s.copy(), net, engine="rust"))
    if ep or er:
        out["clean"] = f"error {ep!r} / {er!r}" if ep != er else ""
        return {k: v for k, v in out.items() if v}
    out["clean"] = frames_equal(cp, cr)

    rp, ep = run(lambda: richdata(s.copy(), net, engine="python"))
    rr, er = run(lambda: richdata(s.copy(), net, engine="rust"))
    out["enrich"] = (f"error {ep!r} / {er!r}" if ep != er else "") if ep or er else frames_equal(rp, rr)

    ip, ep = run(lambda: interpolateTrajectory(cp, engine="python"))
    ir, er = run(lambda: interpolateTrajectory(cp, engine="rust"))
    if ep or er:
        out["interpolate"] = f"error {ep!r} / {er!r}" if ep != er else ""
        return {k: v for k, v in out.items() if v}
    out["interpolate"] = frames_equal(ip, ir)

    # both matchers read the same input: the Python interpolation
    import mapmatching
    mp, mr = MapMatcher(net, engine="python"), MapMatcher(net, engine="rust")
    ties = []
    mapmatching.min = tie_min(net, ties)
    try:
        okp, ep = run(lambda: mp.match(ip.copy()))
    finally:
        del mapmatching.min
    out["_ties"] = len(ties)
    out["_failed"] = bool(ep) or not okp
    okr, er = run(lambda: mr.match(ip.copy()))
    if mp.timed_out or mr.timed_out:
        # the Rust engine goes further in the same time: the parts of the path are not the same
        out["_timeout"] = True
    elif ep or er or okp != okr:
        out["match"] = f"result {okp!r}/{ep!r} != {okr!r}/{er!r}" if (ep, okp) != (er, okr) else ""
    elif okp:
        out["match"] = frames_equal(mp.matchdf, mr.matchdf)
        pp = [(i["edge"].getID(), i["reverse"]) for i in mp.path]
        pr = [(i["edge"].getID(), i["reverse"]) for i in mr.path]
        if not out["match"] and pp != pr:
            out["match"] = "path"
        out["_points"] = len(ip)
        num = [c for c in mp.matchdf.columns if c not in TEXT_COLS and mp.matchdf[c].dtype != object]
        out["_maxdiff"] = float((mp.matchdf[num] - mr.matchdf[num]).abs().to_numpy().max())
    return {k: v for k, v in out.items() if v}


@pytest.fixture(scope="module")
def data():
    return load()


def test_engines_equal(data, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    net, gps = data
    bad = {}
    for t, s in trips(gps, 40):
        d = compare_trip(net, s)
        d = {k: v for k, v in d.items() if not k.startswith("_")}
        if d:
            bad[t] = d
    assert not bad, bad


def test_push_equals_match(data, tmp_path, monkeypatch):
    """The online use (one point at a time) gives the matches of the full trace."""
    monkeypatch.chdir(tmp_path)
    from cleandata import cleaningData
    from interpolation import interpolateTrajectory
    from mapmatching import MapMatcher, rust_network
    net, gps = data
    for t, s in trips(gps, 10):
        traj = interpolateTrajectory(cleaningData(s, net)).reset_index(drop=True)
        m = MapMatcher(net, engine="rust")
        if not run(lambda: m.match(traj))[0]:
            continue
        online = engine.rust.Matcher(rust_network(net), 60, 40, True, False, True, 10)
        for r in traj.itertuples():
            online.push(r.x, r.y, r.speed, r.bearing)
        ids = net._edgeidlist
        assert [ids[e] for e in online.result()["edge"]] == m.matchdf["edgeid"].tolist(), t


# ---------------------------------------------------------------------------------------------
# The faults of the old matcher (before 2026-11-21). Each test runs both engines.
# ---------------------------------------------------------------------------------------------

def trip_points(data, trip):
    from cleandata import cleaningData
    from interpolation import interpolateTrajectory
    net, gps = data
    s = gps[gps["trip_id_extended"] == trip].sort_values("timestamp").drop_duplicates()
    return interpolateTrajectory(cleaningData(s, net, engine="python"), engine="python")


@pytest.mark.parametrize("eng", ["python", "rust"])
def test_fault1_step_back_empties_the_path(data, tmp_path, monkeypatch, eng):
    """The old matcher raised IndexError 'pop from empty list' on this trip."""
    monkeypatch.chdir(tmp_path)
    from mapmatching import MapMatcher
    m = MapMatcher(data[0], engine=eng)
    assert m.match(trip_points(data, "14010000510061335_0")) == 1
    assert len(m.matchdf) > 0 and len(m.path) > 0


@pytest.mark.parametrize("eng", ["python", "rust"])
def test_fault1_no_decision_point_left(data, tmp_path, monkeypatch, eng):
    """The old matcher returned 0 on this trip ('decision list is empty')."""
    monkeypatch.chdir(tmp_path)
    from mapmatching import MapMatcher
    m = MapMatcher(data[0], engine=eng)
    assert m.match(trip_points(data, "14010000510073977_0")) == 1
    assert len(m.matchdf) > 0 and len(m.path) > 0


@pytest.mark.parametrize("eng", ["python", "rust"])
def test_fault2_time_limit(data, tmp_path, monkeypatch, eng):
    """The old matcher had a fixed 10 s limit and returned 0 there. Now the limit is MAX_RUNNING_TIME,
    and at the limit match returns 1 with the path found so far."""
    monkeypatch.chdir(tmp_path)
    from mapmatching import MapMatcher
    m = MapMatcher(data[0], MAX_RUNNING_TIME=0, engine=eng)
    assert m.match(trip_points(data, "14010000510073977_0")) == 1   # a trip with step backs
    assert m.timed_out and len(m.matchdf) > 0


@pytest.mark.parametrize("eng", ["python", "rust"])
def test_fault3_no_candidate_edge(data, tmp_path, monkeypatch, eng):
    """The old matcher raised ValueError 'ln(edges)==0' when no edge is near a point.
    Now it skips the point (a gap) and matches the other points."""
    monkeypatch.chdir(tmp_path)
    from mapmatching import MapMatcher
    pts = trip_points(data, "14010000510059476_0")
    pts.loc[pts.index[0], "x"] += 1000.0          # the first point is 1 km from all edges
    m = MapMatcher(data[0], engine=eng)
    assert m.match(pts) == 1
    assert pts.index[0] not in set(m.matchdf["index"]) and len(m.matchdf) == len(pts) - 1


def test_engines_equal_after_fixes(data, tmp_path, monkeypatch):
    """The trips of the faults give the same matches in both engines."""
    monkeypatch.chdir(tmp_path)
    net, gps = data
    for t in ["14010000510061335_0", "14010000510073977_0", "14010000526371002_0", "14010000526351621_0"]:
        s = gps[gps["trip_id_extended"] == t].sort_values("timestamp").drop_duplicates()
        d = {k: v for k, v in compare_trip(net, s).items() if not k.startswith("_")}
        assert not d, (t, d)


def test_reset_quiet_no_file(data, tmp_path, monkeypatch, capsys):
    """reset() clears the path; match() writes no file and prints nothing (only with debug=True)."""
    monkeypatch.chdir(tmp_path)
    from mapmatching import MapMatcher
    m = MapMatcher(data[0], engine="python")
    m.match(trip_points(data, "14010000510061335_0"))
    assert not os.listdir(tmp_path) and capsys.readouterr().out == ""
    m.reset()
    assert m.path == [] and m.matchdf is None


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 100
    net, gps = load()
    os.chdir(tempfile.mkdtemp())
    stats = {"trips": 0, "points": 0, "trips with a tie": 0, "match fails (both engines)": 0,
             "timed out (not compared)": 0, "max difference": 0.0}
    diffs = {}
    for t, s in trips(gps, n):
        d = compare_trip(net, s)
        stats["trips"] += 1
        stats["points"] += d.pop("_points", 0)
        stats["trips with a tie"] += d.pop("_ties", 0) > 0
        stats["match fails (both engines)"] += d.pop("_failed", False)
        stats["timed out (not compared)"] += d.pop("_timeout", False)
        stats["max difference"] = max(stats["max difference"], d.pop("_maxdiff", 0.0))
        for k, v in d.items():
            diffs.setdefault(k, []).append((t, v))
    print(stats)
    for k, v in diffs.items():
        print(f"{k}: {len(v)} trips differ")
        for t, msg in v[:5]:
            print("   ", t, msg)
    if not diffs:
        print("all stages equal")
