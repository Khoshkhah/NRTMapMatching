"""Measure the speed of the two engines on trips of data/gps_inside_island.zip.

    python test/bench_rust_engine.py [number of trips]

For each stage: the time for each trip. For the matcher also the time for each new point:
the Rust engine has an online use (`nrt_matcher.Matcher.push`, one point at a time); the
Python code matches only a full trace, so its number is the trace time divided by the points.
"""
import contextlib
import io
import os
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_rust_engine as T  # noqa: E402  (load, trips and the sources/ path)
import engine  # noqa: E402
from cleandata import cleaningData, richdata  # noqa: E402
from interpolation import interpolateTrajectory  # noqa: E402
from mapmatching import MapMatcher, rust_network  # noqa: E402


def timed(f):
    with contextlib.redirect_stdout(io.StringIO()):
        t = time.perf_counter()
        r = f()
        return time.perf_counter() - t, r


def main(n):
    net, gps = T.load()
    os.chdir(tempfile.mkdtemp())
    rnet = rust_network(net)
    ms = {k: [] for k in ("clean", "enrich", "interpolate", "match", "core")}
    per_point = {"python": [], "rust": []}
    push_us = []
    points = 0
    for _t, s in T.trips(gps, n):
        try:
            row = {}
            for eng in ("python", "rust"):
                row[("clean", eng)], c = timed(lambda: cleaningData(s.copy(), net, engine=eng))
                row[("enrich", eng)], _ = timed(lambda: richdata(s.copy(), net, engine=eng))
                row[("interpolate", eng)], ip = timed(lambda: interpolateTrajectory(c, engine=eng))
                m = MapMatcher(net, engine=eng)
                row[("match", eng)], ok = timed(lambda: m.match(ip))
                if not ok:
                    raise RuntimeError
        except Exception:  # the matcher fails on this trip (both engines): skip it
            continue
        tr = ip.reset_index(drop=True)
        cols = [tr[c].astype(float).tolist() for c in ("x", "y", "speed", "bearing")]
        core = engine.rust.Matcher(rnet, 60, 40, True, False, True, 10)
        t = time.perf_counter()
        core.match_trace(tr.index.tolist(), *cols)
        row[("core", "rust")] = time.perf_counter() - t
        online = engine.rust.Matcher(rnet, 60, 40, True, False, True, 10)
        for x, y, sp, b in zip(*cols):
            t = time.perf_counter_ns()
            online.push(x, y, sp, b)
            push_us.append((time.perf_counter_ns() - t) / 1000)
        for k in ms:
            ms[k].append((row.get((k, "python"), np.nan) * 1000, row[(k, "rust")] * 1000))
        per_point["python"].append(row[("match", "python")] / len(ip) * 1e6)
        per_point["rust"].append(row[("core", "rust")] / len(ip) * 1e6)
        points += len(ip)

    print(f"{len(ms['match'])} trips, {points} points to match (sample rate 1 s)\n")
    print("| stage | Python, ms/trip | Rust, ms/trip | Rust is faster by |")
    print("|---|---:|---:|---:|")
    names = {"clean": "cleaningData", "enrich": "richdata", "interpolate": "interpolateTrajectory",
             "match": "MapMatcher.match", "core": "Rust core only (match_trace)"}
    for k, v in ms.items():
        a = np.array(v)
        py, rs = np.nanmedian(a[:, 0]), np.median(a[:, 1])
        pys = "-" if np.isnan(py) else f"{py:.2f}"
        print(f"| {names[k]} | {pys} | {rs:.3f} | " + ("-" if np.isnan(py) else f"{py / rs:.0f}x") + " |")
    p = np.array(push_us)
    print(f"\nTime for each new point, median of trips: Python {np.median(per_point['python']):.1f} us "
          f"(trace time / points), Rust core {np.median(per_point['rust']):.2f} us")
    print(f"Rust push (one point, online): median {np.median(p):.2f} us, p99 {np.percentile(p, 99):.1f} us, "
          f"max {p.max():.0f} us over {len(p)} pushes")


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 100)
