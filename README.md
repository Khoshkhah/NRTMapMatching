# Near Real-time Map Matching (NRTMapMatching)

[![Documentation](https://img.shields.io/badge/docs-latest-blue)](https://khoshkhah.github.io/NRTMapMatching)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

A Python library for matching GPS trajectory data to road network edges in near real-time. The system processes GPS observations, cleans and enriches the data, interpolates trajectories using Bezier curves, and matches them to road networks using an advanced map matching algorithm.

## Features

-  **Road Network Support**: Import networks from SUMO .net files or OpenStreetMap (OSM) files
-  **GPS Data Cleaning**: Remove outliers and smooth trajectories
-  **Data Enrichment**: Enhance GPS data with speed and bearing calculations
-  **Trajectory Interpolation**: Interpolate trajectories using cubic Bezier curves
-  **Map Matching**: Match GPS points to road network edges using multi-factor cost function
-  **Spatial Indexing**: Efficient edge queries using R-tree spatial indexing
-  **Stop Detection**: Automatic detection and handling of vehicle stops

## Quick Start

### Installation

```bash
pip install -r requirements.txt
```

### Basic Usage

```python
import sumolib
import network
from cleandata import richdata
from interpolation import interpolateTrajectory
from mapmatching import MapMatcher
import pandas as pd

# 1. Load network
net1 = sumolib.net.readNet("data/network.net", withInternal=False)
mynet = network.Net()
mynet.importFromSumoNet(net1)

# 2. Load GPS data (requires: lon, lat, timestamp, speed, bearing)
gps = pd.read_csv("gps_data.csv")

# 3. Clean and enrich
cleaned = richdata(gps, mynet, id=0)

# 4. Interpolate
interpolated = interpolateTrajectory(cleaned, sample_rate=1)

# 5. Map match
matcher = MapMatcher(mynet)
if matcher.match(interpolated) == 1:
    point_match = matcher.save_pointmatch("point_match.csv")
    route_match = matcher.save_routematch("route_match.csv")
```

## Rust engine

The crate `rust/nrt-matcher` is a Rust version of the map matching: `cleaningData`, `richdata`, `interpolateTrajectory`, the R-tree edge search and `MapMatcher.match`. The network model, the projection (pyproj), the file input and output, and `save_routematch` stay in Python.

Build and install it into your Python environment (Rust and Python 3.9 or later are necessary):

```bash
pip install ./rust/nrt-matcher
```

The Python API does not change. When the module `nrt_matcher` is installed, the functions use it. Otherwise they use the Python code. To select the engine, set the argument `engine` (`"auto"`, `"rust"` or `"python"`) of these functions and of `MapMatcher`, or set the environment variable `NRT_ENGINE`.

For online use, `nrt_matcher.Matcher.push(x, y, speed, bearing)` matches one new point (see `test/test_rust_engine.py`). It gives the same result as the full trace.

### The matcher: a beam search (since 2026-11-21)

`MapMatcher.match` keeps at most `BEAM_WIDTH` hypotheses (default 20). A hypothesis is a path, its last matched point and its total cost. For each point, the matcher extends each hypothesis to its candidate edges: stay on the edge, change to an outgoing edge, or both. The speed and the rest of the edge select the candidates. The cost of a candidate is the cost of the old matcher (bearing, distance, air distance, road distance, reverse direction). The matcher does not use a candidate that is farther than `MAX_GPS_ERROR + MAX_MAP_ERROR` from the point. The `BEAM_WIDTH` children with the lowest total cost stay, at most one for each edge and direction. If no hypothesis can continue, the match starts again from the edges near the point. If no edge is near the point, the matcher skips the point. The result is the hypothesis with the lowest total cost.

The old matcher kept a full tree of decision points and stepped back when a point was too far from the edge. It had these faults, which the beam search does not have (one test for each fault in `test/test_rust_engine.py`):

- A step back could remove the full path and then raise `IndexError: pop from empty list`.
- The path could get more edges after each step back, up to hundreds of thousands of edges on a long trace.
- The search had a fixed 10 s limit, and at the limit `match` returned 0. The beam search does a fixed maximum of work for each point, so it has no time limit (`MAX_RUNNING_TIME` is not used).
- A point with no candidate edge raised `ValueError: ... ln(edges)==0`.
- At an exact cost tie, the old matcher took the first edge in a Python set, so the result could change from one run to the next. The beam search uses the sequence of `Net._edgeidlist`.

Also, `reset()` clears the path, and `match()` writes `new_sample.csv` and prints the path only with `MapMatcher(..., debug=True)`.

**Equal results.** `test/test_rust_engine.py` runs both engines on trips of `data/gps_inside_island.zip`. On 400 trips (304,743 matched points), all four stages give the same rows. Each point gets the same edge, and the largest difference in a number is 4e-12. Each point of each trip has a match.

### Road interpolation (since 2026-11-21)

`interpolateTrajectory(traj, net=net)` puts the points of a long gap (`ROAD_MIN_GAP`, default 6 s or more) along the shortest road path between the two GPS points, at constant speed. The path starts and ends on one of the 3 nearest edges (within 50 m) of each GPS point. If the path is longer than `ROAD_DETOUR_FACTOR` (2) × the straight distance + `ROAD_DETOUR_M` (100 m), for example because of GPS noise, or if no path is found, the gap gets the Bezier curve. Shorter gaps and stops also get the Bezier curve. Without `net`, the function uses only the Bezier curve, as before.

The Bezier curve cannot know where the road turns between two GPS points that are far apart. On the Monaco traces at 30 s, with GPS noise of only 3 m, the Bezier points are 28 m from the true route on average, and 52 % are more than 20 m away. With the road interpolation, the error at 15 s goes down from 18.6 m to 8.8 m, and at 30 s from 28.5 m to 17.9 m.

`interpolateTrajectory` also adds the column `confidence`: 1 at a GPS point, down to 0 in the middle of a gap. `MapMatcher(INTERP_WEIGHT=...)` multiplies the bearing, distance and air distance terms of a point by `INTERP_WEIGHT + (1 - INTERP_WEIGHT) × confidence`. The default 1 gives all points the same weight. On Monaco, no value and no other shape of this weight (square, step, a higher weight for the GPS points) changed the score by more than 0.003, so it stays off.

The cost weights are parameters: `W_BEARING` (1), `W_DIST` (30), `W_AIR` (10), `W_ROAD` (5), `W_REVERSE` (100000). A search of 1000 weight sets on half of the Monaco routes did not give better scores on the other half. The beam width had the effect: 20 is better than 5, and 50 or 100 give no more gain.

**Monaco test** (600 simulated traces on 50 routes, intervals 1, 5, 15 and 30 s, GPS noise 3, 10 and 20 m; both engines give the same path on all 600 traces). "Old" is the backtracking matcher; "new" is the beam search with `BEAM_WIDTH=20` and the road interpolation:

| Interval | Recall, old → new | Precision, old → new | No route, old → new |
|---|---|---|---|
| 1 s | 0.28 → 0.97 | 0.33 → 0.92 | 61 % → 0 % |
| 5 s | 0.23 → 0.94 | 0.26 → 0.94 | 68 % → 0 % |
| 15 s | 0.20 → 0.86 | 0.22 → 0.84 | 71 % → 0 % |
| 30 s | 0.06 → 0.69 | 0.07 → 0.69 | 87 % → 0 % |
| All | 0.19 → 0.87 | 0.22 → 0.85 | 72 % → 0 % |

**Speed** (`python test/bench_rust_engine.py 200`, Apple M6, 200 trips, 143,826 points at 1 s, `BEAM_WIDTH=20`):

| Stage | Python, ms per trip | Rust, ms per trip | Faster by |
|---|---:|---:|---:|
| `cleaningData` | 6.2 | 0.87 | 7× |
| `richdata` | 15.7 | 2.0 | 8× |
| `interpolateTrajectory` (Bezier) | 38.8 | 0.67 | 58× |
| `MapMatcher.match` | 264 | 2.2 | 123× |
| Rust matcher only (`Matcher.match_trace`) | | 1.2 | 226× |

The time for each new point: Python 471 µs (the trace time divided by the points), Rust `push` 3.5 µs (median), 10 µs (p99), 128 µs (maximum). In `cleaningData` and `richdata`, most of the Rust time is the pandas steps and the projection, which stay in Python.

## Documentation

**Full documentation available at: [https://khoshkhah.github.io/NRTMapMatching](https://khoshkhah.github.io/NRTMapMatching)**

The documentation includes:
- [Installation Guide](docs/installation.md)
- [User Guide](docs/user-guide.md) with detailed examples
- [API Reference](docs/api-reference.md)
- [Configuration](docs/configuration.md) parameters
- [Algorithms](docs/algorithms.md) explanation
- [Data Formats](docs/data-formats.md) specification

## Requirements

- Python 3.7+
- pandas, numpy, sumolib, shapely, rtree, pyproj
- osmnx, pyrosm (for OSM import)

See `requirements.txt` for complete list.

## Project Structure

```
NRTMapMatching/
├── sources/          # Core modules
├── rust/nrt-matcher/ # Rust engine of the map matching (Python module nrt_matcher)
├── docs/            # Documentation
├── test/            # Example notebooks
└── data/            # Sample data
```

## Examples

See the `test/` directory for Jupyter notebooks with complete examples:
- `alltest.ipynb` - Complete workflow
- `testmapmatching.ipynb` - Map matching examples
- `testcleeningdata.ipynb` - Data cleaning examples
- `testbezier.ipynb` - Interpolation examples

## Citation

If you use this library in your research, please cite:

```bibtex
@software{nrtmapmatching,
  title = {NRTMapMatching: Near Real-time Map Matching},
  author = {Kaveh Khoshkhah},
  year = {2023},
  url = {https://github.com/khoshkhah/NRTMapMatching}
}
```


