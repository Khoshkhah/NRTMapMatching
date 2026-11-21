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

`MapMatcher.match` keeps at most `BEAM_WIDTH` hypotheses (default 5). A hypothesis is a path, its last matched point and its total cost. For each point, the matcher extends each hypothesis to its candidate edges: stay on the edge, change to an outgoing edge, or both. The speed and the rest of the edge select the candidates. The cost of a candidate is the cost of the old matcher (bearing, distance, air distance, road distance, reverse direction). The matcher does not use a candidate that is farther than `MAX_GPS_ERROR + MAX_MAP_ERROR` from the point. The `BEAM_WIDTH` children with the lowest total cost stay, at most one for each edge and direction. If no hypothesis can continue, the match starts again from the edges near the point. If no edge is near the point, the matcher skips the point. The result is the hypothesis with the lowest total cost.

The old matcher kept a full tree of decision points and stepped back when a point was too far from the edge. It had these faults, which the beam search does not have (one test for each fault in `test/test_rust_engine.py`):

- A step back could remove the full path and then raise `IndexError: pop from empty list`.
- The path could get more edges after each step back, up to hundreds of thousands of edges on a long trace.
- The search had a fixed 10 s limit, and at the limit `match` returned 0. The beam search does a fixed maximum of work for each point, so it has no time limit (`MAX_RUNNING_TIME` is not used).
- A point with no candidate edge raised `ValueError: ... ln(edges)==0`.
- At an exact cost tie, the old matcher took the first edge in a Python set, so the result could change from one run to the next. The beam search uses the sequence of `Net._edgeidlist`.

Also, `reset()` clears the path, and `match()` writes `new_sample.csv` and prints the path only with `MapMatcher(..., debug=True)`.

**Equal results.** `test/test_rust_engine.py` runs both engines on trips of `data/gps_inside_island.zip`. On 400 trips (304,743 matched points), all four stages give the same rows. Each point gets the same edge, and the largest difference in a number is 4e-12. Each point of each trip has a match.

**Monaco test** (600 simulated traces on 50 routes, intervals 1, 5, 15 and 30 s, GPS noise 3, 10 and 20 m; both engines give the same path on all 600 traces):

| Interval | Recall, old → beam | Precision, old → beam | No route, old → beam |
|---|---|---|---|
| 1 s | 0.28 → 0.95 | 0.33 → 0.91 | 61 % → 0 % |
| 5 s | 0.23 → 0.89 | 0.26 → 0.90 | 68 % → 0 % |
| 15 s | 0.20 → 0.65 | 0.22 → 0.74 | 71 % → 0 % |
| 30 s | 0.06 → 0.42 | 0.07 → 0.56 | 87 % → 0 % |

The matcher moves at most one edge for each interpolated 1 s point, so its scores are lower at long intervals.

**Speed** (`python test/bench_rust_engine.py 200`, Apple M6, 200 trips, 143,826 points at 1 s):

| Stage | Python, ms per trip | Rust, ms per trip | Faster by |
|---|---:|---:|---:|
| `cleaningData` | 6.0 | 0.81 | 7× |
| `richdata` | 15.6 | 2.0 | 8× |
| `interpolateTrajectory` | 36.6 | 0.53 | 69× |
| `MapMatcher.match` | 108.0 | 1.3 | 81× |
| Rust matcher only (`Matcher.match_trace`) | | 0.42 | 256× |

The time for each new point: Python 181 µs (the trace time divided by the points), Rust `push` 1.7 µs (median), 7.5 µs (p99), 100 µs (maximum). In `cleaningData` and `richdata`, most of the Rust time is the pandas steps and the projection, which stay in Python.

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


