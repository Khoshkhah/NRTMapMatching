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

For online use, `nrt_matcher.Matcher.push(x, y, speed, bearing)` matches one new point and continues the match from the points before it (see `test/test_rust_engine.py`).

**Equal results.** `test/test_rust_engine.py` runs both engines on trips of `data/gps_inside_island.zip`. On 400 trips (282,656 matched points), all four stages give the same rows. Each point gets the same edge, and the largest difference in a number is 4e-12. The test does not compare the 17 trips that stop at `MAX_RUNNING_TIME`, because in the same time the Rust engine goes further than the Python code. At an exact cost tie (76 of the 400 trips), the Python code takes the first edge in a Python set, and that sequence can change from one run to the next. The Rust engine takes the edge with the lowest index. The test gives the Python code the same tie rule.

**Matcher faults fixed** (2026-11-21, the same in both engines; the test has one test for each fault):

- A step back that removed the full path raised `IndexError: pop from empty list`. Now an empty path is not popped. When no decision point is left, the match starts again at the current point (before, `match` returned 0).
- The time limit was a fixed 10 s, and at the limit `match` returned 0. Now the limit is `MAX_RUNNING_TIME` (default 5 s). At the limit, `match` returns 1 with the path found so far, and `timed_out` is `True`.
- A point with no candidate edge (a dead end, or no edge near the point) raised `ValueError: ... ln(edges)==0`. Now the match starts again at that point. If no edge is near the point, the matcher skips it (a gap).
- `reset()` did not clear the path. `match()` wrote `new_sample.csv` into the current folder and printed the path many times. Now it does this only with `MapMatcher(..., debug=True)`.

On a trace with no fault that takes less than 5 s, the result does not change.

**Speed** (`python test/bench_rust_engine.py 200`, Apple M6, 150 trips that match, 100,282 points at 1 s):

| Stage | Python, ms per trip | Rust, ms per trip | Faster by |
|---|---:|---:|---:|
| `cleaningData` | 5.7 | 0.74 | 8× |
| `richdata` | 14.3 | 1.8 | 8× |
| `interpolateTrajectory` | 35.1 | 0.52 | 68× |
| `MapMatcher.match` | 26.6 | 1.8 | 14× |
| Rust matcher only (`Matcher.match_trace`) | | 0.075 | 355× |

The time for each new point: Python 49 µs (the trace time divided by the points), Rust `push` 0.17 µs (median), 0.5 µs (p99). Most of the Rust time in `MapMatcher.match` is the file `new_sample.csv` that `match()` writes for each trace. In `cleaningData` and `richdata`, most of the Rust time is the pandas steps and the projection, which stay in Python.

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


