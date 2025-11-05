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

## Documentation

📖 **Full documentation available at: [https://khoshkhah.github.io/NRTMapMatching](https://khoshkhah.github.io/NRTMapMatching)**

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


