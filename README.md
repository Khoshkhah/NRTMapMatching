# Near Real-time Map Matching (NRTMapMatching)

A Python library for matching GPS trajectory data to road network edges in near real-time. The system processes GPS observations, cleans and enriches the data, interpolates trajectories using Bezier curves, and matches them to road networks using an advanced map matching algorithm.

## Features

- **Road Network Support**: Import networks from SUMO .net files or OpenStreetMap (OSM) files
- **GPS Data Cleaning**: Remove outliers and smooth trajectories
- **Data Enrichment**: Enhance GPS data with speed and bearing calculations
- **Trajectory Interpolation**: Interpolate trajectories using cubic Bezier curves for smooth paths
- **Map Matching**: Match GPS points to road network edges using a multi-factor cost function
- **Spatial Indexing**: Efficient edge queries using R-tree spatial indexing
- **Stop Detection**: Automatic detection and handling of vehicle stops

## Installation

### Prerequisites

- Python 3.7+
- Required Python packages (see `requirements.txt`)

### Install Dependencies

```bash
pip install -r requirements.txt
```

**Note**: The `requirements.txt` includes `zipfile`, which is a built-in Python module and doesn't need to be installed separately.

## Quick Start

### 1. Load a Road Network

```python
import sumolib
import network

# Load SUMO network
net1 = sumolib.net.readNet("data/iland.net", withInternal=False)
mynet = network.Net()
mynet.importFromSumoNet(net1)

# Or load OSM network
# mynet.importFromOSM("data/network.osm")
```

### 2. Load GPS Data

```python
import pandas as pd

gps = pd.read_csv("gps_data.csv")
# Required columns: lon, lat, timestamp, speed, bearing
```

### 3. Clean and Enrich Data

```python
from cleandata import richdata

cleaned = richdata(gps, mynet, id=0, alpha=0.7)
```

### 4. Interpolate Trajectory

```python
from interpolation import interpolateTrajectory

interpolated = interpolateTrajectory(cleaned, sample_rate=1)
```

### 5. Perform Map Matching

```python
from mapmatching import MapMatcher

# Create matcher
matcher = MapMatcher(mynet, MAX_GPS_ERROR=60, MAX_MAP_ERROR=40)

# Match trajectory
result = matcher.match(interpolated)

if result == 1:
    # Save point matching results
    point_match = matcher.save_pointmatch("point_match.csv")
    
    # Save route matching results
    route_match = matcher.save_routematch("route_match.csv")
else:
    print("Map matching failed")
```

## Project Structure

```
NRTMapMatching/
├── sources/
│   ├── network.py          # Road network loading and management
│   ├── cleandata.py        # GPS data cleaning and enrichment
│   ├── interpolation.py    # Trajectory interpolation using Bezier curves
│   ├── mapmatching.py      # Core map matching algorithm
│   ├── geotools.py         # Geographic utility functions
│   └── util.py             # Logging utilities
├── test/
│   ├── alltest.ipynb       # Comprehensive test notebook
│   ├── testmapmatching.ipynb
│   ├── testcleeningdata.ipynb
│   ├── testenrichdata.ipynb
│   └── testbezier.ipynb
├── docs/                   # Documentation
│   ├── index.md           # Documentation index
│   ├── overview.md        # Overview and architecture
│   ├── installation.md    # Installation guide
│   ├── user-guide.md      # Usage guide
│   ├── api-reference.md   # API documentation
│   ├── data-formats.md    # Data format specifications
│   ├── algorithms.md      # Algorithm details
│   └── configuration.md   # Configuration parameters
├── data/                   # Sample data files
├── requirements.txt        # Python dependencies
└── README.md              # This file
```

## Input Data Format

GPS data must be a pandas DataFrame with the following columns:

- `lon`: Longitude (degrees)
- `lat`: Latitude (degrees)
- `timestamp`: Unix timestamp (seconds, integer)
- `speed`: Speed (m/s, float)
- `bearing`: Bearing angle (degrees, 0-360, integer)

Example:
```python
import pandas as pd

gps_data = pd.DataFrame({
    'lon': [24.7536, 24.7537, 24.7538],
    'lat': [59.4370, 59.4371, 59.4372],
    'timestamp': [1609459200, 1609459201, 1609459202],
    'speed': [10.5, 11.2, 10.8],
    'bearing': [90, 92, 91]
})
```

## Output Data Format

### Point Matching Results

Contains matched GPS points with the following key columns:
- `x`, `y`: Matched coordinates on road network
- `x_sample`, `y_sample`: Original GPS coordinates
- `edgeid`: Matched edge ID
- `offset`: Offset along edge (meters)
- `dist`: Distance from GPS to matched point (meters)
- `speed`: Speed at matched point
- `timestamp`: Timestamp

### Route Matching Results

Contains edge sequences with:
- `from_edge`: Source edge ID
- `edgeid`: Current edge ID
- `departure`: Entry timestamp
- `arrival`: Exit timestamp
- `travel_time`: Time spent on edge (seconds)
- `stop_time`: Number of stopped points
- `shape`: LineString geometry of the edge

## Configuration Parameters

### MapMatcher Parameters

- `MAX_GPS_ERROR`: Maximum GPS error in meters (default: 60)
- `MAX_MAP_ERROR`: Maximum map error in meters (default: 40)
- `MAP_ONE_WAY_FIX`: Allow matching to edges in reverse direction (default: True)
- `U_TURN_ON_ONEWAY`: Allow U-turns on one-way streets (default: False)
- `LOOP`: Allow revisiting edges (default: True)
- `MAX_SPEED`: Maximum speed for validation in m/s (default: 100)
- `DIFF_GPS_ERROR`: GPS error difference threshold in meters (default: 10)
- `MAX_RUNNING_TIME`: Maximum running time in seconds (default: 5)

### Data Cleaning Parameters

- `alpha`: Smoothing parameter for exponential moving average (0-1, default: 0.7)
- `MIN_SPEED`: Minimum speed threshold in m/s (default: 1)
- `MAX_SPEED_FOR_OUTLIER`: Maximum speed for outlier detection in m/s (default: 50)
- `MINSPEED_FOR_BEARING`: Minimum speed for bearing calculation in m/s (default: 2)

## Algorithm Overview

### Map Matching Algorithm

The map matching algorithm uses a greedy approach with backtracking:

1. **Initial Matching**: Finds candidate edges within search radius for the first GPS point
2. **Iterative Matching**: For each GPS point:
   - Determines if vehicle should stay on current edge or change
   - Evaluates candidate edges using a multi-factor cost function
   - Selects best edge based on:
     - Geometric distance to edge
     - Bearing alignment
     - Speed consistency
     - Road network topology
3. **Backtracking**: If match quality is poor, algorithm backtracks to previous decision point

### Bezier Interpolation

- Uses cubic Bezier curves for smooth trajectory interpolation
- Control points positioned based on velocity vectors
- Ensures smooth transitions and realistic speeds
- Handles stops by maintaining constant position

## Examples

See the `test/` directory for comprehensive examples:
- `alltest.ipynb`: Complete workflow example
- `testmapmatching.ipynb`: Map matching examples
- `testcleeningdata.ipynb`: Data cleaning examples
- `testenrichdata.ipynb`: Data enrichment examples
- `testbezier.ipynb`: Bezier interpolation examples

## Dependencies

- `pandas`: Data manipulation
- `numpy`: Numerical computations
- `sumolib`: SUMO network file reading
- `shapely`: Geometric operations
- `rtree`: Spatial indexing
- `pyproj`: Coordinate system transformations
- `osmnx`: OpenStreetMap network processing (for OSM import)
- `pyrosm`: OSM file reading (for OSM import)

## Limitations

1. Requires road network data (SUMO .net or OSM file)
2. GPS data must have speed and bearing information
3. Performance depends on network size and GPS point density
4. May struggle with complex intersections or GPS errors > 100m
5. One-way street handling may need adjustment for specific use cases

## Performance Considerations

- Uses R-tree spatial indexing for efficient edge queries
- Search radius is adjustable based on GPS accuracy
- Backtracking is limited to prevent infinite loops
- Configurable maximum running time

## Documentation

📖 **Full documentation is available at: [https://YOUR_USERNAME.github.io/NRTMapMatching](https://khoshkhah.github.io/NRTMapMatching)**

The documentation is also available in the [`docs/`](docs/) folder:

- **[Overview](docs/overview.md)** - Introduction and architecture
- **[Installation Guide](docs/installation.md)** - Setup and dependencies  
- **[User Guide](docs/user-guide.md)** - Usage examples and workflows
- **[API Reference](docs/api-reference.md)** - Complete API documentation
- **[Data Formats](docs/data-formats.md)** - Input/output specifications
- **[Algorithms](docs/algorithms.md)** - Algorithm explanations
- **[Configuration](docs/configuration.md)** - Parameters and settings


## License

[Add your license information here]

## Contributing

[Add contribution guidelines here]

## Citation

If you use this library in your research, please cite:

```
[Add citation information]
```

## Support

For issues and questions, please open an issue on the project repository.

## Authors

[Add author information]
