# NRTMapMatching - Near Real-time Map Matching Documentation

## Overview

NRTMapMatching is a Python library for matching GPS trajectory data to road network edges in near real-time. The system processes GPS observations, cleans and enriches the data, interpolates trajectories, and matches them to a road network using an advanced map matching algorithm.

## Architecture

The system consists of several key components:

1. **Network Module** (`network.py`): Handles road network loading and management
2. **Data Cleaning Module** (`cleandata.py`): Cleans and enriches GPS observations
3. **Interpolation Module** (`interpolation.py`): Interpolates trajectories using Bezier curves
4. **Map Matching Module** (`mapmatching.py`): Core map matching algorithm
5. **Geographic Tools** (`geotools.py`): Utility functions for geographic calculations
6. **Utilities** (`util.py`): Logging and utility functions

## Module Details

### 1. Network Module (`network.py`)

The network module provides classes for managing road networks.

#### Classes

##### `Node`
Represents a node (intersection) in the road network.

**Attributes:**
- `coord`: Tuple (x, y) representing node coordinates
- `id`: Unique identifier for the node
- `outgoing`: List of outgoing edges
- `incoming`: List of incoming edges

**Methods:**
- `addIncoming(e)`: Add an incoming edge
- `addOutgoing(e)`: Add an outgoing edge
- `getID()`: Get node ID
- `getIncoming()`: Get list of incoming edges
- `getOutgoing()`: Get list of outgoing edges
- `getCoord()`: Get node coordinates

##### `Edge`
Represents a road edge (segment) in the network.

**Attributes:**
- `id`: Unique identifier for the edge
- `fromnode`: Source node
- `tonode`: Target node
- `speed`: Speed limit (m/s)
- `length`: Edge length (meters)
- `shape`: List of points defining the edge geometry
- `incoming`: List of incoming edges
- `outgoing`: List of outgoing edges

**Methods:**
- `getID()`: Get edge ID
- `getShape()`: Get edge geometry
- `getSpeed()`: Get speed limit
- `getLength()`: Get edge length
- `getToNode()`: Get target node
- `getFromNode()`: Get source node
- `getBoundingBox()`: Get bounding box for spatial indexing

##### `Net`
Main network class that manages the entire road network.

**Attributes:**
- `edges`: Dictionary of all edges
- `nodes`: Dictionary of all nodes
- `geoproj`: PyProj projection object for coordinate conversion
- `_location`: Location metadata including projection parameters
- `_rtree`: R-tree spatial index for efficient edge queries

**Methods:**
- `importFromSumoNet(snet)`: Import network from SUMO .net file
- `importFromOSM(osmfile)`: Import network from OpenStreetMap file
- `getNeighboringEdges(x, y, r)`: Find edges within radius `r` of point (x, y)
- `convertLonLat2XY(lon, lat)`: Convert longitude/latitude to local coordinates
- `convertXY2LonLat(x, y)`: Convert local coordinates to longitude/latitude
- `getNode(n)`: Get node by ID
- `getEdge(n)`: Get edge by ID

**Key Features:**
- Supports SUMO network files and OSM files
- Automatic UTM zone calculation for OSM imports
- R-tree spatial indexing for fast edge queries
- Coordinate system conversion between lat/lon and local UTM coordinates

### 2. Data Cleaning Module (`cleandata.py`)

This module provides functions for cleaning and enriching GPS trajectory data.

#### Functions

##### `cleaningData(obs, net, id=0, MINSPEED_FOR_BEARING=1, MAX_SPEED_FOR_OUTLIER=100)`

Cleans GPS observation data by:
1. Validating required columns (lon, lat, timestamp, speed, bearing)
2. Converting lon/lat to x/y coordinates
3. Removing outliers based on maximum speed
4. Handling stops (consecutive points with zero speed)
5. Assigning stop indices

**Parameters:**
- `obs`: DataFrame with columns: lon, lat, timestamp, speed, bearing
- `net`: Network object for coordinate conversion
- `id`: Trajectory ID (default: 0)
- `MINSPEED_FOR_BEARING`: Minimum speed for bearing calculation (m/s)
- `MAX_SPEED_FOR_OUTLIER`: Maximum speed for outlier detection (m/s)

**Returns:**
- DataFrame with columns: id, x, y, timestamp, speed, bearing, stopindex

##### `richdata(obs, net, id=0, alpha=.7, type="local", MIN_SPEED=1, MAX_SPEED_FOR_OUTLIER=50, MINSPEED_FOR_BEARING=2)`

Enriches GPS data with additional processing:
1. Convert to x, y coordinates
2. Remove outliers
3. Smooth velocity using exponential weighted moving average
4. Calculate bearing and speed from smoothed trajectory
5. Assign zero speed for low speeds
6. Update bearings based on speed thresholds
7. Assign stop indices and fix locations for stops

**Parameters:**
- `obs`: DataFrame with GPS observations
- `net`: Network object
- `id`: Trajectory ID
- `alpha`: Smoothing parameter for exponential moving average (0-1)
- `type`: Smoothing type ("local" or "aggregated")
- `MIN_SPEED`: Minimum speed threshold (m/s)
- `MAX_SPEED_FOR_OUTLIER`: Maximum speed for outlier detection (m/s)
- `MINSPEED_FOR_BEARING`: Minimum speed for bearing calculation (m/s)

**Returns:**
- Enriched DataFrame with same columns as `cleaningData`

##### `smoothingPoint(obs, alpha=.7, type="local")`

Smooths GPS points using exponential weighted moving average of velocity.

**Parameters:**
- `obs`: DataFrame with x, y, timestamp columns
- `alpha`: Smoothing parameter (0-1, higher = less smoothing)
- `type`: Smoothing type ("local" or "aggregated")

**Returns:**
- DataFrame with smoothed x, y coordinates

##### `removeOutlier(obs, MAX_SPEED_FOR_OUTLIER=50)`

Removes GPS points that would require unrealistic speeds.

**Parameters:**
- `obs`: DataFrame with x, y, timestamp columns
- `MAX_SPEED_FOR_OUTLIER`: Maximum allowed speed (m/s)

**Returns:**
- Filtered DataFrame

##### `xystop_point_editing(df)`

Sets stop locations to the median of consecutive stop points.

**Parameters:**
- `df`: DataFrame with stopindex column

**Returns:**
- DataFrame with updated x, y coordinates for stops

### 3. Interpolation Module (`interpolation.py`)

This module provides trajectory interpolation using Bezier curves.

#### Functions

##### `interpolateTrajectory(traj, sample_rate=1)`

Interpolates a trajectory to create points at regular time intervals using Bezier curves.

**Parameters:**
- `traj`: DataFrame with columns: x, y, timestamp, speed, bearing, stopindex
- `sample_rate`: Time interval between interpolated points (seconds)

**Returns:**
- DataFrame with interpolated points including original and extra points

**Algorithm:**
- Uses cubic Bezier curves for moving segments
- Control points are calculated based on velocity and bearing
- Handles stops by maintaining constant position

##### `bezierInterpolation(a, b, sample_rate=1)`

Interpolates between two GPS points using a cubic Bezier curve.

**Parameters:**
- `a`: First point (dict with x, y, timestamp, speed, bearing, stopindex)
- `b`: Second point (dict with x, y, timestamp, speed, bearing, stopindex)
- `sample_rate`: Time interval between points (seconds)

**Returns:**
- DataFrame with interpolated points

**Algorithm:**
- Calculates control points based on velocity vectors
- Control point distance depends on speed and distance between points
- Uses derivative of Bezier curve to calculate bearing and speed

##### `Bezier_3(p0, p1, p2, p3, t)`

Evaluates a cubic Bezier curve at parameter t.

**Parameters:**
- `p0, p1, p2, p3`: Control points (numpy arrays)
- `t`: Parameter value (0-1)

**Returns:**
- Point on the curve

##### `derivative_Bezier_3(p0, p1, p2, p3, t)`

Calculates the derivative (velocity) of a cubic Bezier curve at parameter t.

### 4. Map Matching Module (`mapmatching.py`)

The core map matching algorithm that matches GPS points to road network edges.

#### Class: `MapMatcher`

Main class for performing map matching.

##### Initialization

```python
MapMatcher(net, MAX_GPS_ERROR=60, MAX_MAP_ERROR=40, 
           MAP_ONE_WAY_FIX=True, U_TURN_ON_ONEWAY=False,
           LOOP=True, MAX_SPEED=100, DIFF_GPS_ERROR=10,
           MAX_RUNNING_TIME=5)
```

**Parameters:**
- `net`: Network object
- `MAX_GPS_ERROR`: Maximum GPS error in meters (default: 60)
- `MAX_MAP_ERROR`: Maximum map error in meters (default: 40)
- `MAP_ONE_WAY_FIX`: Allow matching to edges in reverse direction (default: True)
- `U_TURN_ON_ONEWAY`: Allow U-turns on one-way streets (default: False)
- `LOOP`: Allow revisiting edges (default: True)
- `MAX_SPEED`: Maximum speed for validation (m/s, default: 100)
- `DIFF_GPS_ERROR`: GPS error difference threshold (meters, default: 10)
- `MAX_RUNNING_TIME`: Maximum running time in seconds (default: 5)

##### Methods

###### `match(sample_gps)`

Performs map matching on a GPS trajectory.

**Parameters:**
- `sample_gps`: DataFrame with columns: x, y, timestamp, speed, bearing, stopindex, type

**Returns:**
- 1 if successful, 0 if failed

**Algorithm:**
1. Initial point matching: Find candidate edges within search radius
2. For each GPS point:
   - Decide whether to stay on current edge or change to new edge
   - Calculate cost for each candidate edge based on:
     - Bearing error
     - Distance to edge
     - Air distance error
     - Road distance error
     - Reverse direction penalty
   - Select edge with minimum cost
   - Handle edge transitions and backtracking if needed

**Output:**
- Stores results in `self.matchdf` DataFrame

###### `save_routematch(routematchfile=None)`

Saves route matching results (edge sequences).

**Parameters:**
- `routematchfile`: Optional file path to save CSV

**Returns:**
- DataFrame with columns: from_edge, edgeid, edge_reverse, from_edge_reverse, departure, arrival, stop_time, travel_time, shape

###### `save_pointmatch(pointmatchfile=None)`

Saves point matching results (matched GPS points).

**Parameters:**
- `pointmatchfile`: Optional file path to save CSV

**Returns:**
- DataFrame with matched points including edge IDs, offsets, and match quality metrics

###### `reset()`

Resets the matcher state.

##### Cost Function

The cost function used for edge selection:

```python
cost = 1*bearing_error + 30*match_point_distance + 10*air_distance_error + 5*road_distance_error
```

If reverse direction: `cost += 100000`

##### Decision Making

The algorithm uses three decision types:
- **STAY**: Vehicle remains on current edge
- **CHANGE**: Vehicle transitions to a new edge
- **NODECISION**: Ambiguous case, considers both staying and changing

Decision is based on remaining edge length, current speed, and maximum speed.

### 5. Geographic Tools (`geotools.py`)

Utility functions for geographic calculations.

#### Key Functions

##### `distance2d(point1, point2)`
Calculates Euclidean distance between two 2D points.

##### `calculate_bearing_angle(point1, point2)`
Calculates bearing angle (0-360 degrees) from point1 to point2.

##### `distancePointToLine(point, line)`
Calculates distance from a point to a line segment.

##### `polygonOffsetWithMinimumDistanceToPoint(point, polygon)`
Finds the point on a polygon closest to a given point and returns the offset along the polygon.

##### `offsetBearing(polygon, offset)`
Calculates the bearing angle at a specific offset along a polygon.

##### `road_distance(currentedge, currentoffset, lastedge, lastoffset, lastedgelength)`
Calculates road network distance between two points on edges.

##### `calculateUTMZone(min_lon, max_lon, min_lat, max_lat)`
Calculates appropriate UTM zone for a bounding box.

##### `dfLonLat2XY(df, net)`
Converts DataFrame with lon/lat columns to x/y columns.

##### `dfPoint2LonLat(df, net)`
Converts DataFrame with x/y columns to lon/lat columns.

##### `polyLength(polygon)`
Calculates total length of a polyline.

### 6. Utilities (`util.py`)

Utility functions for logging.

#### Functions

##### `get_logger(name)`
Creates a logger with file handler.

**Parameters:**
- `name`: Logger name

**Returns:**
- Logger object

## Data Formats

### Input GPS Data Format

Required columns:
- `lon`: Longitude (degrees)
- `lat`: Latitude (degrees)
- `timestamp`: Unix timestamp (seconds)
- `speed`: Speed (m/s)
- `bearing`: Bearing angle (degrees, 0-360)

### Cleaned/Enriched Data Format

Columns:
- `id`: Trajectory ID
- `x`: X coordinate (meters, local projection)
- `y`: Y coordinate (meters, local projection)
- `timestamp`: Unix timestamp (seconds)
- `speed`: Speed (m/s)
- `bearing`: Bearing angle (degrees)
- `stopindex`: Stop index (0 = moving, >0 = stopped)

### Interpolated Data Format

Same as cleaned data, plus:
- `type`: "origin" (original point) or "extra" (interpolated point)

### Map Matching Output Format

Point matching (`matchdf`):
- `index`: Original GPS point index
- `timestamp`: Timestamp
- `x`, `y`: Matched coordinates
- `x_sample`, `y_sample`: Original GPS coordinates
- `edgeid`: Matched edge ID
- `offset`: Offset along edge (meters)
- `dist`: Distance from GPS to matched point (meters)
- `distbearing`: Bearing error (degrees)
- `rd`: Road distance error (meters)
- `cost_air`: Air distance error (meters)
- `matchbearing`: Bearing of matched edge segment
- `speed`: Speed
- `predict_distance`: Predicted distance based on speed
- `matched_road_distance`: Actual road distance
- `edge_reverse`: Whether edge is traversed in reverse
- `from_edge`: Previous edge ID
- `from_edge_reverse`: Whether previous edge was reversed

Route matching (`routedf`):
- `from_edge`: Source edge ID
- `edgeid`: Current edge ID
- `edge_reverse`: Whether edge is traversed in reverse
- `from_edge_reverse`: Whether source edge was reversed
- `departure`: Timestamp when entering edge
- `arrival`: Timestamp when leaving edge
- `stop_time`: Number of stopped points on edge
- `travel_time`: Total travel time on edge (seconds)
- `shape`: LineString geometry of the edge

## Usage Workflow

### Typical Workflow

1. **Load Network**
   ```python
   import sumolib
   import network
   
   net1 = sumolib.net.readNet("network.net", withInternal=False)
   mynet = network.Net()
   mynet.importFromSumoNet(net1)
   ```

2. **Load GPS Data**
   ```python
   import pandas as pd
   
   gps = pd.read_csv("gps_data.csv")
   ```

3. **Clean/Enrich Data**
   ```python
   from cleandata import richdata
   
   cleaned = richdata(gps, mynet, id=0, alpha=0.7)
   ```

4. **Interpolate Trajectory**
   ```python
   from interpolation import interpolateTrajectory
   
   interpolated = interpolateTrajectory(cleaned, sample_rate=1)
   ```

5. **Map Matching**
   ```python
   from mapmatching import MapMatcher
   
   matcher = MapMatcher(mynet)
   matcher.match(interpolated)
   
   # Save results
   point_match = matcher.save_pointmatch("point_match.csv")
   route_match = matcher.save_routematch("route_match.csv")
   ```

## Algorithm Details

### Map Matching Algorithm

The map matching algorithm uses a greedy approach with backtracking:

1. **Initial Matching**: Finds candidate edges within search radius for first GPS point
2. **Iterative Matching**: For each subsequent GPS point:
   - Determines if vehicle should stay on current edge or change
   - Evaluates candidate edges using cost function
   - Selects best edge
   - Handles edge transitions and backtracking on errors

3. **Backtracking**: If match quality is poor (distance > radius), algorithm backtracks to previous decision point

4. **Cost Calculation**: Multi-factor cost considering:
   - Geometric distance
   - Bearing alignment
   - Speed consistency
   - Road network topology

### Bezier Interpolation

The interpolation uses cubic Bezier curves:
- Control points positioned based on velocity vectors
- Distance from endpoints depends on speed
- Ensures smooth transitions and realistic speeds
- Handles stops by maintaining constant position

## Performance Considerations

- **Spatial Indexing**: Uses R-tree for efficient edge queries
- **Search Radius**: Adjustable based on GPS accuracy
- **Backtracking**: Limits backtracking to prevent infinite loops
- **Running Time**: Configurable maximum running time

## Limitations

1. Requires road network data (SUMO .net or OSM file)
2. GPS data must have speed and bearing information
3. Performance depends on network size and GPS point density
4. May struggle with complex intersections or GPS errors > 100m
5. One-way street handling may need adjustment for specific use cases

## Dependencies

- pandas: Data manipulation
- numpy: Numerical computations
- sumolib: SUMO network file reading
- shapely: Geometric operations
- rtree: Spatial indexing
- pyproj: Coordinate system transformations
- osmnx: OpenStreetMap network processing (for OSM import)
- pyrosm: OSM file reading (for OSM import)

## Error Handling

The system includes error handling for:
- Missing required columns in input data
- Invalid network files
- GPS points outside network bounds
- Map matching failures (returns 0)

## Future Improvements

Potential enhancements:
1. Parallel processing for multiple trajectories
2. Machine learning-based cost function
3. Better handling of GPS gaps
4. Support for additional network formats
5. Real-time streaming API
6. Visualization tools

