# API Reference

Complete API documentation for all modules and functions.

## Table of Contents

- [Network Module](#network-module)
- [Data Cleaning Module](#data-cleaning-module)
- [Interpolation Module](#interpolation-module)
- [Map Matching Module](#map-matching-module)
- [Geographic Tools](#geographic-tools)
- [Utilities](#utilities)

## Network Module

The network module (`network.py`) provides classes for managing road networks.

### Classes

#### `Node`

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

#### `Edge`

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

#### `Net`

Main network class that manages the entire road network.

**Attributes:**
- `edges`: Dictionary of all edges
- `nodes`: Dictionary of all nodes
- `geoproj`: PyProj projection object for coordinate conversion
- `_location`: Location metadata including projection parameters
- `_rtree`: R-tree spatial index for efficient edge queries

**Methods:**

##### `importFromSumoNet(snet)`

Import network from SUMO .net file.

**Parameters:**
- `snet`: SUMO network object from `sumolib.net.readNet()`

**Example:**
```python
import sumolib
net1 = sumolib.net.readNet("network.net", withInternal=False)
mynet = network.Net()
mynet.importFromSumoNet(net1)
```

##### `importFromOSM(osmfile)`

Import network from OpenStreetMap file.

**Parameters:**
- `osmfile`: Path to OSM file (.osm or .osm.pbf)

**Example:**
```python
mynet = network.Net()
mynet.importFromOSM("network.osm")
```

##### `getNeighboringEdges(x, y, r)`

Find edges within radius `r` of point (x, y).

**Parameters:**
- `x`: X coordinate
- `y`: Y coordinate
- `r`: Search radius in meters

**Returns:**
- List of tuples `(edge, distance)` for edges within radius

##### `convertLonLat2XY(lon, lat, rawUTM=False)`

Convert longitude/latitude to local coordinates.

**Parameters:**
- `lon`: Longitude (degrees)
- `lat`: Latitude (degrees)
- `rawUTM`: If True, return raw UTM coordinates without offset

**Returns:**
- Tuple (x, y) in local coordinate system

##### `convertXY2LonLat(x, y, rawUTM=False)`

Convert local coordinates to longitude/latitude.

**Parameters:**
- `x`: X coordinate
- `y`: Y coordinate
- `rawUTM`: If True, treat input as raw UTM coordinates

**Returns:**
- Tuple (lon, lat) in degrees

##### `getNode(n)`

Get node by ID.

**Parameters:**
- `n`: Node ID

**Returns:**
- Node object

##### `getEdge(n)`

Get edge by ID.

**Parameters:**
- `n`: Edge ID

**Returns:**
- Edge object

## Data Cleaning Module

The data cleaning module (`cleandata.py`) provides functions for cleaning and enriching GPS trajectory data.

### Functions

#### `cleaningData(obs, net, id=0, MINSPEED_FOR_BEARING=1, MAX_SPEED_FOR_OUTLIER=100)`

Cleans GPS observation data by validating required columns, converting coordinates, removing outliers, and handling stops.

**Parameters:**
- `obs`: DataFrame with columns: lon, lat, timestamp, speed, bearing
- `net`: Network object for coordinate conversion
- `id`: Trajectory ID (default: 0)
- `MINSPEED_FOR_BEARING`: Minimum speed for bearing calculation (m/s, default: 1)
- `MAX_SPEED_FOR_OUTLIER`: Maximum speed for outlier detection (m/s, default: 100)

**Returns:**
- DataFrame with columns: id, x, y, timestamp, speed, bearing, stopindex

**Raises:**
- Prints error message and returns 0 if required columns are missing

#### `richdata(obs, net, id=0, alpha=.7, type="local", MIN_SPEED=1, MAX_SPEED_FOR_OUTLIER=50, MINSPEED_FOR_BEARING=2)`

Enriches GPS data with additional processing including smoothing and outlier removal.

**Parameters:**
- `obs`: DataFrame with GPS observations
- `net`: Network object
- `id`: Trajectory ID (default: 0)
- `alpha`: Smoothing parameter for exponential moving average, 0-1 (default: 0.7)
- `type`: Smoothing type, "local" or "aggregated" (default: "local")
- `MIN_SPEED`: Minimum speed threshold (m/s, default: 1)
- `MAX_SPEED_FOR_OUTLIER`: Maximum speed for outlier detection (m/s, default: 50)
- `MINSPEED_FOR_BEARING`: Minimum speed for bearing calculation (m/s, default: 2)

**Returns:**
- Enriched DataFrame with columns: id, x, y, timestamp, speed, bearing, stopindex

#### `smoothingPoint(obs, alpha=.7, type="local")`

Smooths GPS points using exponential weighted moving average of velocity.

**Parameters:**
- `obs`: DataFrame with x, y, timestamp columns
- `alpha`: Smoothing parameter, 0-1, higher = less smoothing (default: 0.7)
- `type`: Smoothing type, "local" or "aggregated" (default: "local")

**Returns:**
- DataFrame with smoothed x, y coordinates

#### `removeOutlier(obs, MAX_SPEED_FOR_OUTLIER=50)`

Removes GPS points that would require unrealistic speeds.

**Parameters:**
- `obs`: DataFrame with x, y, timestamp columns
- `MAX_SPEED_FOR_OUTLIER`: Maximum allowed speed (m/s, default: 50)

**Returns:**
- Filtered DataFrame

#### `xystop_point_editing(df)`

Sets stop locations to the median of consecutive stop points.

**Parameters:**
- `df`: DataFrame with stopindex column

**Returns:**
- DataFrame with updated x, y coordinates for stops

## Interpolation Module

The interpolation module (`interpolation.py`) provides trajectory interpolation using Bezier curves.

### Functions

#### `interpolateTrajectory(traj, sample_rate=1)`

Interpolates a trajectory to create points at regular time intervals using Bezier curves.

**Parameters:**
- `traj`: DataFrame with columns: x, y, timestamp, speed, bearing, stopindex
- `sample_rate`: Time interval between interpolated points in seconds (default: 1)

**Returns:**
- DataFrame with interpolated points including original and extra points

**Algorithm:**
- Uses cubic Bezier curves for moving segments
- Control points are calculated based on velocity and bearing
- Handles stops by maintaining constant position

#### `bezierInterpolation(a, b, sample_rate=1)`

Interpolates between two GPS points using a cubic Bezier curve.

**Parameters:**
- `a`: First point (dict with x, y, timestamp, speed, bearing, stopindex)
- `b`: Second point (dict with x, y, timestamp, speed, bearing, stopindex)
- `sample_rate`: Time interval between points in seconds (default: 1)

**Returns:**
- DataFrame with interpolated points

**Algorithm:**
- Calculates control points based on velocity vectors
- Control point distance depends on speed and distance between points
- Uses derivative of Bezier curve to calculate bearing and speed

#### `Bezier_3(p0, p1, p2, p3, t)`

Evaluates a cubic Bezier curve at parameter t.

**Parameters:**
- `p0, p1, p2, p3`: Control points (numpy arrays)
- `t`: Parameter value (0-1)

**Returns:**
- Point on the curve (numpy array)

#### `derivative_Bezier_3(p0, p1, p2, p3, t)`

Calculates the derivative (velocity) of a cubic Bezier curve at parameter t.

**Parameters:**
- `p0, p1, p2, p3`: Control points (numpy arrays)
- `t`: Parameter value (0-1)

**Returns:**
- Velocity vector (numpy array)

## Map Matching Module

The map matching module (`mapmatching.py`) provides the core map matching algorithm.

### Class: `MapMatcher`

Main class for performing map matching.

#### Initialization

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

#### Methods

##### `match(sample_gps)`

Performs map matching on a GPS trajectory.

**Parameters:**
- `sample_gps`: DataFrame with columns: x, y, timestamp, speed, bearing, stopindex, type

**Returns:**
- 1 if successful, 0 if failed

**Algorithm:**
1. Initial point matching: Find candidate edges within search radius
2. For each GPS point:
   - Decide whether to stay on current edge or change to new edge
   - Calculate cost for each candidate edge
   - Select edge with minimum cost
   - Handle edge transitions and backtracking if needed

**Output:**
- Stores results in `self.matchdf` DataFrame

##### `save_routematch(routematchfile=None)`

Saves route matching results (edge sequences).

**Parameters:**
- `routematchfile`: Optional file path to save CSV

**Returns:**
- DataFrame with columns: from_edge, edgeid, edge_reverse, from_edge_reverse, departure, arrival, stop_time, travel_time, shape

##### `save_pointmatch(pointmatchfile=None)`

Saves point matching results (matched GPS points).

**Parameters:**
- `pointmatchfile`: Optional file path to save CSV

**Returns:**
- DataFrame with matched points including edge IDs, offsets, and match quality metrics

##### `reset()`

Resets the matcher state (clears matchdf and routedf).

##### `show_path()`

Prints the current matched path (edge IDs).

## Geographic Tools

The geographic tools module (`geotools.py`) provides utility functions for geographic calculations.

### Functions

#### `distance2d(point1, point2)`

Calculates Euclidean distance between two 2D points.

**Parameters:**
- `point1`: Tuple (x, y) of first point
- `point2`: Tuple (x, y) of second point

**Returns:**
- Distance in meters

#### `calculate_bearing_angle(point1, point2)`

Calculates bearing angle (0-360 degrees) from point1 to point2.

**Parameters:**
- `point1`: Tuple (x, y) of first point
- `point2`: Tuple (x, y) of second point

**Returns:**
- Bearing angle in degrees (0-360)

#### `distancePointToLine(point, line)`

Calculates distance from a point to a line segment.

**Parameters:**
- `point`: Tuple (x, y) of point
- `line`: Tuple (line_start, line_end) of line segment

**Returns:**
- Distance in meters

#### `polygonOffsetWithMinimumDistanceToPoint(point, polygon)`

Finds the point on a polygon closest to a given point and returns the offset along the polygon.

**Parameters:**
- `point`: Tuple (x, y) of point
- `polygon`: List of points defining the polygon

**Returns:**
- Tuple (offset, closest_point) where offset is distance along polygon

#### `offsetBearing(polygon, offset)`

Calculates the bearing angle at a specific offset along a polygon.

**Parameters:**
- `polygon`: List of points defining the polygon
- `offset`: Distance along polygon in meters

**Returns:**
- Bearing angle in degrees

**Raises:**
- `ValueError`: If offset is greater than polygon length or negative

#### `road_distance(currentedge, currentoffset, lastedge, lastoffset, lastedgelength)`

Calculates road network distance between two points on edges.

**Parameters:**
- `currentedge`: Current edge ID
- `currentoffset`: Offset along current edge
- `lastedge`: Last edge ID
- `lastoffset`: Offset along last edge
- `lastedgelength`: Length of last edge

**Returns:**
- Road distance in meters

#### `calculateUTMZone(min_lon, max_lon, min_lat, max_lat)`

Calculates appropriate UTM zone for a bounding box.

**Parameters:**
- `min_lon, max_lon`: Longitude range (degrees)
- `min_lat, max_lat`: Latitude range (degrees)

**Returns:**
- UTM zone number

#### `dfLonLat2XY(df, net)`

Converts DataFrame with lon/lat columns to x/y columns.

**Parameters:**
- `df`: DataFrame with lon, lat columns
- `net`: Network object for coordinate conversion

**Returns:**
- DataFrame with x, y columns (lon, lat removed)

#### `dfPoint2LonLat(df, net)`

Converts DataFrame with x/y columns to lon/lat columns.

**Parameters:**
- `df`: DataFrame with x, y columns
- `net`: Network object for coordinate conversion

**Returns:**
- DataFrame with lon, lat columns (x, y removed)

#### `polyLength(polygon)`

Calculates total length of a polyline.

**Parameters:**
- `polygon`: List of points defining the polyline

**Returns:**
- Total length in meters

## Utilities

The utilities module (`util.py`) provides logging functions.

### Functions

#### `get_logger(name)`

Creates a logger with file handler.

**Parameters:**
- `name`: Logger name (string)

**Returns:**
- Logger object

**Note:**
- Creates log directory if it doesn't exist
- Logs are saved to `../../log/{name}.log`

