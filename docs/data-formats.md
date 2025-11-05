# Data Formats

This document describes the data formats used throughout the NRTMapMatching system.

## Input GPS Data Format

### Required Columns

GPS observation data must be a pandas DataFrame with the following columns:

| Column | Type | Description |
|--------|------|-------------|
| `lon` | float | Longitude in degrees |
| `lat` | float | Latitude in degrees |
| `timestamp` | int | Unix timestamp in seconds |
| `speed` | float | Speed in meters per second (m/s) |
| `bearing` | int | Bearing angle in degrees (0-360) |

### Example

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

### Notes

- Data must be sorted by timestamp
- Duplicate timestamps are handled (last value kept)
- Minimum 2 observations required

## Cleaned/Enriched Data Format

### Columns

After cleaning or enriching, the DataFrame contains:

| Column | Type | Description |
|--------|------|-------------|
| `id` | int | Trajectory ID |
| `x` | float | X coordinate in meters (local projection) |
| `y` | float | Y coordinate in meters (local projection) |
| `timestamp` | int | Unix timestamp in seconds |
| `speed` | float | Speed in meters per second (m/s) |
| `bearing` | float | Bearing angle in degrees (0-360) |
| `stopindex` | int | Stop index (0 = moving, >0 = stopped) |

### Notes

- Coordinates are in local UTM projection (not lon/lat)
- `stopindex` = 0 indicates vehicle is moving
- `stopindex` > 0 indicates vehicle is stopped (consecutive stops share same index)
- Stop locations are set to median of consecutive stop points

## Interpolated Data Format

### Columns

Interpolated trajectory data includes all cleaned data columns plus:

| Column | Type | Description |
|--------|------|-------------|
| `type` | str | Point type: "origin" (original) or "extra" (interpolated) |

### Notes

- Original GPS points are marked as "origin"
- Interpolated points are marked as "extra"
- Interpolation creates points at regular time intervals (sample_rate)

## Map Matching Output Format

### Point Matching Results (`matchdf`)

Contains matched GPS points with the following columns:

| Column | Type | Description |
|--------|------|-------------|
| `index` | int | Original GPS point index |
| `timestamp` | int | Unix timestamp in seconds |
| `x` | float | Matched X coordinate on road network |
| `y` | float | Matched Y coordinate on road network |
| `x_sample` | float | Original GPS X coordinate |
| `y_sample` | float | Original GPS Y coordinate |
| `edgeid` | str | Matched edge ID |
| `offset` | float | Offset along edge in meters |
| `dist` | float | Distance from GPS to matched point (meters) |
| `distbearing` | float | Bearing error in degrees |
| `rd` | float | Road distance error (meters) |
| `cost_air` | float | Air distance error (meters) |
| `matchbearing` | float | Bearing of matched edge segment (degrees) |
| `speed` | float | Speed at matched point (m/s) |
| `predict_distance` | float | Predicted distance based on speed (meters) |
| `matched_road_distance` | float | Actual road distance traveled (meters) |
| `edge_reverse` | bool | Whether edge is traversed in reverse |
| `from_edge` | str | Previous edge ID (None for first point) |
| `from_edge_reverse` | bool | Whether previous edge was reversed |
| `edge_length` | float | Length of matched edge (meters) |
| `type` | str | Point type from interpolated data |
| `decitsion` | str | Decision type (STAY/CHANGE/NODECISION) |

### Route Matching Results (`routedf`)

Contains edge sequences with the following columns:

| Column | Type | Description |
|--------|------|-------------|
| `from_edge` | str | Source edge ID (None for first edge) |
| `edgeid` | str | Current edge ID |
| `edge_reverse` | bool | Whether edge is traversed in reverse |
| `from_edge_reverse` | bool | Whether source edge was reversed |
| `departure` | int | Timestamp when entering edge |
| `arrival` | int | Timestamp when leaving edge |
| `stop_time` | int | Number of stopped points on edge |
| `travel_time` | int | Total travel time on edge (seconds) |
| `shape` | LineString | Shapely LineString geometry of the edge |

### Notes

- Route matching results are unique edge sequences
- Each row represents one edge segment in the route
- `shape` column contains Shapely geometry objects (can be converted to GeoJSON)

## Coordinate Systems

### Geographic Coordinates (lon/lat)

- Standard WGS84 geographic coordinates
- Longitude: -180 to 180 degrees
- Latitude: -90 to 90 degrees
- Used for: Input GPS data, final output

### Local Projected Coordinates (x/y)

- UTM (Universal Transverse Mercator) projection
- Automatically determined based on bounding box
- Units: meters
- Used for: Internal calculations, network edges

### Conversion

Use network methods for coordinate conversion:

```python
# Convert lon/lat to x/y
x, y = net.convertLonLat2XY(lon, lat)

# Convert x/y to lon/lat
lon, lat = net.convertXY2LonLat(x, y)
```

## Data Validation

### Input Validation

The system validates:
- Required columns are present
- Minimum number of observations (>= 2)
- Timestamp ordering (data is sorted automatically)

### Error Handling

- Missing columns: Prints error message, returns 0 or empty DataFrame
- Invalid timestamps: Data is sorted by timestamp
- Duplicate timestamps: Last value is kept
- Outliers: Removed based on speed thresholds

## Example Workflow

```python
# 1. Load GPS data (lon/lat)
gps = pd.read_csv("gps_data.csv")

# 2. Clean/enrich (converts to x/y)
cleaned = richdata(gps, mynet)

# 3. Interpolate (still x/y)
interpolated = interpolateTrajectory(cleaned)

# 4. Map match (still x/y)
matcher = MapMatcher(mynet)
matcher.match(interpolated)

# 5. Convert back to lon/lat if needed
from geotools import dfPoint2LonLat
matched_lonlat = dfPoint2LonLat(matcher.matchdf, mynet)
```

