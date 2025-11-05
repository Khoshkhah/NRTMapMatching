# User Guide

This guide provides step-by-step examples for using NRTMapMatching.

## Complete Workflow Example

### 1. Load a Road Network

```python
import sumolib
import network

# Option A: Load SUMO network
net1 = sumolib.net.readNet("data/iland.net", withInternal=False)
mynet = network.Net()
mynet.importFromSumoNet(net1)

# Option B: Load OSM network
# mynet.importFromOSM("data/network.osm")
```

### 2. Load GPS Data

```python
import pandas as pd

# Load from CSV
gps = pd.read_csv("gps_data.csv")

# Or from ZIP file
import zipfile
zf = zipfile.ZipFile('data/gps_inside_island.zip')
gps = pd.read_csv(zf.open("gps_inside_island.csv"))

# Required columns: lon, lat, timestamp, speed, bearing
print(gps.columns)
```

### 3. Prepare GPS Data

```python
# Select a specific trip
unique_trip = list(gps["trip_id_extended"].unique())
sample_gps = gps[gps["trip_id_extended"] == unique_trip[0]]
sample_gps = sample_gps.sort_values("timestamp").drop_duplicates()

# Ensure required columns exist
required_cols = ["lon", "lat", "timestamp", "speed", "bearing"]
assert all(col in sample_gps.columns for col in required_cols)
```

### 4. Clean and Enrich Data

```python
from cleandata import richdata

# Enrich data with smoothing and outlier removal
cleaned = richdata(
    sample_gps, 
    mynet, 
    id=0, 
    alpha=0.7,  # Smoothing parameter
    MIN_SPEED=1,
    MAX_SPEED_FOR_OUTLIER=50,
    MINSPEED_FOR_BEARING=2
)

# Or use basic cleaning
# from cleandata import cleaningData
# cleaned = cleaningData(sample_gps, mynet, id=0)
```

### 5. Interpolate Trajectory

```python
from interpolation import interpolateTrajectory

# Interpolate to create points at 1-second intervals
interpolated = interpolateTrajectory(cleaned, sample_rate=1)

print(f"Original points: {len(cleaned)}")
print(f"Interpolated points: {len(interpolated)}")
```

### 6. Perform Map Matching

```python
from mapmatching import MapMatcher

# Create matcher with default parameters
matcher = MapMatcher(
    mynet,
    MAX_GPS_ERROR=60,      # Maximum GPS error in meters
    MAX_MAP_ERROR=40,      # Maximum map error in meters
    MAP_ONE_WAY_FIX=True,  # Allow reverse direction matching
    LOOP=True              # Allow revisiting edges
)

# Perform matching
result = matcher.match(interpolated)

if result == 1:
    print("Map matching successful!")
else:
    print("Map matching failed")
```

### 7. Save Results

```python
# Save point matching results
point_match = matcher.save_pointmatch("point_match.csv")
print(f"Matched {len(point_match)} points")

# Save route matching results
route_match = matcher.save_routematch("route_match.csv")
print(f"Matched {len(route_match)} route segments")

# Access matched data directly
print(matcher.matchdf.head())
print(matcher.routedf.head())
```

## Working with Results

### Analyze Point Matches

```python
import pandas as pd

# Load point matches
point_match = pd.read_csv("point_match.csv")

# Calculate match quality metrics
print(f"Average distance error: {point_match['dist'].mean():.2f} meters")
print(f"Average bearing error: {point_match['distbearing'].mean():.2f} degrees")
print(f"Max distance error: {point_match['dist'].max():.2f} meters")

# Filter by quality
good_matches = point_match[point_match['dist'] < 20]
print(f"Good matches (< 20m): {len(good_matches)}/{len(point_match)}")
```

### Analyze Route Matches

```python
# Load route matches
route_match = pd.read_csv("route_match.csv")

# Calculate route statistics
print(f"Total route segments: {len(route_match)}")
print(f"Total travel time: {route_match['travel_time'].sum()} seconds")
print(f"Total stop time: {route_match['stop_time'].sum()} seconds")

# Get edge sequence
edge_sequence = route_match['edgeid'].tolist()
print(f"Route: {' -> '.join(edge_sequence[:5])}...")
```

## Common Use Cases

### Processing Multiple Trajectories

```python
unique_trips = list(gps["trip_id_extended"].unique())

for i, trip_id in enumerate(unique_trips[:10]):  # Process first 10 trips
    print(f"Processing trip {i+1}/{min(10, len(unique_trips))}: {trip_id}")
    
    # Get trip data
    trip_data = gps[gps["trip_id_extended"] == trip_id]
    trip_data = trip_data.sort_values("timestamp").drop_duplicates()
    
    # Process
    cleaned = richdata(trip_data, mynet, id=i)
    interpolated = interpolateTrajectory(cleaned, sample_rate=1)
    
    # Match
    matcher = MapMatcher(mynet)
    result = matcher.match(interpolated)
    
    if result == 1:
        # Save results with unique filenames
        matcher.save_pointmatch(f"results/point_match_{i}.csv")
        matcher.save_routematch(f"results/route_match_{i}.csv")
```

### Adjusting Parameters for Better Results

```python
# For high GPS accuracy (good devices)
matcher = MapMatcher(
    mynet,
    MAX_GPS_ERROR=30,      # Smaller error radius
    MAX_MAP_ERROR=20,
    DIFF_GPS_ERROR=5
)

# For low GPS accuracy (poor conditions)
matcher = MapMatcher(
    mynet,
    MAX_GPS_ERROR=100,     # Larger error radius
    MAX_MAP_ERROR=60,
    DIFF_GPS_ERROR=20
)

# For dense urban areas with many one-way streets
matcher = MapMatcher(
    mynet,
    MAP_ONE_WAY_FIX=True,
    U_TURN_ON_ONEWAY=False,
    LOOP=False  # Disallow loops in dense areas
)
```

### Converting Coordinates

```python
from geotools import dfPoint2LonLat, dfLonLat2XY

# Convert matched points back to lat/lon
matched_lonlat = dfPoint2LonLat(matcher.matchdf, mynet)

# Convert GPS data to x/y
gps_xy = dfLonLat2XY(gps, mynet)
```

## Troubleshooting

### Common Issues

1. **Map matching fails (returns 0)**
   - Check GPS data quality (too many outliers?)
   - Increase `MAX_GPS_ERROR` and `MAX_MAP_ERROR`
   - Verify network covers the GPS trajectory area

2. **Poor match quality (high distances)**
   - Check GPS accuracy
   - Verify network data is up-to-date
   - Adjust smoothing parameters in `richdata()`

3. **Missing required columns**
   - Ensure GPS data has: lon, lat, timestamp, speed, bearing
   - Check column names match exactly

4. **Network import fails**
   - Verify network file format (SUMO .net or OSM)
   - Check file path is correct
   - Ensure network file is valid

## Next Steps

- See [API Reference](api-reference.md) for detailed function documentation
- Check [Configuration](configuration.md) for all available parameters
- Review [Algorithms](algorithms.md) to understand how matching works

