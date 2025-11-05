# Configuration

This document describes all configuration parameters available in NRTMapMatching.

## MapMatcher Parameters

### Initialization Parameters

```python
MapMatcher(net, MAX_GPS_ERROR=60, MAX_MAP_ERROR=40, 
           MAP_ONE_WAY_FIX=True, U_TURN_ON_ONEWAY=False,
           LOOP=True, MAX_SPEED=100, DIFF_GPS_ERROR=10,
           MAX_RUNNING_TIME=5)
```

#### `net` (required)
- **Type**: Network object
- **Description**: Road network object created from `network.Net()`
- **Example**: `mynet = network.Net(); mynet.importFromSumoNet(sumo_net)`

#### `MAX_GPS_ERROR` (default: 60)
- **Type**: float
- **Units**: meters
- **Description**: Maximum expected GPS error. Used to calculate search radius.
- **Recommended Values**:
  - High accuracy GPS (RTK): 10-30 meters
  - Standard GPS: 30-60 meters
  - Low accuracy GPS: 60-100 meters
  - Urban canyons: 80-120 meters

#### `MAX_MAP_ERROR` (default: 40)
- **Type**: float
- **Units**: meters
- **Description**: Maximum expected map/network error. Combined with GPS error for search radius.
- **Recommended Values**:
  - High-quality maps: 20-40 meters
  - Standard maps: 40-60 meters
  - Outdated maps: 60-100 meters

#### `MAP_ONE_WAY_FIX` (default: True)
- **Type**: bool
- **Description**: Allow matching to edges in reverse direction (useful for one-way street errors).
- **Use Cases**:
  - `True`: When map data may have one-way direction errors
  - `False`: When map data is highly accurate and one-way rules are strict

#### `U_TURN_ON_ONEWAY` (default: False)
- **Type**: bool
- **Description**: Allow U-turns on one-way streets.
- **Use Cases**:
  - `True`: When vehicles may make U-turns (e.g., delivery vehicles)
  - `False`: Standard road network behavior

#### `LOOP` (default: True)
- **Type**: bool
- **Description**: Allow revisiting edges (useful for loops and roundabouts).
- **Use Cases**:
  - `True`: When routes may revisit edges (default)
  - `False`: In dense networks to prevent unrealistic paths

#### `MAX_SPEED` (default: 100)
- **Type**: float
- **Units**: meters per second (m/s)
- **Description**: Maximum speed for validation (≈ 360 km/h).
- **Note**: Used for edge validation, not for speed limit enforcement

#### `DIFF_GPS_ERROR` (default: 10)
- **Type**: float
- **Units**: meters
- **Description**: GPS error difference threshold for decision making.
- **Effect**: Larger values make decisions more conservative

#### `MAX_RUNNING_TIME` (default: 5)
- **Type**: float
- **Units**: seconds
- **Description**: Maximum running time for map matching algorithm.
- **Note**: Algorithm will terminate if exceeded (returns 0)

## Data Cleaning Parameters

### `cleaningData()` Parameters

```python
cleaningData(obs, net, id=0, MINSPEED_FOR_BEARING=1, MAX_SPEED_FOR_OUTLIER=100)
```

#### `obs` (required)
- **Type**: pandas.DataFrame
- **Description**: GPS observations with required columns

#### `net` (required)
- **Type**: Network object
- **Description**: Network for coordinate conversion

#### `id` (default: 0)
- **Type**: int
- **Description**: Trajectory ID

#### `MINSPEED_FOR_BEARING` (default: 1)
- **Type**: float
- **Units**: m/s
- **Description**: Minimum speed for calculating new bearing. Below this, previous bearing is used.
- **Recommended**: 0.5-2.0 m/s

#### `MAX_SPEED_FOR_OUTLIER` (default: 100)
- **Type**: float
- **Units**: m/s (≈ 360 km/h)
- **Description**: Maximum speed for outlier detection. Points requiring higher speeds are removed.
- **Recommended**: 50-150 m/s depending on vehicle type

### `richdata()` Parameters

```python
richdata(obs, net, id=0, alpha=.7, type="local", 
         MIN_SPEED=1, MAX_SPEED_FOR_OUTLIER=50, MINSPEED_FOR_BEARING=2)
```

#### `alpha` (default: 0.7)
- **Type**: float
- **Range**: 0.0 - 1.0
- **Description**: Smoothing parameter for exponential moving average. Higher = less smoothing.
- **Recommended Values**:
  - High GPS noise: 0.5-0.7 (more smoothing)
  - Low GPS noise: 0.7-0.9 (less smoothing)
  - Raw data: 0.9-1.0 (minimal smoothing)

#### `type` (default: "local")
- **Type**: str
- **Options**: "local", "aggregated"
- **Description**: Smoothing type. Currently only "local" is fully implemented.

#### `MIN_SPEED` (default: 1)
- **Type**: float
- **Units**: m/s
- **Description**: Minimum speed threshold. Speeds below this are set to 0.
- **Recommended**: 0.5-2.0 m/s

#### `MAX_SPEED_FOR_OUTLIER` (default: 50)
- **Type**: float
- **Units**: m/s (≈ 180 km/h)
- **Description**: Maximum speed for outlier detection.
- **Recommended by Vehicle Type**:
  - Bicycles: 15-20 m/s
  - Cars: 40-50 m/s
  - Trucks: 30-40 m/s
  - Highways: 50-60 m/s

#### `MINSPEED_FOR_BEARING` (default: 2)
- **Type**: float
- **Units**: m/s
- **Description**: Minimum speed for bearing calculation.
- **Recommended**: 1-3 m/s

## Interpolation Parameters

### `interpolateTrajectory()` Parameters

```python
interpolateTrajectory(traj, sample_rate=1)
```

#### `traj` (required)
- **Type**: pandas.DataFrame
- **Description**: Cleaned trajectory data

#### `sample_rate` (default: 1)
- **Type**: float
- **Units**: seconds
- **Description**: Time interval between interpolated points.
- **Recommended Values**:
  - High-frequency analysis: 0.5-1 second
  - Standard analysis: 1-2 seconds
  - Low-frequency analysis: 2-5 seconds

## Parameter Tuning Guide

### For High GPS Accuracy

```python
matcher = MapMatcher(
    mynet,
    MAX_GPS_ERROR=30,
    MAX_MAP_ERROR=20,
    DIFF_GPS_ERROR=5
)

cleaned = richdata(
    gps, mynet,
    alpha=0.8,  # Less smoothing needed
    MAX_SPEED_FOR_OUTLIER=40
)
```

### For Low GPS Accuracy

```python
matcher = MapMatcher(
    mynet,
    MAX_GPS_ERROR=100,
    MAX_MAP_ERROR=60,
    DIFF_GPS_ERROR=20
)

cleaned = richdata(
    gps, mynet,
    alpha=0.5,  # More smoothing
    MAX_SPEED_FOR_OUTLIER=60
)
```

### For Dense Urban Areas

```python
matcher = MapMatcher(
    mynet,
    MAX_GPS_ERROR=80,
    MAX_MAP_ERROR=40,
    MAP_ONE_WAY_FIX=True,
    U_TURN_ON_ONEWAY=False,
    LOOP=False  # Disallow loops
)
```

### For Highway Networks

```python
matcher = MapMatcher(
    mynet,
    MAX_GPS_ERROR=50,
    MAX_MAP_ERROR=30,
    MAX_SPEED=150,  # Higher max speed
    LOOP=True
)

cleaned = richdata(
    gps, mynet,
    MAX_SPEED_FOR_OUTLIER=60  # Higher outlier threshold
)
```

### For Real-time Processing

```python
matcher = MapMatcher(
    mynet,
    MAX_RUNNING_TIME=2  # Faster timeout
)

# Use larger sample_rate for faster processing
interpolated = interpolateTrajectory(cleaned, sample_rate=2)
```

## Default Configuration Summary

| Parameter | Default | Typical Range |
|-----------|---------|---------------|
| `MAX_GPS_ERROR` | 60 m | 30-100 m |
| `MAX_MAP_ERROR` | 40 m | 20-60 m |
| `alpha` | 0.7 | 0.5-0.9 |
| `sample_rate` | 1 s | 0.5-5 s |
| `MAX_SPEED_FOR_OUTLIER` | 50 m/s | 20-60 m/s |
| `MIN_SPEED` | 1 m/s | 0.5-2 m/s |

## Best Practices

1. **Start with defaults**: Default values work well for most cases
2. **Tune based on GPS quality**: Adjust error parameters based on your GPS device
3. **Match vehicle type**: Adjust speed thresholds for bicycles, cars, trucks
4. **Monitor match quality**: Use distance metrics in results to tune parameters
5. **Test incrementally**: Change one parameter at a time to understand effects

