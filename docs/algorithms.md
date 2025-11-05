# Algorithms

This document describes the algorithms used in NRTMapMatching.

## Map Matching Algorithm

The map matching algorithm uses a greedy approach with backtracking to match GPS points to road network edges.

### Overview

1. **Initial Matching**: Find candidate edges within search radius for the first GPS point
2. **Iterative Matching**: For each subsequent GPS point, determine the best matching edge
3. **Backtracking**: If match quality is poor, backtrack to previous decision point
4. **Cost Calculation**: Multi-factor cost function for edge selection

### Step-by-Step Process

#### 1. Initial Point Matching

For the first GPS point:
- Calculate search radius: `radius = MAX_GPS_ERROR + MAX_MAP_ERROR`
- Find all edges within radius using R-tree spatial index
- Store candidate edges for initial matching

#### 2. Decision Making

For each GPS point, the algorithm makes one of three decisions:

- **STAY**: Vehicle remains on current edge
  - Condition: Remaining edge length >= (max_speed × time) + GPS error
  - Candidate edges: Only current edge

- **CHANGE**: Vehicle transitions to a new edge
  - Condition: Remaining edge length < (speed × time) - GPS error
  - Candidate edges: Outgoing edges from current edge

- **NODECISION**: Ambiguous case
  - Condition: Between STAY and CHANGE thresholds
  - Candidate edges: Current edge + outgoing edges

#### 3. Cost Calculation

For each candidate edge, calculate a multi-factor cost:

```python
cost = 1 × bearing_error + 
       30 × match_point_distance + 
       10 × air_distance_error + 
       5 × road_distance_error
```

If edge is traversed in reverse:
```python
cost += 100000  # Large penalty for reverse direction
```

**Cost Components:**

- **Bearing Error**: Difference between GPS bearing and edge bearing (degrees)
- **Match Point Distance**: Distance from GPS point to nearest point on edge (meters)
- **Air Distance Error**: Difference between GPS distance and matched distance (meters)
- **Road Distance Error**: Difference between predicted distance (speed × time) and actual road distance (meters)

#### 4. Edge Selection

- Select candidate edge with minimum cost
- Verify match quality: distance < search radius
- If quality is poor, backtrack to previous decision point

#### 5. Backtracking

If match quality is poor (distance > radius):
1. Remove current edge from candidate list
2. Pop decision from decision list
3. Remove edge from path
4. Return to previous GPS point
5. Try alternative candidate edges

#### 6. Edge Transition Handling

When transitioning between edges:
- Combine edge shapes for smooth transitions
- Calculate road distance across edges
- Track edge direction (forward/reverse)

### Cost Function Details

The cost function balances multiple factors:

| Factor | Weight | Description |
|--------|--------|-------------|
| Bearing Error | 1 | Alignment of GPS and edge direction |
| Match Distance | 30 | Geometric distance to edge |
| Air Distance Error | 10 | Consistency of GPS movement |
| Road Distance Error | 5 | Consistency with speed |

### Search Radius

The search radius determines candidate edges:
```
search_radius = MAX_GPS_ERROR + MAX_MAP_ERROR
```

Typical values:
- High GPS accuracy: 30-50 meters
- Medium GPS accuracy: 60-80 meters
- Low GPS accuracy: 100+ meters

### Performance Optimizations

1. **Spatial Indexing**: R-tree for efficient edge queries
2. **Early Termination**: Maximum running time limit
3. **Backtracking Limit**: Prevents infinite loops
4. **Edge Caching**: Reuses edge shape calculations

## Bezier Interpolation Algorithm

The interpolation uses cubic Bezier curves to create smooth trajectories between GPS points.

### Overview

- Uses cubic Bezier curves (4 control points)
- Control points positioned based on velocity vectors
- Maintains realistic speeds and bearings
- Handles stops appropriately

### Cubic Bezier Curve

A cubic Bezier curve is defined by four control points P₀, P₁, P₂, P₃:

```
B(t) = (1-t)³P₀ + 3(1-t)²tP₁ + 3(1-t)t²P₂ + t³P₃
```

Where:
- t ∈ [0, 1] is the parameter
- P₀ is the start point (point a)
- P₃ is the end point (point b)
- P₁ and P₂ are control points

### Control Point Calculation

Control points are calculated based on velocity vectors:

1. **Calculate velocity vectors** from speed and bearing:
   ```
   v_a_x = (speed_a + 1) × sin(bearing_a)
   v_a_y = (speed_a + 1) × cos(bearing_a)
   ```

2. **Calculate control point distances**:
   ```
   control = 10 + max(speed_a, speed_b)^1.2
   alpha = (0.5 × √(speed_a+1) / (√(speed_a+1) + control)) × distance × 3
   beta = (0.5 × √(speed_b+1) / (√(speed_b+1) + control)) × distance × 3
   ```

3. **Position control points**:
   ```
   P₁ = P₀ + (alpha × v_a) / 3
   P₂ = P₃ - (beta × v_b) / 3
   ```

### Interpolation Process

For each pair of consecutive points (a, b):

1. **Check if stopped**: If both points are stopped, maintain constant position
2. **Calculate control points**: Based on velocity vectors
3. **Sample curve**: Evaluate Bezier curve at regular intervals (sample_rate)
4. **Calculate interpolated values**:
   - Position: B(t) from Bezier curve
   - Speed: Distance between consecutive interpolated points / sample_rate
   - Bearing: Derivative of Bezier curve (tangent direction)

### Speed and Bearing Calculation

**Speed**:
- Calculated from distance between consecutive interpolated points
- `speed = distance / sample_rate`

**Bearing**:
- Calculated from derivative of Bezier curve
- Derivative gives velocity vector (tangent to curve)
- Bearing is angle of velocity vector

### Stop Handling

When both points are stopped (`stopindex > 0`):
- Maintain constant position (no interpolation)
- Speed = 0
- Bearing = previous bearing
- Create points at regular intervals with same position

## Data Cleaning Algorithm

### Outlier Removal

Outliers are removed based on speed:

1. Calculate estimated speed between consecutive points:
   ```
   speed_estimated = distance / time_delta
   ```

2. Remove point if `speed_estimated > MAX_SPEED_FOR_OUTLIER`

### Smoothing

Exponential weighted moving average (EWMA) of velocity:

1. Calculate velocity components:
   ```
   vx = (x_next - x) / time_delta
   vy = (y_next - y) / time_delta
   ```

2. Apply EWMA smoothing:
   ```
   vx_smoothed = EWMA(vx, alpha)
   vy_smoothed = EWMA(vy, alpha)
   ```

3. Update positions:
   ```
   x_new = x + vx_smoothed × time_delta
   y_new = y + vy_smoothed × time_delta
   ```

### Stop Detection

Stops are detected and handled:

1. **Identify stops**: Points with `speed < MIN_SPEED`
2. **Assign stop indices**: Consecutive stops share same index
3. **Fix stop locations**: Set stop location to median of consecutive stop points

## Performance Characteristics

### Time Complexity

- **Map Matching**: O(n × m × k) where:
  - n = number of GPS points
  - m = average number of candidate edges per point
  - k = average number of backtracking steps

- **Interpolation**: O(n × s) where:
  - n = number of GPS points
  - s = average number of samples per segment

- **Network Query**: O(log n) using R-tree spatial index

### Space Complexity

- **Network Storage**: O(e + n) where:
  - e = number of edges
  - n = number of nodes

- **Matching State**: O(d) where:
  - d = depth of decision tree

### Optimization Strategies

1. **Spatial Indexing**: R-tree for O(log n) edge queries
2. **Early Termination**: Maximum running time limit
3. **Backtracking Limit**: Prevents excessive backtracking
4. **Edge Caching**: Reuses shape calculations

## Limitations

1. **GPS Accuracy**: Algorithm assumes GPS errors < 100m
2. **Network Completeness**: Requires complete road network data
3. **Complex Intersections**: May struggle with complex multi-level intersections
4. **GPS Gaps**: Large time gaps between points may cause issues
5. **One-way Streets**: Reverse direction matching may need tuning

