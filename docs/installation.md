# Installation

## Prerequisites

- Python 3.7 or higher
- pip (Python package installer)

## Install Dependencies

### Basic Installation

```bash
pip install -r requirements.txt
```

### Required Packages

The project requires the following Python packages:

- `pandas` - Data manipulation and analysis
- `numpy` - Numerical computations
- `sumolib` - SUMO network file reading
- `shapely` - Geometric operations
- `rtree` - Spatial indexing
- `pyproj` - Coordinate system transformations
- `osmnx` - OpenStreetMap network processing (for OSM import)
- `pyrosm` - OSM file reading (for OSM import)

**Note**: The `requirements.txt` includes `zipfile`, which is a built-in Python module and doesn't need to be installed separately.

### Optional Dependencies

For development and testing:
- `jupyter` - For running test notebooks
- `matplotlib` - For visualization (if needed)

## Verify Installation

After installation, you can verify the installation by importing the modules:

```python
import network
from cleandata import richdata
from interpolation import interpolateTrajectory
from mapmatching import MapMatcher
```

If no errors occur, the installation was successful.

## Network Data

You'll also need road network data:

- **SUMO Networks**: `.net` files created with SUMO's netconvert tool
- **OSM Files**: OpenStreetMap `.osm` or `.osm.pbf` files

See the [User Guide](user-guide.md) for examples of loading network data.

