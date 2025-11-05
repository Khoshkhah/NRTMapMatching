# Overview

## Introduction

NRTMapMatching is a Python library for matching GPS trajectory data to road network edges in near real-time. The system processes GPS observations, cleans and enriches the data, interpolates trajectories, and matches them to a road network using an advanced map matching algorithm.

## Architecture

The system consists of several key components:

1. **Network Module** (`network.py`): Handles road network loading and management
   - Supports SUMO .net files and OpenStreetMap (OSM) files
   - Provides spatial indexing for efficient edge queries
   - Manages coordinate system conversions

2. **Data Cleaning Module** (`cleandata.py`): Cleans and enriches GPS observations
   - Removes outliers based on speed thresholds
   - Smooths trajectories using exponential moving averages
   - Detects and handles vehicle stops
   - Calculates derived metrics (speed, bearing)

3. **Interpolation Module** (`interpolation.py`): Interpolates trajectories using Bezier curves
   - Uses cubic Bezier curves for smooth path interpolation
   - Maintains realistic speeds and bearings
   - Handles stops appropriately

4. **Map Matching Module** (`mapmatching.py`): Core map matching algorithm
   - Multi-factor cost function for edge selection
   - Greedy algorithm with backtracking
   - Handles edge transitions and one-way streets

5. **Geographic Tools** (`geotools.py`): Utility functions for geographic calculations
   - Distance calculations
   - Bearing calculations
   - Point-to-line distance
   - Coordinate transformations

6. **Utilities** (`util.py`): Logging and utility functions

## Key Features

- **Road Network Support**: Import networks from SUMO .net files or OpenStreetMap files
- **GPS Data Cleaning**: Remove outliers and smooth trajectories
- **Data Enrichment**: Enhance GPS data with speed and bearing calculations
- **Trajectory Interpolation**: Interpolate trajectories using cubic Bezier curves
- **Map Matching**: Match GPS points to road network edges using advanced algorithms
- **Spatial Indexing**: Efficient edge queries using R-tree spatial indexing
- **Stop Detection**: Automatic detection and handling of vehicle stops

## Workflow

The typical workflow consists of:

1. **Load Network**: Import road network from SUMO or OSM file
2. **Load GPS Data**: Load GPS observations with required fields
3. **Clean/Enrich Data**: Remove outliers and enrich with calculated metrics
4. **Interpolate Trajectory**: Create interpolated points at regular intervals
5. **Map Matching**: Match GPS points to road network edges
6. **Save Results**: Export matched points and routes

For detailed examples, see the [User Guide](user-guide.md).

