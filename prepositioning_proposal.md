# Line-of-Sight Visibility Analysis for Wildfire Ignition Points

## Background Context

### Problem Statement
We have a set of historical wildland ignition points (a few thousand features) and need to determine which locations on the road network can observe these ignition points, accounting for terrain obstruction.

### Previous Approach
- Computed full viewshed raster for each ignition point as observer
- Summed all viewsheds to create composite layer showing ignition visibility count
- Created distance-to-ignition raster layer with visibility range threshold
- Multiplied layers to identify road cells within visibility range with ignition counts

**Problems:**
- Extremely time-consuming (full viewshed computation for thousands of points)
- Generates very large raster datasets
- Processing entire DEM extent when only road network matters

### Proposed Improved Approach
Instead of computing full viewsheds:
1. Sample road network at discrete points
2. Perform pairwise line-of-sight (LOS) checks between road points and ignition points
3. Store results as matrices (dense distance and dense boolean visibility) for efficient querying
4. Enable non-dominant search optimization (maximize visibility count, minimize total distance)

**Advantages:**
- Orders of magnitude faster (only processes road network, not entire DEM)
- Much smaller output datasets (matrices vs. full rasters)
- Enables sophisticated multi-objective optimization

---

## Research Question

**Primary Question:**
From any location on the road network, how many historical wildfire ignition incidents can be observed (accounting for terrain obstruction)?

**Secondary Question:**
For each distinct visibility count, which road location(s) minimize the total Euclidean distance to all ignition points?

This supports optimal placement of fire detection resources (cameras, observers, etc.).

---

## Algorithm Steps

### Step 1: Road Network Sampling
- **Input:** Road network as vector linestrings (Shapefile, GeoJSON, or GeoPackage)
- **Process:** 
  - Interpolate each linestring at fixed intervals (user-defined, e.g., 150 feet)
  - For short segments (length <= interval * 1.5), use midpoint
  - Extract coordinates as point array
- **Output:** N road points with (x, y) coordinates

### Step 2: Distance Matrix Computation
- **Input:** 
  - N road points from Step 1
  - M ignition points (x, y coordinates from geospatial file)
- **Process:** 
  - Compute Euclidean distance between every road point and every ignition point
  - Create dense N × M matrix
- **Output:** Dense distance matrix (N × M, float32)
- **Rationale:** Non-sparse because total distance computation (sum across all ignitions) is needed for non-dominant search

### Step 3: Line-of-Sight Visibility Computation
- **Input:**
  - N road points
  - M ignition points
  - Distance matrix from Step 2
  - DEM (Digital Elevation Model) as GeoTIFF
  - Visibility range threshold (e.g., 12 miles)
- **Process:**
  - For each (road_point, ignition_point) pair:
    - Skip if Euclidean distance > visibility threshold
    - Perform LOS check using DEM (see algorithm below)
    - Record true/false visibility
- **Output:** Dense boolean visibility matrix (N×M uint8: 1 = visible, 0 = not visible)
- **Optimization:** Parallelize using OpenMP (each road point is independent)

### Step 4: Data Storage
- **Format:** HDF5 (single file, efficient for dense matrices)
- **Structure:** All datasets at file root (one table + two matrices).
  ```
  /coords              [N × 2 float64]      # road point coordinates (lon/lat EPSG:4326 if CRS set; else raw x,y)
  /distances           [N × M float32]      # DENSE: Euclidean distances
  /visibility_matrix   [N × M uint8]        # DENSE: 1 = visible, 0 = not visible (road i → ignition j)
  ```
- **Access:** Python (h5py), Go (gonum/hdf5)

---

## Program Input/Output Specification

### Inputs
1. **Road network file** (vector format: Shapefile, GeoJSON, GeoPackage)
   - Linestring geometry representing road network
2. **Ignition points file** (vector format: Shapefile, GeoJSON, GeoPackage)
   - Point geometry for historical wildfire ignition locations
3. **Digital Elevation Model (DEM)** (raster format: GeoTIFF)
   - Terrain elevation data
4. **Configuration parameters:**
   - `road_sample_interval`: spacing between road samples (e.g., 150 feet)
   - `visibility_range`: maximum distance for visibility checks (e.g., 12 miles)

### Outputs
1. **HDF5 file** (`visibility_analysis.h5`)
   - Contains all matrices and interpolated road point coordinate data as specified in Step 4

---

## Line-of-Sight Algorithm

### Conceptual Overview

Line-of-sight between two points is **blocked** if terrain anywhere along the path rises above the straight line connecting them in 3D space.

**Key Insight:** The sight line is a **sloped line in 3D space**, not a horizontal threshold.

#### Example Scenario
```
Observer A: elevation 10 ft
Target B: elevation 5 ft (100m away horizontally)
Terrain at midpoint (50m): elevation 8 ft

Sight line elevation at midpoint: 10 + 0.5 × (5 - 10) = 7.5 ft
Terrain elevation: 8 ft

8 ft > 7.5 ft → BLOCKED ❌
```

Visual representation:
```
Side view:

10 ft |  A (observer)
      |   \
      |    \  <-- sight line
 8 ft |     X  <-- TERRAIN BLOCKS (8 > 7.5)
      |      \   (sight line at 7.5 ft)
      |       \
 5 ft |________B (target)
      |
      0m  50m  100m
```

**Important Property:** Line-of-sight is **symmetric/reciprocal**. If A can see B, then B can see A (same sight line, opposite directions).

### Algorithm Steps

1. **Extract elevations** at start point (observer) and end point (target) from DEM
2. **Sample the path** between points at DEM cell resolution
3. **For each sample point:**
   - Calculate **expected elevation** along straight 3D line (linear interpolation)
   - Extract **actual terrain elevation** from DEM
   - If `actual_elevation > expected_elevation` → **blocked**, return false
4. If no blocking terrain found → **visible**, return true

### C++ Implementation (with GDAL)

```cpp
#include <gdal_priv.h>
#include <cmath>
#include <vector>

bool lineOfSight(GDALDataset* demDataset,
                 double x1, double y1,  // observer coordinates
                 double x2, double y2,  // target coordinates
                 double observerHeight = 0.0,  // height offset for observer
                 double targetHeight = 0.0)     // height offset for target
{
    GDALRasterBand* band = demDataset->GetRasterBand(1);
    
    // Get geotransform for coordinate conversion
    double geoTransform[6];
    demDataset->GetGeoTransform(geoTransform);
    
    // Helper function to get elevation at a geographic coordinate
    auto getElevation = [&](double x, double y) -> double {
        // Convert geographic coords to pixel coords
        double pixel = (x - geoTransform[0]) / geoTransform[1];
        double line = (y - geoTransform[3]) / geoTransform[5];
        
        float elevation;
        band->RasterIO(GF_Read,
                      static_cast<int>(pixel),
                      static_cast<int>(line),
                      1, 1, &elevation, 1, 1, GDT_Float32, 0, 0);
        return elevation;
    };
    
    // Get elevations at endpoints (with height offsets)
    double z1 = getElevation(x1, y1) + observerHeight;
    double z2 = getElevation(x2, y2) + targetHeight;
    
    // Calculate horizontal distance and determine sample count
    double dx = x2 - x1;
    double dy = y2 - y1;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    // Sample at DEM cell resolution for accuracy
    double cellSize = std::abs(geoTransform[1]);
    int numSamples = static_cast<int>(distance / cellSize) + 1;
    
    // Check each sample point along the line
    for (int i = 1; i < numSamples - 1; i++) {
        // Parameter t: 0 at start, 1 at end
        double t = static_cast<double>(i) / (numSamples - 1);
        
        // Interpolate geographic position
        double x = x1 + t * dx;
        double y = y1 + t * dy;
        
        // Expected elevation along straight 3D sight line
        double expectedZ = z1 + t * (z2 - z1);
        
        // Actual terrain elevation at this point
        double actualZ = getElevation(x, y);
        
        // If terrain is higher than sight line, view is blocked
        if (actualZ > expectedZ) {
            return false;  // Not visible
        }
    }
    
    return true;  // Visible - no blocking terrain found
}
```

### Algorithm Complexity

- **Time per LOS check:** O(distance / cell_size)
  - For 12-mile visibility range with 30m DEM cells: ~644 samples per check
- **Total computation:** O(N × M × samples_per_ray)
  - N = road points, M = ignition points
  - Can be parallelized across road points (independent checks)



---

## Future Enhancements

1. **Earth Curvature:** For distances > 10 miles, incorporate earth curvature correction
2. **Atmospheric Refraction:** Account for light refraction in atmosphere (typically ~15% increase in visible distance)
3. **Vegetation/Buildings:** Incorporate DSM (Digital Surface Model) instead of bare-earth DEM
4. **Temporal Factors:** Consider smoke plume height, time-of-day visibility variations
5. **Uncertainty Quantification:** Account for DEM vertical accuracy, ignition position uncertainty

---

*Document Version: 1.0*  
*Last Updated: 2026-02-02*