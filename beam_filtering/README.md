# beam_filtering

This module contains special point cloud filters used for BEAMRobotics.

## Filters

1. **CropBox**:
    * This filter is very similar to pcl's cropbox filter
    * PCL's cropbox filter seems to have some weird bugs where the filter works
    differently on different machines, so this is just a simple implementation
    to avoid these issues

2. **DROR**:
    * Dynamic Radius Outlier Filter
    * Based on Nick and Steve's paper at CRV 2018: https://ieeexplore.ieee.org/abstract/document/8575761

3. **VoxelDownsample**:
    * Voxeldownsample is a filter for downsampling a pointcloud using a voxel grid. Points in each voxel are replaced with a single point in their centroid.  This is currently implemented as a wrapper over PCL's voxel grid filter.
    * PCL stores the number of voxels using 32bit integers, so integer overflow protection is implemented. Clouds are split up to be filtered piecewise if overflow is predicted. Midpoint splitting is used in a recursive pattern until overflow is not predicted for any piece.  The output cloud is the concatenation of each piece filtered.
    * VoxelDownsample currently supports the PointXYZ point type with templating coming soon.