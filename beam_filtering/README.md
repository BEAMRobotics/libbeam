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
    * The VoxelDownsamplingFilter downsamples a point cloud by comparing points to a voxel grid. If a voxel contains points it is replaced with a point in its centroid.  This is currently implemented as a wrapper over PCL's voxel grid filter. If any of the voxel dimensions are set to zero, or anything below 1mm, then no filtering is performed.
    * If the point cloud volume is great or the voxel size is small, integer overflow can occur.  Pcl uses 32 bit integers for building the voxel grid.  . Then the voxel grid filter is applied to all individual point clouds and recombined at the end.
    * In order to break up the point coud into  
    * WIP