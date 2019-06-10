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
