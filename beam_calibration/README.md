# beam_calibration

This module contains the multiple different camera models and distortion models used for BEAMRobotics.

## Supported Models

1. **Pinhole Camera Model** [fx, fy, cx, cy]
    * **With Radial-Tangential distortion** [k1, k2, k3, r1, r2]
    * **With Equidistant distortion** [k1, k2, k3, k4]

### Example JSON calibration file

```
{
  "camera_type": "pinhole",
  "date": "2018_12_20",
  "method": "matlab",
  "calibration": [
    {
      "image_width": 2048,
      "image_height": 1536,
      "frame_id": "F1_link",
      "distortion_model": "radtan",
      "intrinsics": [
        2338.485520924695,
        2333.0927287230647,
        1002.8381839138167,
        784.1498440053573
      ],
      "distortion_coefficients": [
        -0.2294924671994032,
        0.18008566892263364,
        0,
        -0.0005326294604360527,
        -0.0004378797791316729
      ]
    }
  ]
}
```

## References

1. J. Kannala and S. Brandt (2006). A Generic Camera Model and Calibration Method for Conventional, Wide-Angle, and Fish-Eye Lenses, IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. 28, no. 8, pp. 1335-1340

2. Usenko, V., Demmel, N., & Cremers, D. (2018). The Double Sphere Camera Model. 2018 International Conference on 3D Vision (3DV). doi:10.1109/3dv.2018.00069
