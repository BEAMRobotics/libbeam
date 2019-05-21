# beam_calibration

This module contains the multiple different camera models and distortion models used for BEAMRobotics.

## Supported Models

1. Pinhole Camera Model
    * With Radial-Tangential distortion
    * With Equidistant distortion

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

