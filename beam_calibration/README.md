# beam_calibration

This module contains the multiple different camera models and distortion models used for BEAMRobotics.

## Install instructions

As of May 2020, libbeam requires c++ version 17 which requires g++/gcc 7.
To install g++/gcc 7 on Ubuntu 16.04, run:

```
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-7 g++-7
```

Then, get your current version using

```
g++ -v
gcc -v
```

And replace the symlinks:

```
sudo rm /usr/bin/gcc
sudo rm /usr/bin/g++
sudo ln -s /usr/bin/gcc-7 /usr/bin/gcc
sudo ln -s /usr/bin/g++-7 /usr/bin/g++
```

## Supported Camera Models

1. **Radtan** Intrinsics: [fx, fy, cx, cy, k1, k2, p1, p2]
2. **Double Sphere** Intrinsics: [fx, fy, cx, cy, eps, alpha]
3. **Kannala Brandt** Intrinsics: [fx, fy, cx, cy, k1, k2, k3, k4]
4. **Ladybug** Intrinsics: [fx, fy, cy, cx]
    * This camera model is a special case using the Ladybug SDK. It cannot be instantiated by our JSON calibration file and must be instantiated on its own using the standard .conf file

### Info
The radtan model is equivalent to OpenCV's normal camera model, and Kannala Brandt is equivalent to OpenCV's fisheye model.

### Example JSON calibration file

```
{
  "date": "2019-05-29",
  "method": "opencv",
  "camera_type": "KANNALABRANDT",
  "image_width": 2048,
  "image_height": 1536,
  "frame_id": "F1_link",
  "intrinsics": [
    783.44463219576687,
    783.68479107567089,
    996.34300258081578,
    815.47561902246832,
    0.0052822823133193853,
    0.0069435221311202099,
    -0.0025332897347625323,
    -0.0013896892385779631
  ]
}
```


## Calibrate.py Usage

`python Calibrator.py [camera_model] [height] [width] [frame_id]`

camera_model = ["equidistant", "radtan"]

height/width = dimensions of calibration board

frame_id = name of camera being calibrated

## References

1. J. Kannala and S. Brandt (2006). A Generic Camera Model and Calibration Method for Conventional, Wide-Angle, and Fish-Eye Lenses, IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. 28, no. 8, pp. 1335-1340

2. Usenko, V., Demmel, N., & Cremers, D. (2018). The Double Sphere Camera Model. 2018 International Conference on 3D Vision (3DV). doi:10.1109/3dv.2018.00069

3. B. Khomutenko, G. Garcia, and P. Martinet. (2016) An enhanced unified camera model. IEEE Robotics and Automation Letters.
