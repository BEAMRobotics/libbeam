# beam_cv

This module contains functions and classes commonly used in image processing/computer vision.

## geometry

Common 3D computer vision geometric algorithms are implemented here to work with our camera models:
1. RelativePoseEstimator: Estimates the relative pose between two frames
2. Triangulation: linear triangulation of pixels between two frames
3. AbsolutePoseEstimator: Estimates absolute pose of camera given 3d points and corresponding pixels

## descriptors

Class structure for image descriptor extractors, descriptors implemented:
1. ORB
2. BRISK
3. SIFT


## detectors

Class structure for image detectors, detectors implemented:
1. ORB
2. FAST
3. SIFT


## matchers

Class structure for feature matchers, implemented macthers:
1. FLANN

## tracker

Object to provide a track of features given a set of images