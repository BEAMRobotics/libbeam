import numpy as np
import cv2
import glob
import sys
import json
from collections import OrderedDict
import datetime

def main():
    if(len(sys.argv) != 8):
        print("Invalid number of arguments.")
        print("Usage:\n python Calibrator.py [path_to_images] [camera_model] [height] [width] [frame_id] [image_extension] [convert_to_gray]")
        print("\n path_to_images: full path to images directory")
        print("\n camera_model: equidistant (KB - fisheye) or radtan (pinhole with radtan dist)")
        print("\n height: checkerboard height")
        print("\n width: checkerboard width")
        print("\n frame_id: to assign to intrinsics json")
        print("\n image_extension: .jpg, .jpeg, or .png")
        print("\n convert_to_gray: set to true if images are not mono or grayscale")
        sys.exit()
    method = sys.argv[1]
    path = sys.argv[2]
    height = int(sys.argv[3])
    width = int(sys.argv[4])
    frame_id = sys.argv[5]
    image_extension = sys.argv[6]
    convert_to_gray = bool(sys.argv[7])
    if method == "equidistant":
        calibrateFisheye(path, height, width, frame_id, image_extension, convert_to_gray)
    elif method == "radtan":
        calibrateRadtan(path, height, width, frame_id, image_extension, convert_to_gray)

def calibrateRadtan(path, height, width, frame_id, image_extension, convert_to_gray):
# termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((height*width,3), np.float32)
    objp[:,:2] = np.mgrid[0:width,0:height].T.reshape(-1,2)
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    images = glob.glob(path + '/*' + image_extension)
    gray = None
    _img_shape = None
    for fname in images:
        print(fname)
        img = cv2.imread(fname)
        gray = img
        if convert_to_gray:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if _img_shape == None:
            _img_shape = img.shape[:2]
        else:
            assert _img_shape == img.shape[:2], "All images must share the same size."

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (height,width), None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)
            # Draw and display the corners
            cv2.drawChessboardCorners(img, (height,width), corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()
    ret, K, D, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

    intrinsics = [K[0,0], K[1,1], K[0,2], K[1,2]]
    coeffs = [D[0][0], D[0][1], D[0][2], D[0][3], D[0][4]]
    model = "radtan"
    saveToJson(intrinsics, coeffs, model, _img_shape, frame_id)


def calibrateFisheye(path, height, width, frame_id, image_extension, convert_to_gray):
    CHECKERBOARD = (height,width)
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

    _img_shape = None
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    images = glob.glob(path + '/*' + image_extension)
    print('Looking for images in: ' + images)

    for fname in images:
        print('reading image: ' + fname)
        img = cv2.imread(fname)
        gray = img
        if convert_to_gray:
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        if _img_shape == None:
            _img_shape = img.shape[:2]
        else:
            assert _img_shape == img.shape[:2], "All images must share the same size."

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2,ret)
            cv2.imshow('img',img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()

    N_OK = len(objpoints)
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
    rms, _, _, _, _ = \
        cv2.fisheye.calibrate(
            objpoints,
            imgpoints,
            gray.shape[::-1],
            K,
            D,
            rvecs,
            tvecs,
            calibration_flags,
            (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
        )
    intrinsics = [K[0,0], K[1,1], K[0,2], K[1,2]]
    coeffs = [D[0][0], D[1][0], D[2][0], D[3][0]]
    model = "equidistant"
    saveToJson(intrinsics, coeffs, model, _img_shape, frame_id)

def saveToJson(intrinsics, coeffs, model, dims, frame_id):
    date = str(datetime.date.today())
    x = OrderedDict()
    x["camera_type"] = "pinhole"
    x["date"]= date
    x["method"] = "opencv"
    x["calibration"] = [OrderedDict()]
    x["calibration"][0]["image_width"] = dims[1]
    x["calibration"][0]["image_height"] = dims[0]
    x["calibration"][0]["frame_id"] = frame_id
    x["calibration"][0]["distortion_model"] = model
    x["calibration"][0]["intrinsics"] = intrinsics
    x["calibration"][0]["distortion_coefficients"] = coeffs
    with open('calibration.json', 'w') as outfile:
        json.dump(x, outfile)


if __name__ == "__main__":
    main()