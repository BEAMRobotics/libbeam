import numpy as np
import cv2
import glob
import sys
import json
from collections import OrderedDict
import datetime

def main():
    if(len(sys.argv) != 5):
        print("Invalid number of arguments.")
        print("Usage:\n python Calibrator.py [camera_model] [height] [width] [frame_id]")
        sys.exit()
    method = sys.argv[1]
    path = raw_input("Enter path to folder containing images: ")
    height = int(sys.argv[2])
    width = int(sys.argv[3])
    frame_id = sys.argv[4]
    if method == "equidistant":
        calibrateFisheye(path, height, width, frame_id)
    elif method == "radtan":
        calibrateRadtan(path, height, width, frame_id)

def calibrateRadtan(path, height, width, frame_id):
    CHECKERBOARD = (height,width)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objpoints = [] 
    imgpoints = []
    images = glob.glob(path + '/*.png')
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,None)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)
            img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2,ret)
            cv2.imshow('img',img)
            cv2.waitKey(500)
    cv2.destroyAllWindows()
    ret, K, D, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    intrinsics = [K[0,0], K[1,1], K[0,2], K[1,2]]
    coeffs = [D[0][0], D[1][0], D[2][0], D[3][0]]
    model = "radtan"
    saveToJson(intrinsics, coeffs, model, _img_shape, frame_id)


def calibrateFisheye(path, height, width, frame_id):
    CHECKERBOARD = (height,width)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW
    objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    _img_shape = None
    objpoints = [] 
    imgpoints = []
    images = glob.glob(path + '/*.png')
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        if _img_shape == None:
            _img_shape = img.shape[:2]
        else:
            assert _img_shape == img.shape[:2], "All images must share the same size."
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,None)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)
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