"""
File: cam.py
Author: Thomas Woodruff
Date: 11/11/19
Revision: 0.1
Description: Utility functions for camera
             object.
"""

import numpy as np
import cv2

class camera():
    '''
    class to handle camera operations
    '''
    def __init__(self, port):
        # START CAMERA
        self.cam = cv2.VideoCapture(port) #,cv2.CAP_FFMPEG)
        self.fps = self.cam.get(cv2.CAP_PROP_FPS)
        self.running = True

    def run(self):
        # CAPTURE A FRAME AND UNDISTORT
        if self.running:
            ret,frame = self.cam.read()
            self.frame = undistortFishEye(frame)
            #self.frame = projective_warp(frame)

    def update(self):
        # RETURN FRAME
        return self.frame

    def show(self):
        # DISPLAY AN IMAGE/VIDEO STREAM
        k = cv2.waitKey(1)
        if k == ord('q') & 0xFF:
            self.shutdown()

    def shutdown(self):
        # CLOSE CAMERA
        self.running = False
        self.cam.release()
        cv2.destroyAllWindows()


def rotate(image, angle):
    '''
    rotate an image by the given angle and return
    the rotated image
    '''
    # GET IMAGE DIMENSIONS
    (h, w) = image.shape[:2]
    (cX, cY) = (w / 2, h / 2)

    # DETERMINE AFFINE TRANSFORMATION
    M = cv2.getRotationMatrix2D((cX, cY), -angle, 1.0)

    # APPLY TRANSFORMATION AND RETURN
    return cv2.warpAffine(image, M, (w, h))

def projective_warp(img):
    dst_size=(640,480)
    src=np.float32([(0.25,0.5),(0.80,0.5),(0,0),(1,0)])
    dst=np.float32([(0,1), (1,1), (0,0), (1,0)])

    img_size = np.float32([img.shape[1],img.shape[0]])
    src = src * img_size
    dst = dst * img_size #np.float32(dst_size)

    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, dst_size)
    return warped

def getDistortionParams():
    '''
    credit: https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0
    '''
    import os
    import glob

    CHECKERBOARD = (6,9)

    subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
    calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW

    objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

    _img_shape = None
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    images = glob.glob('/home/pi/Documents/KU_ARC/pics/*.jpg')

    for fname in images:
        img = cv2.imread(fname)
        if _img_shape == None:
            _img_shape = img.shape[:2]
        else:
            assert _img_shape == img.shape[:2], "All images must share the same size."
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
            imgpoints.append(corners)

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

    print("Found " + str(N_OK) + " valid images for calibration")
    print("DIM=" + str(_img_shape[::-1]))
    print("K=np.array(" + str(K.tolist()) + ")")
    print("D=np.array(" + str(D.tolist()) + ")")


def undistortFishEye(image):
    '''
    credit: https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0
    '''
    # CONSTANTS FROM CALIBRATION
    #computer
    # K = np.array([[351.485, 0.0, 320.894],[0.0,351.058,246.049],[0.0,0.0,1.0]])
    # D = np.array([[-0.0707],[-0.3549],[0.9277],[-0.6842]])
    #rpi
    K = np.array([[472.892,0.0,319.364],[0.0,474.471,271.360],[0.0,0.0,1.0]])
    D = np.array([[-0.1570],[0.6792],[-2.1645],[2.1713]])

    # GET IMAGE SIZE
    h,w = image.shape[:2]
    DIM = (w,h)

    # PERFORM DISTORTION CORRECTION
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    return undistorted_img


if __name__ == "__main__":
    getDistortionParams()
    #cam = camera(0)
    #cam.run()
    #undistort = cam.update()
    #cv2.imshow("undistorted",undistort)
    #cam.show()
