# !/usr/bin/env python
 
import cv2 as cv2
import numpy as np
import os
import glob
WIDTH = 640
HEIGHT = 480
 
# Defining the dimensions of checkerboard
CHECKERBOARD = (5,8)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 3, 0.1)
 
# Creating vector to store vectors of 3D points for each checkerboard image
objpoints = []
# Creating vector to store vectors of 2D points for each checkerboard image
imgpoints = []
 
 
# Defining the world coordinates for 3D points
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
prev_img_shape = None
 
# Extracting path of individual image stored in a given directory
images = glob.glob('./images/*.jpg')
for fname in images:
    img = cv2.imread(fname)
    resized = cv2.resize(img, (WIDTH, HEIGHT), interpolation = cv2.INTER_AREA)
    gray = cv2.cvtColor(resized,cv2.COLOR_BGR2GRAY)
    print(gray.shape)
    chessboard_dim = (5, 8)
    found_all, corners = cv2.findChessboardCorners(img, chessboard_dim)
    cv2.drawChessboardCorners(img, chessboard_dim, corners, found_all)
    # cv2.imshow("Chessboard with corners", img)
    # cv2.waitKey()

    # Find the chess board corners
    # If desired number of corners are found in the image then ret = true
    # ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
     
    """
    If desired number of corner are detected,
    we refine the pixel coordinates and display
    them on the images of checker board
    """
    print("start")
    if found_all == True:
        print("sucess")
        objpoints.append(objp)
        # refining pixel coordinates for given 2d points.
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
         
        imgpoints.append(corners2)

"""
Performing camera calibration by
passing the value of known 3D points (objpoints)
and corresponding pixel coordinates of the
detected corners (imgpoints)
"""
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (HEIGHT, WIDTH), None, None)
 

# Refining the camera matrix using parameters obtained by calibration
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (WIDTH, HEIGHT), 0.1, (WIDTH, HEIGHT))
 
 # Camera matrix : 
# [[1.51216159e+03 0.00000000e+00 1.18554956e+03]
#  [0.00000000e+00 1.50860716e+03 8.21240275e+02]
#  [0.00000000e+00 0.00000000e+00 1.00000000e+00]]


# dist :
# [[-3.48574010e-01  1.54178739e-01  2.07261833e-03 -1.67209560e-04
#   -3.75548420e-02]]

# newcameramtx:
# [[1.38142977e+03 0.00000000e+00 1.25862188e+03]
#  [0.00000000e+00 1.38097924e+03 7.73731516e+02]
#  [0.00000000e+00 0.00000000e+00 1.00000000e+00]]



mapx, mapy=cv2.initUndistortRectifyMap(mtx,dist,None, newcameramtx, (WIDTH, HEIGHT), 5)

img1 = cv2.imread("reference.jpg")
img2 = cv2.imread("./Help_images/chessboard_ideal.png")
img1 = cv2.remap(img1, mapx, mapy, cv2.INTER_LINEAR)
cv2.imwrite("before_2.jpg", img1)

# Find corners
ret1, corners1 = cv2.findChessboardCorners(img1, (5, 8))
ret2, corners2 = cv2.findChessboardCorners(img2, (5, 8))

# Find ipm-transform
H, _ = cv2.findHomography(corners1, corners2)
# Calculate remap-coordinates
mapx, mapy = cv2.initUndistortRectifyMap(mtx,dist,None, np.matmul(H, newcameramtx), (WIDTH, HEIGHT), 5)

with open('./Matrices/mapx.txt', 'w') as f:
    for row in mapx:
        for col in row:
            f.write("%s " % (col))
        f.write("\n")
f.close()

with open('./Matrices/mapy.txt', 'w') as f:
    for row in mapy:
        for col in row:
            f.write("%s " % (col))
        f.write("\n")
f.close()


## Test matrix
img3 = cv2.imread("reference.jpg")
img3 = cv2.remap(img3, mapx, mapy, cv2.INTER_LINEAR)
cv2.imwrite("combined_2.jpg", img3)
