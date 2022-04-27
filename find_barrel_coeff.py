# !/usr/bin/env python

import cv2 as cv2
import numpy as np
import glob
WIDTH = 640
HEIGHT = 480

# Defining the dimensions of checkerboard
CHECKERBOARD = (5, 8)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 3, 0.1)

# Creating vector to store vectors of 3D points for each checkerboard image
objpoints = []
# Creating vector to store vectors of 2D points for each checkerboard image
imgpoints = []


# Defining the world coordinates for 3D points
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
prev_img_shape = None

# Extracting path of individual image stored in a given directory
images = glob.glob('./calibration_640_420/*.jpg')
# images = glob.glob('./images/*.jpg')
# WIDTH = 2592
# HEIGHT = 1944

for fname in images:
    img = cv2.imread(fname)
    # resized = cv2.resize(img, (WIDTH, HEIGHT), interpolation=cv2.INTER_NEAREST)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    chessboard_dim = (5, 8)
    found_all, corners = cv2.findChessboardCorners(img, chessboard_dim)
    cv2.drawChessboardCorners(img, chessboard_dim, corners, found_all)

    """
    If desired number of corner are detected,
    we refine the pixel coordinates and display
    them on the images of checker board
    """
    print("start")
    if found_all:
        print("sucess")
        objpoints.append(objp)
        # refining pixel coordinates for given 2d points.
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
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
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (WIDTH, HEIGHT), 5)

img1 = cv2.imread("./calibration_640_420/reference.jpg")
img2 = cv2.imread("./Chessboard_ideal/chessboard_640_480.png")
img1 = cv2.remap(img1, mapx, mapy, cv2.INTER_LINEAR)

# Find corners
ret1, corners1 = cv2.findChessboardCorners(img1, (5, 8))
ret2, corners2 = cv2.findChessboardCorners(img2, (5, 8))

# Find ipm-transform
H, _ = cv2.findHomography(corners1, corners2)
# Calculate remap-coordinates
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, np.matmul(H, newcameramtx), (WIDTH, HEIGHT), 5)

mask = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
mask[:]=(0, 0, 0)
mask = cv2.warpPerspective(mask, H, (img1.shape[1], img1.shape[0]), borderMode=cv2.BORDER_CONSTANT, borderValue=(255, 255, 255))

mask =  cv2.GaussianBlur(mask, (29, 29), 60)

mask = cv2.resize(mask, (320, 240))
cv2.imwrite("mask.png", mask)

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
img3 = cv2.imread("./calibration_640_420/reference.jpg")
img3 = cv2.remap(img3, mapx, mapy, cv2.INTER_LINEAR)
cv2.imwrite("combined_2.jpg", img3)
