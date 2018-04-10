import cv2
import numpy as np
import glob
import time
import datetime
cap = cv2.VideoCapture(0)
cap.set(3,1280)
cap.set(4,720)
ret, img = cap.read()
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
imgs = []
x = 0
total = 0
progress = 0
found = 0
while ret:
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	ret1, corners = cv2.findChessboardCorners(gray, (8,6),None)
	if ret1 == True:
		imgs.append(gray)
		objpoints.append(objp)

		corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
		imgpoints.append(corners2)

		# Draw and display the corners
		img = cv2.drawChessboardCorners(img, (8,6), corners2,ret)
		cv2.imshow('corners',img)
		print (total, 'images saved')
		total = total + 1
		cv2.waitKey(300)

	#cv2.imshow('image', corners)
	if cv2.waitKey(5) & 0xFF == ord('p') :
		name = 'calib' + str(x) + '.jpg' 
		cv2.imwrite(name,img)
		x = x + 1

	if cv2.waitKey(5) & 0xFF == ord('q') :
		break
	ret ,img = cap.read()

	if cv2.waitKey(100) & 0xFF == ord('d'):

		cap.release()
		cv2.destroyAllWindows()
		t0 = time.time()
		for img in imgs:
		    ret, corners = cv2.findChessboardCorners(img, (8, 6), None)
		    progress = progress + 1
		    # cv2.imshow('frame', img)
		    # cv2.waitKey(1)
		    t1 = time.time()
		    percent = round(progress/total*100, 1)
		    totalttime = (t1-t0)*100/(percent+1)
		    timeremaining = int(totalttime-(t1-t0))
		    if ret == True:
		        objpoints.append(objp)
		        corners2 = cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), criteria)
		        imgpoints.append(corners2)
		        # print(percent, "%", progress, "of", total, "- found")
		        print(percent, "% eta", timeremaining, "s - grid found")
		        found = found + 1
		    else:
		        # print(percent, "%", progress, "of", total, "- not found")
		        print(percent, "% eta", timeremaining, "s - grid not found")
		print("Out of", total, "images, a checkerboard was found in", found, "images")
		eta = int(4.01+(-0.816*found)+(0.0395*(found**2))+(5.5*(10**-4)*(found**3)))
		print("Please wait...", "eta =", str(datetime.timedelta(seconds=eta)))
		t0 = time.time()
		ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img.shape[::-1], None, None)
		t1 = time.time()
		timeperframe = (t1-t0)/found
		print("Time taken", int(t1-t0))
		print(timeperframe, "seconds per frame")
		print(ret, mtx, dist, rvecs, tvecs)
		exit()