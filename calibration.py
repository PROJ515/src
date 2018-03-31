import numpy as np
import cv2
import glob
import time
import datetime

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((6*9, 3), np.float32)
objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.
images = []
cap = cv2.VideoCapture(1)
total = 0
progress = 0
found = 0
while(True):
    ret, frame = cap.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow('frame', frame)
    # frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_CUBIC)
    images.append(frame)
    total = total+1
    print(total, "images taken")

    if cv2.waitKey(100) & 0xFF == ord('q'):

        cap.release()
        cv2.destroyAllWindows()
        t0 = time.time()
        for img in images:
            ret, corners = cv2.findChessboardCorners(img, (9, 6), None)
            progress = progress + 1
            # cv2.imshow('frame', img)
            # cv2.waitKey(1)
            t1 = time.time()
            percent = round(progress/total*100, 1)
            totalttime = (t1-t0)*100/percent
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

        cv2.destroyAllWindows()
        exit()
