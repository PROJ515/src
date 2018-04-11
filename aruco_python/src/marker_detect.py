import numpy as np
import cv2
import roslib
import rospy

from geometry_msgs.msg import Point


pub = rospy.Publisher('marker_translation',Point, queue_size = 1)
rospy.init_node('marker_detect')
rate = rospy.Rate(50)
cameraMatrix = np.array([[3.11921895e+03, 0.00000000e+00, 6.88348512e+02],
       [0.00000000e+00, 3.02218437e+03, 3.08658911e+02],
       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

distCoeffs = np.array([[ 1.76870275e+00, -3.28187890e+01, -2.74154944e-02, -3.76760143e-02,  1.73787203e+02]])

cap = cv2.VideoCapture(0)
cap.set(3,1280)
cap.set(4,720)
#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    rvecs=[]
    tvecs=[]
    tvec = []
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dictionary)
    #print(corners)
    #print(rejectedImgPoints)

    if len(corners) > 0:
        #frame = cv2.aruco.drawDetectedMarkers(frame,corners, ids)
        #print(corners)
        rvecs, tvecs, other = cv2.aruco.estimatePoseSingleMarkers(corners, 0.635, cameraMatrix, distCoeffs)
        #print(ret)
        #print(rvecs)
        #print(tvecs)
        for x in range(len(tvecs)):
            frame = cv2.aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvecs[x], tvecs[x], 5)

            tvec = tvecs[x]
            print tvec[0][0]
            point_msg = Point(float(tvec[0][0]),float(tvec[0][1]),float(tvec[0][2]))
            pub.publish(point_msg)

    # Our operations on the frame come here

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()