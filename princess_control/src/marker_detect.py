#!/usr/bin/env python
import numpy as np
import cv2
import roslib
import rospy
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension,Int8MultiArray
from geometry_msgs.msg import Point, Quaternion
from headcam_node.msg import Int16Array
from princess_control.msg import FiducialArray, FiducialMsg



pub = rospy.Publisher('marker_translation',Point, queue_size = 1)
fiducial_pub = rospy.Publisher('fiducials',FiducialArray, queue_size = 1)
id_pub = rospy.Publisher('fiducial_ids',Int16Array, queue_size = 1)
rospy.init_node('marker_detect')
rate = rospy.Rate(50)

#
# cameraMatrix = np.array([[3.11921895e+03, 0.00000000e+00, 6.88348512e+02],
#        [0.00000000e+00, 3.02218437e+03, 3.08658911e+02],
#        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
#
# distCoeffs = np.array([[ 1.76870275e+00, -3.28187890e+01, -2.74154944e-02, -3.76760143e-02,  1.73787203e+02]])


print("Getting camera parameters...")
cameraMatrix = np.asarray(rospy.get_param('headcam_fiducial/headcam_coeffs/camera_matrix'))
distCoeffs = np.asarray(rospy.get_param('headcam_fiducial/headcam_coeffs/dist_coeff'))
print("Parameters have been retrieved")
print("Camera matrix: ")
print(cameraMatrix)
print("Distortion Coefficients: ")
print(distCoeffs)

print("##############################################################")
cap = cv2.VideoCapture(1)
cap.set(3,1280)
cap.set(4,720)
#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()
print("Ready to read fiducials...")

while not rospy.is_shutdown():
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
        ids_to_publish = Int8MultiArray(data=ids)
        id_pub.publish(ids)
        #frame = cv2.aruco.drawDetectedMarkers(frame,corners, ids)
        rvecs, tvecs, other = cv2.aruco.estimatePoseSingleMarkers(corners, 0.635, cameraMatrix, distCoeffs)
        data_out = []
        for x in range(len(tvecs)):
            frame = cv2.aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvecs[x], tvecs[x], 5)
            single_fiducial = FiducialMsg()
            tvec = tvecs[x]
            rvec = rvecs[x]
            point_msg = Point(float(tvec[0][0]),float(tvec[0][1]),float(tvec[0][2]))
            pub.publish(point_msg)
            single_fiducial.ID = ids[x]
            single_fiducial.pose.position = Point(float(tvec[0][0]),float(tvec[0][1]),float(tvec[0][2]))
            single_fiducial.pose.orientation = Quaternion(float(rvec[0][0]),float(rvec[0][1]),float(rvec[0][2]), 1.0)
            data_out += (single_fiducial,)
        #print("########################################")

        fiducial_pub.publish(data_out)



    # Our operations on the frame come here

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
