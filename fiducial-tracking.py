import numpy as np
import cv2

cameraMatrix = np.array([[947.84099202, 0, 313.00796217],
                         [0, 949.76583415, 259.16939978],
                         [0, 0, 1]])

distCoeffs = np.array([[-9.27200660e-01, 8.69447209e+00, 1.80066794e-02, 7.26126694e-02, -5.90327157e+01]])

cap = cv2.VideoCapture(1)

#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    rvecs=[]
    tvecs=[]

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dictionary)
    #print(corners)
    #print(rejectedImgPoints)

    if len(corners) > 0:
        #frame = cv2.aruco.drawDetectedMarkers(frame,corners, ids)
        #print(corners)
        rvecs, tvecs, other = cv2.aruco.estimatePoseSingleMarkers(corners, 6, cameraMatrix, distCoeffs)
        #print(ret)
        #print(rvecs)
        #print(tvecs)
        for x in range(len(tvecs)):
            frame = cv2.aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvecs[x], tvecs[x], 5)

    # Our operations on the frame come here

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
