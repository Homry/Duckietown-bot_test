#!/usr/bin/env python
import cv2.aruco as aruco
import rospy
import numpy as np
from sensor_msgs.msg import Image 

MARKER_TYPE = aruco.DICT_5X5_250

def tracker(image_sub, markerLength, CameraMtx, DistortionCoeff):
    arucoDictionacy = aruco.Dictionary_get(MARKER_TYPE)
    parametrs = aruco.DetectorParameters_create()
    image_sub = np.array(image_sub)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(image_sub, arucoDictionacy, parametrs=parametrs, cameraMatrix=CameraMtx, distCoeff=DistortionCoeff)
    for j in range(0, len(ids)):
        rvec, tvec, objPoints = aruco.estimatePoseSingleMarkers(corners[j], markerLength, CameraMtx, DistortionCoeff)
        aruco.drawDetectedMarkers(image_sub, corners, ids)
        aruco.drawAxis(image_sub, CameraMtx, DistortionCoeff, rvec, tvec, 0.1)



if __name__ == '__main__':
    CameraMtx = [682.42294095, 0.0, 469.948484581, 0.0, 681.44128234, 333.951368692, 0.0, 0.0, 1.0]
    DistortionCoeff = [0.0692332572094, -0.170327984628, 0.00436794532634, 0.00496921804668, 0.0]
    rospy.init_node('tracker')
    markerLength = rospy.get_param('~size')
    image_sub = rospy.Subscriber("camera_image", Image)
    tracker(image_sub, markerLength, CameraMtx, DistortionCoeff)
    rospy.spinOnes()
