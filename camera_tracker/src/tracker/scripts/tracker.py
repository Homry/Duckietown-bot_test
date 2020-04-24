#!/usr/bin/env python
import cv2
import cv2.aruco as aruco
import rospy
import numpy as np
from sensor_msgs.msg import Image

MARKER_TYPE = aruco.DICT_5X5_250
CameraMtx = [682.42294095, 0.0, 469.948484581, 0.0, 681.44128234, 333.951368692, 0.0, 0.0, 1.0]
DistortionCoeff = [0.0692332572094, -0.170327984628, 0.00436794532634, 0.00496921804668, 0.0]
markerLength = 0.1

def bgr_from_jpg(image_msg):
#Специально для теста передаю только в эту функцию
    print(type(image_msg)) 
    s = np.fromstring(image_msg.data, np.uint8)   #поле дата конверитровать в список
    bgr = cv2.imdecode(s, cv2.IMREAD_COLOR)    #декодировать спиок
    print(type(bgr))  #в итоге получается пустой тип
    ##if bgr is None:
      ##  msg = 'Could not decode image (cv2.imdecode returned None). '
       ## msg += 'This is usual a sign of data corruption.'
       ## raise ValueError(msg)
    return bgr

def tracker(image_msg):
    cv_img = bgr_from_jpg(image_msg.data)
    arucoDictionacy = aruco.Dictionary_get(MARKER_TYPE)
    parameters = aruco.DetectorParameters_create()
    #image_sub = np.array(image_sub)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_img, arucoDictionacy, parameters=parameters, cameraMatrix=CameraMtx, distCoeff=DistortionCoeff)
    for j in range(0, len(ids)):
        rvec, tvec, objPoints = aruco.estimatePoseSingleMarkers(corners[j], markerLength, CameraMtx, DistortionCoeff)
        aruco.drawDetectedMarkers(cv_img, corners, ids)
        aruco.drawAxis(image_sub, CameraMtx, DistortionCoeff, rvec, tvec, 0.1)



if __name__ == '__main__':
    #CameraMtx = [682.42294095, 0.0, 469.948484581, 0.0, 681.44128234, 333.951368692, 0.0, 0.0, 1.0]
    #DistortionCoeff = [0.0692332572094, -0.170327984628, 0.00436794532634, 0.00496921804668, 0.0]
    rospy.init_node('tracker')
    #markerLength = rospy.get_param('~size')
    image_sub = rospy.Subscriber("camera_image", Image, bgr_from_jpg)

    rospy.spin()
