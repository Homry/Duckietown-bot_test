#!/usr/bin/env python
import cv2
import cv2.aruco as aruco
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


MARKER_TYPE = aruco.DICT_5X5_250
CameraMtx = [682.42294095, 0.0, 469.948484581, 0.0, 681.44128234, 333.951368692, 0.0, 0.0, 1.0]
CameraMtx = np.array(CameraMtx)
DistortionCoeff = [0.0692332572094, -0.170327984628, 0.00436794532634, 0.00496921804668, 0.0]
DistortionCoeff = np.array(DistortionCoeff)
markerLength = 0.1

def from_ImageMsg_to_cvImage(image_msg):
    bridge = CvBridge()
    #print(type(image_msg))
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
    print(type(cv_image))
    #gray_cv_image = cv2.cvtColor(cv_image, cv2.COLOR_HSV2BGR)
    print(cv_image.shape)
    #print(type(CameraMtx))
    return cv_image

def get_coordinates(rvec, tvec, objPoints, id): #calculate coordinates in the camera system, returns a list with time, id, coordinates and angle
    coordinate = []
    data = []
    rvec = np.array(rvec).reshape((1, 3))
    Rot_mat, _ = cv2.Rodrigues(np.asarray(rvec))
    Rot_mat = Rot_mat.T
    centr_marker = np.array([[0, 0, 0]])
    # convert from the marker system to the camera system for each angle
    XYZ_centr = Rot_mat.dot(centr_marker.T) + np.array(tvec[0]).T
    p1 = Rot_mat.dot(objPoints[0].T) + np.array(tvec[0]).T  # upper right corner
    p2 = Rot_mat.dot(objPoints[3].T) + np.array(tvec[0]).T  # bottom right corner
    fi = angle(p1, p2)
    coordinate.append(XYZ_centr.T)
    for i in coordinate:
        data.append([id, i[0][0], i[0][1], i[0][2], fi])
    return data

def tracker(image_msg):
    cv_img = from_ImageMsg_to_cvImage(image_msg)
    arucoDictionacy = aruco.Dictionary_get(MARKER_TYPE)
    parameters = aruco.DetectorParameters_create()
    # image_sub = np.array(image_sub)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_img, arucoDictionacy, parameters=parameters)
    print(ids)
    print(type(ids))
    if ids is not None:
        for j in range(0, len(ids)):
            rvec, tvec, objPoints = aruco.estimatePoseSingleMarkers(corners[j], markerLength, CameraMtx, DistortionCoeff)

            #aruco.drawDetectedMarkers(cv_img, corners, ids)
            #aruco.drawAxis(image_sub, CameraMtx, DistortionCoeff, rvec, tvec, 0.1)
            data = get_coordinates(rvec, tvec, objPoints, ids[j][0])
            print(data)



if __name__ == '__main__':
    # CameraMtx = [682.42294095, 0.0, 469.948484581, 0.0, 681.44128234, 333.951368692, 0.0, 0.0, 1.0]
    # DistortionCoeff = [0.0692332572094, -0.170327984628, 0.00436794532634, 0.00496921804668, 0.0]
    rospy.init_node('tracker')
    # markerLength = rospy.get_param('~size')
    image_sub = rospy.Subscriber("camera_image", Image, tracker)

    rospy.spin()
