#!/usr/bin/env python

import cv2
from geometry_msgs.msg import Vector3
import cv2.aruco as aruco
import rospy
import numpy as np
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge





MARKER_TYPE = aruco.DICT_5X5_250
CameraMtx = np.array([[682.42294095, 0.0, 469.948484581], [0.0, 681.44128234, 333.951368692], [0.0, 0.0, 1.0]])
DistortionCoeff = np.array([0.0692332572094, -0.170327984628, 0.00436794532634, 0.00496921804668, 0.0])
markerLength = 0.1



def from_ImageMsg_to_cvImage(image_msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
    return cv_image


def get_coordinates(rvec, tvec, id):
    coordinate = []
    data = []
    rvec = np.array(rvec).reshape((1, 3))
    Rot_mat, _ = cv2.Rodrigues(np.asarray(rvec))
    Rot_mat = Rot_mat.T
    centr_marker = np.array([[0, 0, 0]])
    XYZ_centr = Rot_mat.dot(centr_marker.T) + np.array(tvec[0]).T
    coordinate.append(XYZ_centr.T)
    for i in coordinate:
        data.append([id, i[0][0], i[0][1], i[0][2]])
    return data


def dist(data1, data2):

     return (math.sqrt(math.pow((data1[0][1] - data2[0][1]), 2) + math.pow((data1[0][2] - data2[0][2]), 2)))


def publ(edge):
    edgeVector = Vector3
    edgeVector.x = edge[0]
    edgeVector.y = edge[1]
    edgeVector.z = edge[2]
    print(1)
    pub.publish(edgeVector)
    print(2)

def tracker(image_msg):
    cv_img = from_ImageMsg_to_cvImage(image_msg)
    arucoDictionacy = aruco.Dictionary_get(MARKER_TYPE)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_img, arucoDictionacy, parameters=parameters)
    if ids is not None:
        markers = []
        data0 = []
        data1 = []
        data2 = []
        data3 = []
        for j in range(0, len(ids)):
            rvec, tvec, objPoints = aruco.estimatePoseSingleMarkers(corners[j], markerLength, CameraMtx,
                                                                    DistortionCoeff)
            aruco.drawDetectedMarkers(cv_img, corners, ids)
            aruco.drawAxis(cv_img, CameraMtx, DistortionCoeff, rvec, tvec, 0.1)
            data = get_coordinates(rvec, tvec, ids[j][0])
            markers.append(data[0])
            if data[0][0] == 0:
                data0 = data
            elif data[0][0] == 3:
                data1 = data
            elif data[0][0] == 4:
                data2 = data
            elif data[0][0] == 5:
                data3 = data

        if data0 and data1 and data2 and data3:
            edge = []
            edge.append(dist(data0, data1))
            edge.append(dist(data0, data2))
            edge.append(dist(data0, data3))
            publ(edge)


    cv2.namedWindow("tracker", cv2.WINDOW_NORMAL)
    cv2.imshow("tracker", cv_img)
    cv2.waitKey(20)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()



if __name__ == '__main__':
    rospy.init_node('tracker')
    pub = rospy.Publisher('vector', Vector3, queue_size=10)
    sub = rospy.Subscriber("camera_image", Image, tracker)

    rospy.spin()

