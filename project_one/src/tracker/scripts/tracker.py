#!/usr/bin/env python

import cv2
#from geometry_msgs.msg import Vector3
import cv2.aruco as aruco
import rospy
import numpy as np
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from markers_array.msg import MarkerArray
from markers_array.msg import MarkerType





MARKER_TYPE = aruco.DICT_5X5_250
#iphone 4k
CameraMtx = np.array([[2849.31712573, 0.0, 1907.54026021], [0.0, 2881.22269638, 1018.12586415], [0.0, 0.0, 1.0]])
DistortionCoeff = np.array([0.132690542564, -0.346855228386, -0.0040297243548, 0.00028860219729, 0.0])
#CameraMtx = np.array([[2686.89445256, 0.0, 1980.84905142], [0.0, 2720.41056849, 938.700636019], [0.0, 0.0, 1.0]])
#DistortionCoeff = np.array([-0.0131451337845, -0.0763250465426, -0.00833953975962, 0.0104788186668, 0.0])

#iphone 1080p
#CameraMtx = np.array([[1282.18188213, 0.0, 1008.94502727], [0.0, 1301.82884224, 462.342558456], [0.0, 0.0, 1.0]])
#DistortionCoeff = np.array([0.0263101376907, -0.0966386994373, -0.0136875576029, 0.0114316590767, 0.0])
#Samsung 720p
#CameraMtx = np.array([[923.518758845, 0.0, 598.2542786], [0.0, 934.705064994, 312.396407384], [0.0, 0.0, 1.0]])
#DistortionCoeff = np.array([0.00432807484621, -0.0811600797043, -0.0105392004397, -0.0101075514349, 0.0])
#iphone 360p
#CameraMtx = np.array([[682.42294095, 0.0, 469.948484581], [0.0, 681.44128234, 333.951368692], [0.0, 0.0, 1.0]])
#DistortionCoeff = np.array([0.0692332572094, -0.170327984628, 0.00436794532634, 0.00496921804668, 0.0])
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

def tracker(image_msg):
    cv_img = from_ImageMsg_to_cvImage(image_msg)
    arucoDictionacy = aruco.Dictionary_get(MARKER_TYPE)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_img, arucoDictionacy, parameters=parameters)
    if ids is not None:
        markers = MarkerArray()
        markers.marker = []
        for j in range(0, len(ids)):
            rvec, tvec, objPoints = aruco.estimatePoseSingleMarkers(corners[j], markerLength, CameraMtx,
                                                                    DistortionCoeff)
            aruco.drawDetectedMarkers(cv_img, corners, ids)
            aruco.drawAxis(cv_img, CameraMtx, DistortionCoeff, rvec, tvec, 0.1)
            data = get_coordinates(rvec, tvec, ids[j][0])
            marker = MarkerType()
            marker.id = data[0][0]
            marker.coord = []
            for i in range(3):
                marker.coord.append(data[0][i+1])
            markers.marker.append(marker)
        pub.publish(markers)
            
    cv2.namedWindow("tracker", cv2.WINDOW_NORMAL)
    cv2.imshow("tracker", cv_img)
    cv2.waitKey(20)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()



if __name__ == '__main__':
    rospy.init_node('tracker')
    pub = rospy.Publisher('markers', MarkerArray, queue_size=10)
    sub = rospy.Subscriber("camera_image", Image, tracker)

    rospy.spin()

