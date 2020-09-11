#!/usr/bin/env python

import cv2
import cv2.aruco as aruco
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from markers_array.msg import MarkerArray
from markers_array.msg import MarkerType





MARKER_TYPE = aruco.DICT_5X5_250

'''CameraMtx = np.array([[2838.56875105, 0.0, 1913.19514094], [0.0, 2845.68793666, 1073.98011982], [0.0, 0.0, 1.0]])
DistortionCoeff = np.array([0.0648295379237, -0.143404253863, 0.00196645189456, -0.000643280435673, 0.0])
'''

CameraMtx = np.array([[2838.56875105, 0.0, 1913.19514094], [0.0, 2845.68793666, 1073.98011982], [0.0, 0.0, 1.0]])
DistortionCoeff = np.array([0.0648295379237, -0.143404253863, 0.00196645189456, -0.000643280435673, 0.0])
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
    pub = rospy.Publisher('markers', MarkerArray)
    sub = rospy.Subscriber("camera_image", Image, tracker)

    rospy.spin()

