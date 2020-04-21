import cv2.aruco as aruco
import rospy


MARKER_TYPE = aruco.DICT_5X5_250

def tracker(image, markerLength, CameraMtx, DistortionCoeff):
    arucoDictionacy = aruco.Dictionary_get(MARKER_TYPE)
    param = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(image, arucoDictionacy, param=param)
    for j in range(len(ids)):
        rvec, tvec, objPoints = aruco.estimatePoseSingleMarkers(corners[j], markerLength, CameraMtx, DistortionCoeff)
        aruco.drawDetectedMarkers(image, corners, ids)
        aruco.drawAxis(image, CameraMtx, DistortionCoeff, rvec, tvec, 0.1)

def sub(CameraMtx, DistortionCoeff):
    rospy.init_node('tracker')
    rospy.Subscriber("camera_image", image)
    tracker(image, markerLength, CameraMtx, DistortionCoeff)
    rospy.spinOnes()

if __name__ == '__main__':
    CameraMtx = [682.42294095, 0.0, 469.948484581, 0.0, 681.44128234, 333.951368692, 0.0, 0.0, 1.0]
    DistortionCoeff = [0.0692332572094, -0.170327984628, 0.00436794532634, 0.00496921804668, 0.0]
    sub(CameraMtx, DistortionCoeff)

