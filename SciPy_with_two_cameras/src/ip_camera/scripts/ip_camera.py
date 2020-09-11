#!/usr/bin/env python
import cv2
import urllib 
import numpy as np
from sensor_msgs.msg import Image 
import roslib
import sys
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError 
import argparse
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse

class IPCamera(object):
    def __init__(self, url):
        try:
            self.stream=urllib.urlopen(url)
        except:
            rospy.logerr('Unable to open camera stream: ' + str(url))
            sys.exit() #'Unable to open camera stream')
        self.bytes=''
        self.image_pub = rospy.Publisher("camera_image", Image)
        self.bridge = CvBridge()


def saveCameraInfo(camera_info_msg, filename):
    # Convert camera_info_msg and save to a yaml file
    rospy.loginfo("[saveCameraInfo] filename: %s" %(filename))
    file = open(filename, 'w')

    # Converted from camera_info_manager.py
    calib = {'image_width': camera_info_msg.width,
    'image_height': camera_info_msg.height,
    'camera_name': rospy.get_name().strip("/"), #TODO check this
    'distortion_model': camera_info_msg.distortion_model,
    'distortion_coefficients': {'data': camera_info_msg.D, 'rows':1, 'cols':5},
    'camera_matrix': {'data': camera_info_msg.K, 'rows':3, 'cols':3},
    'rectification_matrix': {'data': camera_info_msg.R, 'rows':3, 'cols':3},
    'projection_matrix': {'data': camera_info_msg.P,'rows':3, 'cols':4}}
    for i in range(9):
        file.write(str(calib['camera_matrix']['data'][i]) + "\n")
    for i in range (5):
        file.write(str(calib['distortion_coefficients']['data'][i]) + "\n")
    rospy.loginfo("[saveCameraInfo] calib %s" %(calib))
    file.close()
    return True

def cbSrvSetCameraInfo(req):
    # TODO: save req.camera_info to yaml file
    rospy.loginfo("[cbSrvSetCameraInfo] Callback!")
    filename = "/tmp/config.csv"
    response = SetCameraInfoResponse()
    response.success = saveCameraInfo(req.camera_info,filename)
    response.status_message = "Write to %s" %filename #TODO file name
    return response

if __name__ == '__main__':
    
    rospy.init_node('IPCamera', anonymous=True)
    srv_set_camera_info = rospy.Service("/camera/set_camera_info",SetCameraInfo,cbSrvSetCameraInfo)
    url = rospy.get_param('~url')
    ip_camera = IPCamera(url)

    while not rospy.is_shutdown():
        ip_camera.bytes += ip_camera.stream.read(1024)
        a = ip_camera.bytes.find(b'\xff\xd8')
        b = ip_camera.bytes.find(b'\xff\xd9')
        if a!=-1 and b!=-1:
            if a > b:
                ip_camera.bytes= ip_camera.bytes[b+2:]
                continue
            jpg = ip_camera.bytes[a:b+2]
            ip_camera.bytes= ip_camera.bytes[b+2:]
            i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.IMREAD_COLOR)
            image_message = i
            ip_camera.image_pub.publish(ip_camera.bridge.cv2_to_imgmsg(image_message, "bgr8"))

            if cv2.waitKey(1) ==27: # wait until ESC key is pressed in the GUI window to stop it
                exit(0) 
