#!/usr/bin/env python

import rospy
import math
from markers_array.msg import MarkerArray
from markers_array.msg import DistanceToMarker

const_markers_id = [1, 2, 3, 4, 5]

def dist(data1, data2):
     return (math.sqrt(math.pow((data1[0] - data2[0]), 2) + math.pow((data1[1] - data2[1]), 2) + math.pow((data1[2] - data2[2]), 2)))

def distance(MarkerArray_msg):
    checker = 0
    data1 = []
    data2 = []
    data3 = []
    data4 = []
    data5 = []
    for i in range(len(MarkerArray_msg.marker)):
        if MarkerArray_msg.marker[i].id == 1:
            data1 = MarkerArray_msg.marker[i].coord
        elif MarkerArray_msg.marker[i].id == 2:
            data2 = MarkerArray_msg.marker[i].coord
        elif MarkerArray_msg.marker[i].id == 3:
            data3 = MarkerArray_msg.marker[i].coord
        elif MarkerArray_msg.marker[i].id == 4:
            data4 = MarkerArray_msg.marker[i].coord
        elif MarkerArray_msg.marker[i].id == 5:
            data5 = MarkerArray_msg.marker[i].coord
    if data1 and data2 and data3 and data4 and data5:
        for i in range(len(MarkerArray_msg.marker)):
            if MarkerArray_msg.marker[i].id not in const_markers_id:
                edge = DistanceToMarker()
		edge.distance = []
		edge.distance.append(dist(MarkerArray_msg.marker[i].coord, data1))
		edge.distance.append(dist(MarkerArray_msg.marker[i].coord, data2))
		edge.distance.append(dist(MarkerArray_msg.marker[i].coord, data3))
		edge.distance.append(dist(MarkerArray_msg.marker[i].coord, data4))
		edge.distance.append(dist(MarkerArray_msg.marker[i].coord, data5))
		edge.id = MarkerArray_msg.marker[i].id

                pub.publish(edge)


if __name__ == '__main__':
    rospy.init_node('distance')
    pub = rospy.Publisher('edge', DistanceToMarker, queue_size=10)
    sub = rospy.Subscriber("markers", MarkerArray, distance)
    rospy.spin()
