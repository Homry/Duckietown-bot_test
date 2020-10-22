#!/usr/bin/env python

import rospy
import math
from markers_array.msg import MarkerArray
from markers_array.msg import DistanceToMarker


class Distance:

    def __init__(self):
        self.const_markers_id = [0, 4, 5]
        self.edge1 = None
        self.edge2 = None
        self.pub = rospy.Publisher('edge', DistanceToMarker)
        self.sub1 = rospy.Subscriber("markerFromCamOne", MarkerArray, self.getEdgeOne)
        self.sub2 = rospy.Subscriber("markerFromCamOne", MarkerArray, self.getEdgeTwo)


    def getEdgeOne(self, MarkerArray_msg):
        self.edge1 = MarkerArray_msg

    def getEdgeTwo(self, MarkerArray_msg):
        self.edge2 = MarkerArray_msg
        self.calculateDistance()

'''
    Функция проверяет типы переменных. 
    Если обе переменные типа MarkerArray, возвращает True, илаче False
'''
    def checkType(self):
        if isinstance(self.edge1, MarkerArray) and isinstance(self.edge2, MarkerArray):
            return True
        else:
            return False

    ''''''
    def getDistance(self, data1, data2):
        return (math.sqrt(math.pow((data1[0] - data2[0]), 2) + math.pow((data1[1] - data2[1]), 2) + math.pow((data1[2] - data2[2]),2)))

    def getDistanceToMarker(self, data):
        marker1 = []
        marker2 = []
        marker3 = []
        for i in range(len(data.marker)):
            if data.marker[i].id == 0:
                marker1 = data.marker[i].coord
            elif data.marker[i].id == 4:
                marker2 = data.marker[i].coord
            elif data.marker[i].id == 5:
                marker3 = data.marker[i].coord
        if marker1 and marker2 and marker3:
            edgeCam = []
            for i in range(len(self.edge1.marker)):
                if data.marker[i].id not in self.const_markers_id:
                    edge = DistanceToMarker()
                    edge.distance = []
                    edge.distance.append(self.getDistance(self.edge1.marker[i].coord, marker1))
                    edge.distance.append(self.getDistance(self.edge1.marker[i].coord, marker2))
                    edge.distance.append(self.getDistance(self.edge1.marker[i].coord, marker3))
                    edge.id = self.edge1.marker[i].id
                    edgeCam.append(edge)
        return edgeCam

    def getIdFromCam(self, data):
        id = []
        for i in range(len(data)):
            id.append(data[i].id)
        return id

    def conventTwoCam(self, data1, data2, id1, id2):
        for i in range(len(id1)):
            if i <= len(id2):
                if id1[i] == id2[i]:
                    self.pubDistance(self.getEdge2(data1[i], data2[i], id1[i]))
                    '''
                    edge = DistanceToMarker()
                    edge.id = id1[i]
                    edge.distance.append((data1[i].distance[0] + data2[i].distance[0]) / 2)
                    edge.distance.append((data1[i].distance[1] + data2[i].distance[1]) / 2)
                    edge.distance.append((data1[i].distance[2] + data2[i].distance[2]) / 2)
                    self.pubDistance(edge)
                    '''
                else:
                    self.pubDistance(self.getEdge(data1[i], id1[i]))
                    '''
                    edge = DistanceToMarker()
                    edge.id = id1[i]
                    edge.distance.append(data1[i].distance[0])
                    edge.distance.append(data1[i].distance[1])
                    edge.distance.append(data1[i].distance[2])
                    self.pubDistance(edge)
                    '''
                    self.pubDistance(self.getEdge(data2[i], id2[i]))
                    '''
                    edge = DistanceToMarker()
                    edge.id = id2[i]
                    edge.distance.append(data2[i].distance[0])
                    edge.distance.append(data2[i].distance[1])
                    edge.distance.append(data2[i].distance[2])
                    self.pubDistance(edge)
                    '''
            else:
                self.pubDistance(self.getEdge(data1[i], id1[i]))
                '''
                edge = DistanceToMarker()
                edge.id = id1[i]
                edge.distance.append(data1[i].distance[0])
                edge.distance.append(data1[i].distance[1])
                edge.distance.append(data1[i].distance[2])
                self.pubDistance(edge)
                '''

    def getEdge(self, data, id):
        edge = DistanceToMarker()
        edge.id = id
        edge.distance.append(data.distance[0])
        edge.distance.append(data.distance[1])
        edge.distance.append(data.distance[2])
        return edge

    def getEdge2(self, data1, data2, id1):
        edge = DistanceToMarker()
        edge.id = id1
        edge.distance.append((data1.distance[0] + data2.distance[0]) / 2)
        edge.distance.append((data1.distance[1] + data2.distance[1]) / 2)
        edge.distance.append((data1.distance[2] + data2.distance[2]) / 2)
        return edge



    def calculateDistance(self):
        if self.checkType():
            camOne = self.getDistanceToMarker(self.edge1)
            camTwo = self.getDistanceToMarker(self.edge2)
            idCamOne = self.getIdFromCam(camOne)
            idCamTwo = self.getIdFromCam(camTwo)
            if len(idCamOne) >= len(idCamTwo):
                self.conventTwoCam(camOne, camTwo, idCamOne, idCamTwo)
            else:
                self.conventTwoCam(camTwo, camOne, idCamTwo, idCamOne)



    def pubDistance(self, data):
        self.pub.publish(data)


if __name__ == '__main__':
    rospy.init_node('distance')
    a = Distance()
    rospy.spin()
