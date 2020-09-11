#! /usr/bin/env python3


import rospy
from markers_array.msg import DistanceToMarker
from scipy.optimize import fsolve
import numpy as np
import time
#0.539
poseArr = np.array([[0.0572, -0.032, 0.1443], [0.8432, -0.032, 0.1443], [0.0572, 0.5045, 0]])
edgeArr = []
#1[0.846, 0.505, 0]
#2[0.846, 0.27, 0]
#3[0.561, 0.27, 0]
#4[0.5562, -0.032, 0.1443]
def eq(p):
    x, y, z = p
    return (
    x * x - 2 * x * poseArr[0][0] + poseArr[0][0] * poseArr[0][0] + y * y - 2 * y * poseArr[0][1] + poseArr[0][1] *
    poseArr[0][1] + z * z - 2 * z * poseArr[0][2] + poseArr[0][2] * poseArr[0][2] - edgeArr[0] * edgeArr[0],
    x * x - 2 * x * poseArr[1][0] + poseArr[1][0] * poseArr[1][0] + y * y - 2 * y * poseArr[1][1] + poseArr[1][1] *
    poseArr[1][1] + z * z - 2 * z * poseArr[1][2] + poseArr[1][2] * poseArr[1][2] - edgeArr[1] * edgeArr[1],
    x * x - 2 * x * poseArr[2][0] + poseArr[2][0] * poseArr[2][0] + y * y - 2 * y * poseArr[2][1] + poseArr[2][1] *
    poseArr[2][1] + z * z - 2 * z * poseArr[2][2] + poseArr[2][2] * poseArr[2][2] - edgeArr[2] * edgeArr[2])


def equations(DistanceToMarker_msg):
    start_time = time.time()
    global edgeArr
    edgeArr.append(DistanceToMarker_msg.distance[0])
    edgeArr.append(DistanceToMarker_msg.distance[1])
    edgeArr.append(DistanceToMarker_msg.distance[2])
    x, y, z = fsolve(eq, (0, 0, 0))
    print(x, y, z)
    #print("--- %s seconds ---" % (time.time() - start_time))
    edgeArr = []


if __name__ == '__main__':
    rospy.init_node('equations')
    sub = rospy.Subscriber("edge", DistanceToMarker, equations)
    rospy.spin()