#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tensorflow.compat.v1 as tf
import numpy as np
from markers_array.msg import DistanceToMarker

import time


poseArr = np.array([[0, 1.278, 0.4975], [0.7855, 1.278, 0.4975], [0.7851, 0.534, -0.01], [0.7855, 0, 0.146], [0, 0.577, 0]]) #координаты фиксированных маркеров

def equations(DistanceToMarker_msg):
    start_time = time.time()
    sess = tf.InteractiveSession()
    tf.disable_v2_behavior()
    x_init = tf.initializers.random_uniform(minval=-3.0, maxval=3.0)
    y_init = tf.initializers.random_uniform(minval=-3.0, maxval=3.0)
    z_init = tf.initializers.random_uniform(minval=-3.0, maxval=3.0)

    x = tf.get_variable('x', shape=1 ,initializer=x_init)
    y = tf.get_variable('y', shape=1 ,initializer=y_init)
    z = tf.get_variable('z', shape=1 ,initializer=z_init)

    edgeArr = []
    edgeArr.append(DistanceToMarker_msg.distance[0])
    edgeArr.append(DistanceToMarker_msg.distance[1])
    edgeArr.append(DistanceToMarker_msg.distance[2])
    edgeArr.append(DistanceToMarker_msg.distance[3])
    edgeArr.append(DistanceToMarker_msg.distance[4])

    eq1 = x*x - 2*x*poseArr[0][0] + poseArr[0][0]*poseArr[0][0] + y*y - 2*y*poseArr[0][1] + poseArr[0][1]*poseArr[0][1] + z*z - 2*z*poseArr[0][2] + poseArr[0][2]*poseArr[0][2] - edgeArr[0]*edgeArr[0]
    eq2 = x*x - 2*x*poseArr[1][0] + poseArr[1][0]*poseArr[1][0] + y*y - 2*y*poseArr[1][1] + poseArr[1][1]*poseArr[1][1] + z*z - 2*z*poseArr[1][2] + poseArr[1][2]*poseArr[1][2] - edgeArr[1]*edgeArr[1]
    eq3 = x*x - 2*x*poseArr[2][0] + poseArr[2][0]*poseArr[2][0] + y*y - 2*y*poseArr[2][1] + poseArr[2][1]*poseArr[2][1] + z*z - 2*z*poseArr[2][2] + poseArr[2][2]*poseArr[2][2] - edgeArr[2]*edgeArr[2]
    eq4 = x*x - 2*x*poseArr[3][0] + poseArr[3][0]*poseArr[3][0] + y*y - 2*y*poseArr[3][1] + poseArr[3][1]*poseArr[3][1] + z*z - 2*z*poseArr[3][2] + poseArr[3][2]*poseArr[3][2] - edgeArr[3]*edgeArr[3]
    eq5 = x*x - 2*x*poseArr[4][0] + poseArr[4][0]*poseArr[4][0] + y*y - 2*y*poseArr[4][1] + poseArr[4][1]*poseArr[4][1] + z*z - 2*z*poseArr[4][2] + poseArr[4][2]*poseArr[4][2] - edgeArr[4]*edgeArr[4]



    loss = tf.losses.mean_squared_error([eq1, eq2, eq3, eq4, eq5],[(0.0,), (0.0,), (0.0,), (0.0,), (0.0,)]) # функция потерь
    optimizer = tf.train.GradientDescentOptimizer(0.01).minimize(loss)

    epsilon = 0.001

    sess.run(tf.global_variables_initializer())
    sess.run([x,y,z])
    loss_v = sess.run([loss])
    step=0

    check = True
    while loss_v[0] >= epsilon:
        step = step+1
        sess.run(optimizer)
        sess.run([x, y, z])
        loss_v = sess.run([loss])
        if (time.time() - start_time) > 5:
            check = False
            break
    if check:
        print("Step: " + str(step) + " Loss: " + str(loss_v))
        print("id = %d, x = %f, y = %f, , z = %f" % (DistanceToMarker_msg.id, x.eval(), y.eval(), z.eval()))
        print('')
    tf.get_variable_scope().reuse_variables()

if __name__ == '__main__':
    rospy.init_node('equations')
    sub = rospy.Subscriber("edge", DistanceToMarker, equations)
    rospy.spin()
