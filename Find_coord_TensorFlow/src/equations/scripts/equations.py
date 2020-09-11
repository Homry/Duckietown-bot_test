#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tensorflow.compat.v1 as tf
import numpy as np
#from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray


poseArr = np.array([[0.7, 0.3], [0.7, 0.9], [0.1, 0.9]])

def equations(Float64MultiArray_msg):
    sess = tf.InteractiveSession()
    tf.disable_v2_behavior()
    x_init = tf.initializers.random_uniform(minval=-3.0, maxval=3.0)
    y_init = tf.initializers.random_uniform(minval=-3.0, maxval=3.0)

    x = tf.get_variable('x', shape=1 ,initializer=x_init)
    y = tf.get_variable('y', shape=1 ,initializer=y_init)

    edgeArr = []
    edgeArr.append(Float64MultiArray_msg.data[0])
    edgeArr.append(Float64MultiArray_msg.data[1])
    edgeArr.append(Float64MultiArray_msg.data[2])

    eq1 = x*x - 2*x*poseArr[0][0] + poseArr[0][0]*poseArr[0][0] + y*y - 2*y*poseArr[0][1] + poseArr[0][1]*poseArr[0][1] - edgeArr[0]*edgeArr[0]
    eq2 = x*x - 2*x*poseArr[1][0] + poseArr[1][0]*poseArr[1][0] + y*y - 2*y*poseArr[1][1] + poseArr[1][1]*poseArr[1][1] - edgeArr[1]*edgeArr[1]
    eq3 = x*x - 2*x*poseArr[2][0] + poseArr[2][0]*poseArr[2][0] + y*y - 2*y*poseArr[2][1] + poseArr[2][1]*poseArr[2][1] - edgeArr[2]*edgeArr[2]



    loss = tf.losses.mean_squared_error([eq1, eq2, eq3],[(0.0,), (0.0,), (0.0,)]) # функция потерь
    optimizer = tf.train.GradientDescentOptimizer(0.01).minimize(loss)

    epsilon = 0.01

    sess.run(tf.global_variables_initializer())
    sess.run([x,y])
    loss_v = sess.run([loss])
    step=0


    while loss_v[0] >= epsilon:
        step = step+1
        sess.run(optimizer)
        sess.run([x, y])
        loss_v = sess.run([loss])

    print("Step: " + str(step) + " Loss: " + str(loss_v))
    print("x = %f, y = %f" % (x.eval(), y.eval()))
    tf.get_variable_scope().reuse_variables()

if __name__ == '__main__':
    rospy.init_node('equations')
    sub = rospy.Subscriber("vector", Float64MultiArray, equations)
    rospy.spin()
