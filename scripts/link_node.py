#!/usr/bin/env python
import rospy
import sys

import math

import sys, unittest
import os, os.path, time
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, PoseWithCovariance, TwistWithCovariance, Twist, Vector3, Wrench

from robocol_rov.msg import ImuInfo
from robocol_rov.msg import LinkInfo
from link_class import Link

class LinkNode:
    def __init__(self, namespace, sensores):
    	self.namespace = namespace
    	print(self.namespace + '_node: initializing node')

    	self.link = Link(sensores)
        self.info = LinkInfo()
        self.max_iter = 100

        self.first = True
        self.done = False

        link_info = rospy.Publisher('/' + self.namespace + '/info', LinkInfo, queue_size = 10)

        rate = rospy.Rate(50)

        for s in sensores:
            rospy.Subscriber('/' + s + '/info', ImuInfo, self.sensor_info_callback)

        while not rospy.is_shutdown():
       		if self.done:
                 pose_imu = self.imu.dar_pose()
                 imu_pos.publish(pose_imu)
       		rate.sleep()

    def sensor_info_callback(self, msg):
    	if self.first == False:
	    	self.imu.actualizar(msg, 1/50.0)
	        self.done = True

    def publish_info(self):
        new_info = self.link.actualizar()

        #FIXME: Este if depronto molesta
        if new_info != -1:
            link_info.publish()

if __name__ == '__main__':
    try:
    	if len(sys.argv) < 3:
            print("Se necesitan por lo menos 2 parametros para que funcione. Ej: [imu1 imu2]")
        else:
            rospy.init_node(sys.argv[1] + '_node', anonymous=True) #Se inicia el nodo

        lista_sensores = []
        i = 2
        while i < len(sys.argv):
            lista_sensores.append(sys.argv[i])
            i+=1
        node = LinkNode(sys.argv[1], lista_sensores)
        rospy.spin()
    except rospy.ROSInterruptException:

        print('caught exception')
