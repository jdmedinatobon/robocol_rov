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
            rospy.Subscriber('/' + s + 'info', ImuInfo, self.sensor_info_callback)

        while not rospy.is_shutdown():
       		if self.done:
                 pose_imu = self.imu.dar_pose()
                 imu_pos.publish(pose_imu)
       		rate.sleep()

    def sensor_info_callback(self, msg):
    	if self.first == False:
	    	self.imu.actualizar(msg, 1/50.0)
	        #print(msg.linear_acceleration.x)
	        self.done = True

        #TODO: Terminar esto. No estoy seguro aun como organizarlos para que todo
        #este bien sincronizado.
        #Aqui se debe indicar que se comience a solucionar el problema de manera distribuida.
        #Por ahora chambon con un while ahi con un numero maximo de iteraciones, pero
        #debe haber una mejor forma de hacerlo.

        #Recordar que j se refiere a las iteraciones del problema de optimizacion
        j = 0
        while j < self.max_iter:


            j += 1

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
        node = imu_node(sys.argv[1], [sys.argv[2], sys.argv[3]])
        rospy.spin()
    except rospy.ROSInterruptException:

        print('caught exception')
