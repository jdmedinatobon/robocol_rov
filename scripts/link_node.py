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
        self.f = 50

    	self.link = Link(sensores)
        self.info = LinkInfo()
        self.info.id = namespace

        print(self.info.id + '_node: initializing node')

        self.first = True
        self.done = False

        self.link_info = rospy.Publisher('/' + self.info.id + '/info', LinkInfo, queue_size = 10)

        rate = rospy.Rate(self.f)

        for s in sensores:
            rospy.Subscriber('/' + s + '/info', ImuInfo, self.sensor_info_callback)

        while not rospy.is_shutdown():
       		rate.sleep()

    def sensor_info_callback(self, msg):
        print("Se llamo callback en link sensor.")
        print(msg)
    	if self.first:
            self.initialize_link()
            self.first = False
        elif msg.done:
            self.first = True
        else:
            self.info.price = self.link.calcular_info()
            self.link_info.publish(self.info)

    def initialize_link(self):
        self.info.price = self.link.reiniciar()
        self.link_info.publish(self.info)

if __name__ == '__main__':
    try:
    	if len(sys.argv) < 3:
            print("Se necesitan por lo menos 2 parametros para que funcione. Ej: [imu1 imu2]")
        else:
            args = rospy.myargv(sys.argv)
            rospy.init_node(args[1] + '_node', anonymous=True) #Se inicia el nodo

        lista_sensores = []
        i = 2
        while i < len(args):
            lista_sensores.append(args[i])
            i+=1
        print("Creando enlace")
        node = LinkNode(args[1], lista_sensores)
        rospy.spin()
    except rospy.ROSInterruptException:

        print('caught exception')
