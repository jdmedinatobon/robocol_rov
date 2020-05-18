#!/usr/bin/env python
import rospy
import sys

import math, time

import sys, unittest
import os, os.path, time
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, PoseWithCovariance, TwistWithCovariance, Twist, Vector3, Wrench
import numpy as np

from robocol_rov.msg import ImuInit
from robocol_rov.msg import ImuInfo
from robocol_rov.msg import LinkInfo
from link_class import Link

class LinkNode:
    def __init__(self, namespace, sensores):
        self.f = 50

    	self.link = Link(sensores)
        self.sensores = sensores
        self.num_sensors = len(sensores)
        self.info = LinkInfo()
        self.info.id = namespace

        print(self.info.id + '_node: initializing node')

        self.first = True
        self.done = False

        self.sensor_info = {s : 0 for s in sensores} #Estos serian los PI mayuscula.
        self.is_sensor_info_new = {s : 0 for s in sensores}
        self.sensor_grads = {s: 0 for s in sensores}
        self.sensor_hessians = {s: 0 for s in sensores}
        self.sensor_num_links = {s: 0 for s in sensores}

        self.link_info_pub = rospy.Publisher('/' + self.info.id + '/info', LinkInfo, queue_size = 10)

        rate = rospy.Rate(self.f)

        for s in sensores:
            rospy.Subscriber('/' + s + '/init', ImuInit, self.sensor_init_callback)
            rospy.Subscriber('/' + s + '/info', ImuInfo, self.sensor_info_callback)

        while not rospy.is_shutdown():
       		rate.sleep()

    def sensor_init_callback(self, init):
        self.sensor_grads[init.id] = init.grad
        self.sensor_hessians[init.id] = init.hessian
        self.sensor_num_links[init.id] = init.num_links
        self.is_sensor_info_new[init.id] = 1

        if sum(self.is_sensor_info_new.values()) == self.num_sensors:
            self.info.price = self.link.reiniciar(self.sensor_grads, self.sensor_hessians, self.sensor_num_links)
            self.is_sensor_info_new = {s : 0 for s in self.sensores}
            self.link_info_pub.publish(self.info)

    def sensor_info_callback(self, info):

        #Revisar este if.
        if not info.done:
            if np.sum(self.is_sensor_info_new) < self.num_sensors:
                self.sensor_info[info.id] = info.PI
                self.is_sensor_info_new[info.id] = 1
            else:
                self.info.price = self.link.calcular_info(self.sensor_info)
                self.link_info_pub.publish(self.info)
                self.is_sensor_info_new = {s : 0 for s in self.sensores}

    def initialize_link(self):
        self.info.price = self.link.reiniciar()
        self.link_info_pub.publish(self.info)

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
