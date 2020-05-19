#!/usr/bin/env python
import rospy
import sys

import math, time
from threading import Event
import numpy as np

import sys, unittest
import os, os.path, time
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, PoseWithCovariance, TwistWithCovariance, Twist, Vector3, Wrench
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates

from robocol_rov.msg import ImuInit
from robocol_rov.msg import ImuInfo
from robocol_rov.msg import LinkInfo
from imu_class import IMU

class ImuNode:
    def __init__(self, namespace, enlaces):
    	self.namespace = namespace
        self.f = 500
    	print(self.namespace + '_node: initializing node')

    	self.imu = IMU(enlaces)
        self.init = ImuInit()
        self.init.id = namespace
        self.info = ImuInfo()
        self.info.id = namespace
        self.num_links = len(enlaces)
        self.enlaces = enlaces

        self.price_counter = 0
        self.price_max_iter = 100
        self.link_info = {e : 0 for e in enlaces}
        self.is_link_info_new = {e : 0 for e in enlaces}

        #Este counter no hace falta creo.
        #self.consensus_counter = 0
        self.consensus_max_iter = 100

        self.first = True
        self.done = False

        self.flag_price = Event()
        self.flag_consensus = Event()

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
       	rospy.Subscriber('/' + namespace +'/imu', Imu, self.imu_callback)
        self.imu_pos = rospy.Publisher('/' + namespace + '/pos', Pose, queue_size=10)
        self.imu_init_pub = rospy.Publisher('/' + namespace + '/init', ImuInit, queue_size = 10)
        self.imu_info_pub = rospy.Publisher('/' + namespace + '/info', ImuInfo, queue_size = 10)

        rate = rospy.Rate(self.f)

        self.time = time.time()

        for e in enlaces:
            rospy.Subscriber('/' + e + '/info', LinkInfo, self.link_info_callback)

        while not rospy.is_shutdown():
            # #self.imu_info_pub.publish(self.info)

            # if self.done:
            #     pose_imu = self.imu.dar_pose()
            #     self.imu_pos.publish(pose_imu)
                #print(pose_imu)
       		rate.sleep()

    def imu_callback(self, msg):
    	if self.first == False:
            self.imu.actualizar(msg, 1./self.f)
	        #print(msg.linear_acceleration.x)
            self.done = True


        #TODO: Terminar esto. No estoy seguro aun como organizarlos para que todo
        #este bien sincronizado. Creo que voy a usar Threads.
        #Aqui se debe indicar que se comience a solucionar el problema de manera distribuida.
        #Por ahora chambon con un while ahi con un numero maximo de iteraciones, pero
        #debe haber una mejor forma de hacerlo.
        #self.flag_consensus.clear()
        print("imu_callback llamado. Tiempo entre llamados: {} segundos".format(time.time()-self.time))
        pose_imu = self.imu.dar_pose()
        # self.imu_pos.publish(pose_imu)
        # print(pose_imu)

        self.time = time.time()
        self.calcular_consensus()

        #No creo que esta flag se necesite
        #self.flag_consensus.wait(timeout = 1/50.0)

        #print(self.imu.estimated_state)

    def model_states_callback(self, msg):
    	if self.first:
            self.imu.iniciar(msg)
            self.first = False

        self.x_gazebo = msg.pose[1].position.x
    	self.y_gazebo = msg.pose[1].position.y
    	self.z_gazebo = msg.pose[1].position.z

    def link_info_callback(self, info):
        # print("Callback de enlace llamado. Info: {}, Tiempo entre llamados :{}".format(info, time.time()-self.time))

        if self.price_counter < self.price_max_iter:
            # self.info.grad, self.info.hessian = self.imu.calcular_info(info)
            self.link_info[info.id] = info.price
            self.is_link_info_new[info.id] = 1
            # print("Sensor: {}. Suma: {}".format(self.namespace,sum(self.is_link_info_new.values())))

            if sum(self.is_link_info_new.values()) == self.num_links:
                self.price_counter += 1
                # print(self.link_info)
                self.imu.calcular_info(self.link_info)
                self.info.PI = self.imu.PI
                # print(self.info.PI)
                self.imu_info_pub.publish(self.info)
                self.is_link_info_new = {e : 0 for e in self.enlaces}
                # print("Aqui")
        else:
            self.info.done = True
            self.imu_info_pub.publish(self.info)
            self.flag_price.set()
            self.price_counter = 0
        #self.time = time.time()

    def initialize_sensors(self):
        self.imu.calcular_x_barra()
        self.imu.calcular_grad()
        print(self.imu.grad)
        self.imu.calcular_hessian()

        self.info.done = False

        self.init.grad = self.imu.grad.flatten()#self.imu.grad.tolist()
        self.init.hessian = np.diag(self.imu.hessian)
        self.init.num_links = self.num_links
        #self.info.done = False
        #print(self.init)
        self.imu_init_pub.publish(self.init)
        # print(self.init)

    def calcular_consensus(self):
        #Aqui se inicia a resolver el problema de optimizacion
        #Recordar que j se refiere a las iteraciones del problema de optimizacion

        j = 0
        while j < self.consensus_max_iter:
            tiempo = time.time()
            #Paso 1: Esperar a que w (price) este calculado.
            #Usar un Event creo que funciona bastante bien.
            #Esto toca hacerlo en cada iteracion del problema de optimizacion

            self.initialize_sensors()
            self.flag_price.clear()
            self.flag_price.wait()#timeout = 1/50.0)

            print("PI:")
            print(self.imu.PI)
            self.imu.calcular_x_consensus()

            delta = time.time()-tiempo
            print("j = {}. Tiempo Iteracion: {} milisegundos".format(j, delta*1000))
            j += 1

        pose_imu = self.imu.dar_pose()
        self.imu_pos.publish(pose_imu)
        print("Optimizacion Terminada. Estimacion: {}".format(pose_imu))

if __name__ == '__main__':
    try:
    	if len(sys.argv) < 3:
            print("Se necesitan por lo menos 2 parametros para que funcione. Ej: [imu1 imu2]")
        else:
            args = rospy.myargv(sys.argv)
            rospy.init_node(args[1] + '_node', anonymous=True) #Se inicia el nodo

        lista_enlaces = []
        i = 2
        print(len(args))
        while i < len(args):

            lista_enlaces.append(args[i])
            i+=1
        node = ImuNode(args[1], lista_enlaces)
        rospy.spin()
    except rospy.ROSInterruptException:

        print('caught exception')
