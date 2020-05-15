#!/usr/bin/env python
import rospy
import sys

import math
from threading import Event

import sys, unittest
import os, os.path, time
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, PoseWithCovariance, TwistWithCovariance, Twist, Vector3, Wrench
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates

from robocol_rov.msg import ImuInfo
from robocol_rov.msg import LinkInfo
from imu_class import IMU

class ImuNode:
    def __init__(self, namespace, enlaces):
    	self.namespace = namespace
    	print(self.namespace + '_node: initializing node')

    	self.imu = IMU(enlaces)
        self.info = ImuInfo()
        self.info.id = namespace

        self.price_counter = 0
        self.price_max_iter = 10

        #Este counter no hace falta creo.
        #self.consensus_counter = 0
        self.consensus_max_iter = 10

        self.first = True
        self.done = False

        self.flag_price = Event()
        self.flag_consensus = Event()

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
       	rospy.Subscriber('/' + namespace +'/imu', Imu, self.imu_callback)
        self.imu_pos = rospy.Publisher('/' + namespace + '/pos', Pose, queue_size=10)
        self.imu_info = rospy.Publisher('/' + namespace + '/info', ImuInfo, queue_size = 10)

        rate = rospy.Rate(50)

        for e in enlaces:
            rospy.Subscriber('/' + e + '/info', LinkInfo, self.link_info_callback)

        while not rospy.is_shutdown():
       	    if self.done:
                pose_imu = self.imu.dar_pose()
                self.imu_pos.publish(pose_imu)
       		rate.sleep()

    def imu_callback(self, msg):
    	if self.first == False:
	    	self.imu.actualizar(msg, 1/50.0)
	        #print(msg.linear_acceleration.x)
	        self.done = True

        #TODO: Terminar esto. No estoy seguro aun como organizarlos para que todo
        #este bien sincronizado. Creo que voy a usar Threads.
        #Aqui se debe indicar que se comience a solucionar el problema de manera distribuida.
        #Por ahora chambon con un while ahi con un numero maximo de iteraciones, pero
        #debe haber una mejor forma de hacerlo.
        #self.flag_consensus.clear()

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

        print("Callback de enlace llamado. Info: {}".format(info))
        if self.price_counter < self.price_max_iter:
            self.info = self.imu.calcular_info(info)
            self.price_counter += 1
            imu_info.publish(self.info)
        else:
            self.info.bool = True
            imu_info.publish(self.info)
            self.flag_price.set()

    def initialize_sensors(self):

        self.info.grad = self.imu.calcular_grad()
        self.info.hessian = self.imu.calcular_hessian()
        self.info.done = False
        print("Iniciando sensores: Info: {}".format(self.info))

        self.imu_info.publish(self.info)

        print("Publicando la primera vez")

    def calcular_consensus(self):
        #Aqui se inicia a resolver el problema de optimizacion
        #Recordar que j se refiere a las iteraciones del problema de optimizacion
        j = 0
        while j < self.consensus_max_iter:
            #Paso 1: Esperar a que w (price) este calculado.
            #Usar un Event creo que funciona bastante bien.
            #Esto toca hacerlo en cada iteracion del problema de optimizacion
            print("Iniciando")
            self.initialize_sensors()
            self.flag_price.clear()
            self.flag_price.wait()

            self.imu.estimated_state += 1

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
