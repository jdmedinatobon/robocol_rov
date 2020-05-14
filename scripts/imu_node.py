#!/usr/bin/env python
import rospy
import sys

import math

import sys, unittest
import os, os.path, time
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, PoseWithCovariance, TwistWithCovariance, Twist, Vector3, Wrench
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates

from robocol_rov.msg import ImuInfo
from robocol_rov.msg import LinkInfo
from imu_class import Imu

class ImuNode:
    def __init__(self, namespace, enlaces):
    	self.namespace = namespace
    	print(self.namespace + '_node: initializing node')

    	self.imu = Imu(enlaces)
        self.info = ImuInfo()
        self.max_iter = 100

        self.first = True
        self.done = False

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
       	rospy.Subscriber('/'+ self.namespace +'/imu', Imu, self.imu_callback)

        imu_pos = rospy.Publisher('/' + self.namespace + '/pos', Pose, queue_size=10)
        imu_info = rospy.Publisher('/' + self.namespace + '/info', ImuInfo, queue_size = 10)

        rate = rospy.Rate(50)

        for e in enlaces:
            rospy.Subscriber('/' + e + '/info', LinkInfo, self.link_info_callback)

        while not rospy.is_shutdown():
       		if self.done:
                 pose_imu = self.imu.dar_pose()
                 imu_pos.publish(pose_imu)
       		rate.sleep()

    def imu_callback(self, msg):
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

    def model_states_callback(self, msg):
    	if self.first:
            self.imu.iniciar(msg)
            self.first = False

        self.x_gazebo = msg.pose[1].position.x
    	self.y_gazebo = msg.pose[1].position.y
    	self.z_gazebo = msg.pose[1].position.z

    def link_info_callback(self, info):
        self.imu.calcular_info(info)

    def publish_info(self):
        new_info = self.imu.calcular_info()

        #FIXME: Este if depronto molesta
        if new_info != -1:
            imu_info.publish()

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
