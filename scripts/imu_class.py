#!/usr/bin/env python
import numpy as np
from numpy.linalg import inv
import rospy
from geometry_msgs.msg import Pose
from classes import State
from pyquaternion import Quaternion
from threading import Event

class IMU:
    def __init__(self, enlaces):
        self.state = State()
        self.sensor_observation = State()
        self.estimated_state = State()

        self.bias_x = 0
        self.bias_y = 0
        self.bias_z = 9.8

        #TODO: Las siguientes matrices todas toca cambiarlas por sus valores reales.
        self.H = np.diag(np.ones(3))
        #Matriz de covarianza del ruido del sistema
        self.Q = np.diag(np.ones(3))

        #Matriz de covarianza del ruido de la medida
        self.R = np.diag(np.ones(3))

        #Matriz de covarianza del error. Lo que es Pk barra en el paper.
        self.P = np.zeros((3,3))

        #Matriz de restriccion utilizada en el problema de optimizacion
        self.C = np.ones((3,3))

        self.grad = 1.0
        self.hessian = 1.0
        self.PI = 1.0
        #self.num_links = len(enlaces)

        #Por ahora lo de ver cuando estan actualizados chambon con otro diccionario

    def dar_pose(self):
        pose = Pose()

        pose.position.x = self.state.x
        pose.position.y = self.state.y
        pose.position.z = self.state.z
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 0

        return pose

    def iniciar(self, pose):
        self.state.x = pose.pose[1].position.x
        self.state.y = pose.pose[1].position.y
        self.state.z = pose.pose[1].position.z

    def actualizar(self, acel, sample_time):
        quaternion= Quaternion(acel.orientation.x,acel.orientation.y,acel.orientation.z,acel.orientation.w)
        aceler= quaternion.rotate(np.array([acel.linear_acceleration.x, acel.linear_acceleration.y, acel.linear_acceleration.z-self.bias_z]))
        #print(aceler)
    	self.state.x = self.state.x + sample_time*self.state.vx + (((sample_time)**2)/2.0)*(aceler[0]-self.bias_x)
        self.state.y = self.state.y + sample_time*self.state.vy + (((sample_time)**2)/2.0)*(aceler[1]-self.bias_y)
        self.state.z = self.state.z + sample_time*self.state.vz + (((sample_time)**2)/2.0)*(aceler[2])
        self.state.vx = self.state.vx + sample_time*(aceler[0])
        self.state.vy = self.state.vy + sample_time*(aceler[1])
        self.state.vz = self.state.vz + sample_time*(aceler[2])

    def calcular_grad(self):
        #TODO: Poner aqui como calcular el gradiente
        #FIXME: Esto no sirve aun. Mirar si lo de State si es necesario o solo usar un vector de 6 y ya.
        self.grad = inv(self.P)*(self.state.x - self.estimated_state.x)-self.H.T*inv(self.R)*(self.sensor_observation-self.H*self.state.x)

    def calcular_hessian(self):
        #TODO: Poner aqui como calcular la hessiana
        self.hessian = inv(self.P) + self.H.T*inv(self.R)*self.H

    def calcular_info(self, prices):
        self.PI = self.hessian*np.sum(prices.values())
        return self.PI
