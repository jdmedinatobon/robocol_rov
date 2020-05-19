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
        self.state = np.zeros((6,1)) # esto en el paper es equivalente a z, la estimacion del sensor
        self.estimated_state = np.zeros((6,1)) # esto en el paper es equivalente a x barra, la estimacion realizada a partir de nuestro modelo
        self.x_consensus = np.zeros((6,1)) # esto es lo que esperamos llegue a ser magico y converger a un valor global para todos los sensores... mistico? tal vez. Hotel? Trivago

        self.bias_x = 0
        self.bias_y = 0
        self.bias_z = 9.8

        self.s = 0.00001

        self.x_barra = []


        self.Fk = np.diag(np.ones(6)) # matriz de representacion de nuestro modelo

        self.F = np.diag(np.ones(6))

        #TODO: Las siguientes matrices todas toca cambiarlas por sus valores reales.
        self.H = np.diag(np.ones(6))

        #Matriz de covarianza del ruido del sistema
        self.Q = np.diag(np.ones(6))

        #Matriz de covarianza del ruido de la medida
        self.R = np.diag(np.ones(6))

        #Matriz de covarianza del error. Lo que es Pk barra en el paper.
        self.P = np.zeros((6,6))

        #Matriz de restriccion utilizada en el problema de optimizacion
        self.C = np.ones((6,6))

        self.grad = 1.0
        self.hessian = 1.0
        self.PI = 1.0
        #self.num_links = len(enlaces)

        #Por ahora lo de ver cuando estan actualizados chambon con otro diccionario

    def dar_pose(self):
        pose = Pose()

        pose.position.x = self.state[0]
        pose.position.y = self.state[1]
        pose.position.z = self.state[2]
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 0

        return pose

    def iniciar(self, pose):
        self.state[0] = pose.pose[1].position.x
        self.state[1] = pose.pose[1].position.y
        self.state[2] = pose.pose[1].position.z
        self.estimated_state[0] = pose.pose[1].position.x
        self.estimated_state[1] = pose.pose[1].position.y
        self.estimated_state[2] = pose.pose[1].position.z
        self.x_consensus[0] = pose.pose[1].position.x
        self.x_consensus[1] = pose.pose[1].position.y
        self.x_consensus[2] = pose.pose[1].position.z

    def actualizar(self, acel, sample_time):
        quaternion= Quaternion(acel.orientation.x,acel.orientation.y,acel.orientation.z,acel.orientation.w)
        aceler= quaternion.rotate(np.array([acel.linear_acceleration.x, acel.linear_acceleration.y, acel.linear_acceleration.z-self.bias_z]))
        #print(aceler)
    	self.state[0] += sample_time*self.state[3] + (((sample_time)**2)/2.0)*(aceler[0]-self.bias_x)
        self.state[1] += sample_time*self.state[4] + (((sample_time)**2)/2.0)*(aceler[1]-self.bias_y)
        self.state[2] += sample_time*self.state[5] + (((sample_time)**2)/2.0)*(aceler[2])
        self.state[3] += sample_time*(aceler[0])
        self.state[4] += sample_time*(aceler[1])
        self.state[5] += sample_time*(aceler[2])
        self.F[0,3] = sample_time
        self.F[1,4] = sample_time
        self.F[2,5] = sample_time
        self.P = np.dot(np.dot(self.F,self.P),np.transpose(self.F)) + self.Q
        return self.state, self.P

    def calcular_grad(self):
        #TODO: Poner aqui como calcular el gradiente
        self.grad = np.dot(inv(self.P), (self.x_consensus - self.estimated_state))-np.dot(np.dot(self.H.T,inv(self.R)),(self.state-np.dot(self.H, self.state)))

    def calcular_hessian(self):
        #TODO: Poner aqui como calcular la hessiana
        self.hessian = inv(self.P) + np.dot(np.dot(self.H.T,inv(self.R)),self.H)

    def calcular_info(self, prices):
        self.PI = self.hessian*np.sum(prices.values())
        return self.PI

    def calcular_x_barra(self): #Le quite lo del w_k por ahora. #, w_k):
        self.estimated_state = np.dot(self.Fk, self.x_consensus)# + w_k

    def calcular_x_consensus(self, w):
        self.delta_x = -np.dot(inv(self.hessian),self.grad) - np.dot(np.dot(inv(self.hessian),self.C),w)
        self.x_consensus = self.x_consensus + s*self.delta_x
