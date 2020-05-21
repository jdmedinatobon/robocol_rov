#!/usr/bin/env python
import numpy as np
from numpy.linalg import inv
import copy
import rospy
from geometry_msgs.msg import Pose
from classes import State
import pyquaternion
from threading import Event
from sympy import symbols, Matrix, Transpose
from sympy import *

class IMU:
    def __init__(self, enlaces):
        self.state = np.ones((6,1)) # esto en el paper es equivalente a z, la estimacion del sensor
        self.estimated_state = np.zeros((6,1)) # esto en el paper es equivalente a x barra, la estimacion realizada a partir de nuestro modelo
        self.x_consensus = np.zeros((6,1)) # esto es lo que esperamos llegue a ser magico y converger a un valor global para todos los sensores... mistico? tal vez. Hotel? Trivago
        self.mistico = np.zeros((6,1)) 
        self.bias_x = 0
        self.bias_y = 0
        self.bias_z = 9.8

        self.s = 1e-5 #Step size

        #Esta es la matriz Fk del modelo
        self.F = np.eye(6)

        #TODO: Las siguientes matrices todas toca cambiarlas por sus valores reales.
        self.H = np.eye(6)

        #Matriz de covarianza del ruido del sistema
        self.Q = np.eye(6)*0.01

        #Matriz de covarianza del ruido de la medida
        self.R = np.eye(6)*0.01

        #Matriz de covarianza del error. Lo que es Pk barra en el paper.
        self.P = copy.copy(self.Q)#np.zeros((6,6))

        #Matriz de restriccion utilizada en el problema de optimizacion
        self.C = np.ones((6,6))

        self.grad = np.ones((6,1))
        self.hessian = np.ones((6,6))
        self.PI = np.ones((6,1))
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
        self.x_consensus[3] = pose.twist[1].linear.x
        self.x_consensus[4] = pose.twist[1].linear.y
        self.x_consensus[5] = pose.twist[1].linear.z
        self.mistico[0] = pose.pose[1].position.x
        self.mistico[1] = pose.pose[1].position.y
        self.mistico[2] = pose.pose[1].position.z
        self.mistico[3] = pose.twist[1].linear.x
        self.mistico[4] = pose.twist[1].linear.y
        self.mistico[5] = pose.twist[1].linear.z

    def actualizar(self, acel, sample_time):
        self.calcular_z(acel, sample_time)

        #TODO: Terminar este metodo de F
        self.calcular_F(sample_time)

        self.calcular_H(sample_time)
        self.calcular_P()

    def calcular_grad(self):
        self.grad = np.dot(inv(self.P), self.x_consensus - self.estimated_state) - np.dot(inv(self.P), (self.x_consensus - self.estimated_state))-np.dot(np.dot(self.H.T, inv(self.R)),(self.state-np.dot(self.H, self.state)))

    def calcular_hessian(self):
        self.hessian = inv(self.P) + np.dot(np.dot(self.H.T, inv(self.R)),self.H)

    def calcular_z(self, acel, sample_time):
        quaternion = pyquaternion.Quaternion(acel.orientation.w, acel.orientation.x, acel.orientation.y, acel.orientation.z)
        aceler = quaternion.rotate((acel.linear_acceleration.x, acel.linear_acceleration.y, acel.linear_acceleration.z-self.bias_z))

    	self.state[0] += sample_time*self.state[3] + (((sample_time)**2)/2.0)*(aceler[0]-self.bias_x)
        self.state[1] += sample_time*self.state[4] + (((sample_time)**2)/2.0)*(aceler[1]-self.bias_y)
        self.state[2] += sample_time*self.state[5] + (((sample_time)**2)/2.0)*(aceler[2])
        self.state[3] += sample_time*(aceler[0])
        self.state[4] += sample_time*(aceler[1])
        self.state[5] += sample_time*(aceler[2])

    def calcular_F(self, sample_time):
        #TODO: Poner como se calcula F.
        self.F[0,3] = sample_time
        self.F[1,4] = sample_time
        self.F[2,5] = sample_time

    def calcular_H(self, sample_time):
        self.H[0,3] = sample_time
        self.H[1,4] = sample_time
        self.H[2,5] = sample_time

    def calcular_P(self):
        self.P = inv(self.P) + np.dot(np.dot(self.H.T, inv(self.R)) , self.H)
        self.P = np.dot(np.dot(self.F,self.P), self.F.T) + self.Q

    def calcular_info(self, prices):
        self.PI = np.diag(self.hessian)*np.sum(prices.values())
        self.PI = np.reshape(self.PI, (-1,1))

    def calcular_x_barra(self): #Le quite lo del w_k por ahora. #, w_k):
        self.estimated_state = np.dot(self.F, self.x_consensus)# + w_k

    def calcular_x_consensus(self):
        try:
            self.delta_x = -np.dot(inv(self.hessian), self.grad) + self.PI
            self.x_consensus += self.s*self.delta_x
            self.mistico = copy.copy(self.x_consensus)
        except:
            self.x_consensus = self.mistico
            print("error extranio")
