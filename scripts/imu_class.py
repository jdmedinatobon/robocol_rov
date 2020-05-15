#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Pose
from classes import State
from pyquaternion import Quaternion
from threading import Event

class IMU:
    def __init__(self, enlaces):
        self.state = State()
        self.estimated_state = State()

        self.bias_x = 0
        self.bias_y = 0
        self.bias_z = 9.8

        #TODO: Las siguientes matrices todas toca cambiarlas por sus valores reales.

        #Matriz de covarianza del ruido del sistema
        self.Q = np.diag(np.ones(3))

        #Matriz de covarianza del ruido de la medida
        self.R = np.diag(np.ones(3))

        #Matriz de covarianza del error. Lo que es Pk en el paper.
        self.P = np.zeros((3,3))

        #Matriz de restriccion utilizada en el problema de optimizacion
        self.C = np.ones((3,3))

        self.grad = 0
        self.hessian = 0

        self.link_info = {e : 0 for e in enlaces}
        self.num_links = len(enlaces)

        #Por ahora lo de ver cuando estan actualizados chambon con otro diccionario
        self.is_link_info_new = {e : 0 for e in enlaces}

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

        return self.grad

    def calcular_hessian(self):
        #TODO: Poner aqui como calcular la hessiana

        return self.hessian

    def calcular_info(self, info):
        #Aqui primero se espera a que se tengan las nuevas medidas del gradiente
        #Cuando se tengan, se calculan los nuevos grad y hessian y se retornan para
        #ser publicados por el nodo.

        #Cuando ambos se actualicen calcular w y volver a calcular grad y hessian
        #por ahora feo con un while ahi
        #self.guardar_info(info)

        # if np.sum(self.is_link_info_new.values()) == self.num_links:
        #     #TODO: Terminar esto. Hay que calcular los nuevos grad y hessian.
        #     #Calcular los nuevos grad y hessian y retornarlos para publicar
        #     #Tambien se indica que los valores de grad y hessian son viejos
        #     self.is_link_info_new = dict.fromkeys(self.is_link_info_new, 0)
        #
        #     return self.grad, self.hessian
        # else:
        #     #Si entra aqui es porque no tiene los valores actualizados de sus vecinos
        #     #Se retorna -1 para indicarle al nodo que no publique
        #     print("No deberia entrar aqui: Suma = {}".format(np.sum(self.is_link_info_new.values())))
        #     return -1
        return self.grad, self.hessian

    def guardar_info(self, info):
        self.link_info[info.id] = info.price
        self.is_link_info_new[info.id] = 1

        print("Link info = ".format(self.link_info))
