#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Pose
from classes import State

class Link:
    def __init__(self, sensores):
        self.x = 1.0 #Esta se refiere a la variable de slack asociada a este enlace.
        self.price = 0 #TODO: Poner aqui un numero aleatorio. np.random()
        self.grad = 1.0
        self.hessian = 2.0
        self.mu = 1.0 #TODO: Mirar que valor toma. Por ahora en 0.
        self.initial_sum = 1 #La suma de PI mayuscula en la primera iteracion
        self.current_sum = 1 #La suma de PI mayuscula en la iteracion actual

    def reiniciar(self, grads, hessians, num_links):
        np_grads = np.array(grads.values())
        np_hessians = np.array(hessians.values())
        np_num_links = np.array(num_links.values())
        self.sum_sensor_info = np.sum(np_grads/np_hessians)
        pi_zeros = np_num_links/np_hessians
        self.initial_sum = np.sum(pi_zeros)

        self.calcular_gradiente()
        self.calcular_hessiana()

        self.current_sum = np.sum(pi_zeros)
        self.price = self.price = 1/(1/self.hessian + self.initial_sum)*(self.initial_sum-self.current_sum-self.sum_sensor_info - (self.grad/self.hessian))
        return self.price

    def calcular_gradiente(self):
        self.grad = -self.mu/self.x

    #Esto es el valor de la diagonal de la hessiana correspondiente a este enlace.
    def calcular_hessiana(self):
        self.hessian = self.mu/self.x**2 #FIXME: Revisar este calculo.

    def calcular_current_sum(self, PIs):
        self.current_sum = sum(PIs.values())

    def calcular_info(self, PIs):
        self.calcular_current_sum(PIs)
        self.price = 1/(1/self.hessian + self.initial_sum)*(self.initial_sum-self.current_sum-self.sum_sensor_info - (self.grad/self.hessian))
        #self.price = 2
        return self.price
