#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Pose
from classes import State

class Link:
    def __init__(self, sensores):
        self.x = 0 #Esta se refiere a la variable de slack asociada a este enlace.
        self.price = 0 #TODO: Poner aqui un numero aleatorio. np.random()
        self.grad = 0
        self.hessian = 0
        self.mu = 1 #TODO: Mirar que valor toma.
        self.initial_sum = 1 #La suma de PI mayuscula en la primera iteracion
        self.current_sum = 1 #La suma de PI mayuscula en la iteracion actual

        self.sensor_info = {s : 0 for s in sensores} #Estos serian los PI mayuscula.
        self.is_sensor_info_new = {s : 0 for s in sensores}

    def reiniciar(self):
        self.sum_sensor_info = 1#np.sum(grads/hessians)
        self.initial_sum = 1#np.sum(pi_zero)

        self.price = np.random.rand()
        return self.price

    def calcular_gradiente(self):
        self.grad = -mu/self.x

    #Esto es el valor de la diagonal de la hessiana correspondiente a este enlace.
    def calcular_hessiana(self):
        self.hessiana = mu/self.x**2 #FIXME: Revisar este calculo.

    def calcular_current_sum(self, pi_zero):
        self.current_sum = np.sum(pi_zero)

    def calcular_info(self):
        self.price = 1/(1/self.hessian + self.initial_sum)*(self.initial_sum-self.current_sum-self.sum_sensor_info - (self.grad/self.hessian))
        return self.price
