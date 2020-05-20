#!/usr/bin/env python
import numpy as np
from numpy.linalg import inv
import copy
import rospy
from geometry_msgs.msg import Pose
from classes import State
from pyquaternion import Quaternion
from threading import Event
from sympy import symbols, Matrix, Transpose
from sympy import *

class IMU:
    def __init__(self, enlaces):
        self.state = np.ones((6,1)) # esto en el paper es equivalente a z, la estimacion del sensor
        self.estimated_state = np.zeros((6,1)) # esto en el paper es equivalente a x barra, la estimacion realizada a partir de nuestro modelo
        self.x_consensus = np.zeros((6,1)) # esto es lo que esperamos llegue a ser magico y converger a un valor global para todos los sensores... mistico? tal vez. Hotel? Trivago

        self.bias_x = 0
        self.bias_y = 0
        self.bias_z = 9.8

        self.s = 0.00001 #Step size

        #Esta es la matriz Fk del modelo
        self.F = np.eye(6)

        #TODO: Las siguientes matrices todas toca cambiarlas por sus valores reales.
        self.H = np.eye(6)

        #Matriz de covarianza del ruido del sistema
        self.Q = np.eye(6)

        #Matriz de covarianza del ruido de la medida
        self.R = np.eye(6)

        #Matriz de covarianza del error. Lo que es Pk barra en el paper.
        self.P =copy.copy(self.Q)#np.zeros((6,6))

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

    def actualizar(self, acel, sample_time):
        self.calcular_z(acel, sample_time)

        #TODO: Terminar este metodo de F
        self.calcular_F(sample_time)

        self.calcular_H(sample_time)
        self.calcular_P()

    def calcular_grad(self):
        #TODO: Poner aqui como calcular el gradiente
        self.grad = np.dot(inv(self.P), self.x_consensus - self.estimated_state) - np.dot(inv(self.P), (self.x_consensus - self.estimated_state))-np.dot(np.dot(self.H.T, inv(self.R)),(self.state-np.dot(self.H, self.state)))

    def calcular_hessian(self):
        #TODO: Poner aqui como calcular la hessiana
        self.hessian = inv(self.P) + np.dot(np.dot(self.H.T, inv(self.R)),self.H)

    def calcular_z(self, acel, sample_time):
        quaternion = Quaternion(acel.orientation.x,acel.orientation.y,acel.orientation.z,acel.orientation.w)
        aceler = quaternion.rotate(np.array([acel.linear_acceleration.x, acel.linear_acceleration.y, acel.linear_acceleration.z-self.bias_z]))

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
        self.delta_x = -np.dot(inv(self.hessian), self.grad) + self.PI
        self.x_consensus = self.x_consensus + self.s*self.delta_x

    def calcular_Fk_modelo(self):
		p1, p2, p3, p4, p5, p6 = symbols('p1 p2 p3 p4 p5 p6')
		p = Matrix([[p1, p2, p3,0,0,0,p4, p5, p6,0,0,0]])
		nx = 12
		nu = 6

				#Parametros de aceleracion
		X_ax = -167.6 #Kg
		Y_ay = -477.2 #Kg
		Z_az = -235.7 #Kg
		K_alphax = -11.6 #Kg*m2
		M_alphay = -15.5 #Kg*m2
		N_alphaz = -15.9 #Kg*m2

		#Parametros de velocidad
		X_vx = 26.9 #Kg/s
		Y_vy = 35.8 #Kg/s
		Z_vz = 6.19 #Kg/s
		K_ox = 3.0 #Kg*m2/s*rad
		M_oy = 4.9 #Kg*m2/s*rad
		N_oz = 3.5 #Kg*m2/s*rad

		#Otros parametros
		m = 116 #Kg
		g = 9.8 #N/Kg (Gravedad)
		b = 116.2 #Kg
		W = m*g 
		B = b*g

		I_x = 9.3 #Kg*m2
		I_y = 14.9 #Kg*m2
		I_z = 13.1 #Kg*m2
		x_B = -0.00045 #m
		y_B = -0.00128 #m
		z_B = -0.04298 #m
		X_vx_vx = 241.3 #Kg/m
		Y_vy_vy = 503.8 #Kg/m
		Z_vz_vz = 119.1 #Kg/m
		K_ox_ox = 101.6 #Kg*m2/rad2
		M_oy_oy = 59.9 #Kg*m2/rad2
		N_oz_oz = 76.9 #Kg*m2/rad2

		a,b,c=symbols('a b c')
				#Sistema No Lineal

		G1=Matrix([[cos(p[4])*cos(p[5]),      sin(p[3])*sin(p[4])*cos(p[5])-cos(p[3])*sin(p[5]),      cos(p[3])*sin(p[4])*cos(p[5])+sin(p[3])*sin(p[5])],

									[cos(p[4])*sin(p[5]),     sin(p[3])*sin(p[4])*sin(p[5])+cos(p[3])*cos(p[5]),      cos(p[3])*sin(p[4])*sin(p[5])-sin(p[3])*cos(p[5])], 

									[-sin(p[4]),              sin(p[3])*cos(p[4]),                                    cos(p[3])*cos(p[4])]])
		G2 = Matrix([[1,                        sin(p[3])*tan(p[4]),                                    cos(p[3])*tan(p[4])],

									[0,						            cos(p[3]),                                              -sin(p[3])], 

									[0,                        sin(p[3])*sec(p[4]),                                   cos(p[3])*sec(p[4])]])
		G = BlockMatrix([[G1,ones(3,3)],[zeros(3,3), G2]])#, np.concatenate((np.zeros((3,3)), G2), axis = 1)), axis = 0 )

		ds1 = G1*Matrix(p[6:9]);
		ds2 = G2*Matrix(p[9:12]);

		M_RB = diag(m, m, m, I_x, I_y, I_z)
		M_A = diag(-X_ax, -Y_ay, -Z_az, -K_alphax, -M_alphay, -N_alphaz)
		M = M_RB + M_A



		C_RB = Matrix([[0,       0,       0,            0,       m*p[8],    -m*p[7]],
				           [0,       0,       0,        -m*p[9],        0,       m*p[6]],
				           [0,       0,       0,         m*p[7],    -m*p[6],        0],
				           [0,      m*p[8], -m*p[7],        0,       I_z*p[11], -I_y*p[10]],
				           [-m*p[8],   0,    m*p[6],    -I_z*p[11],     0,       I_x*p[9]],
				           [m*p[7], -m*p[6],   0,         I_y*p[10], -I_x*p[9],     0    ]])


		C_A = Matrix([ [0,               0,         0,           0,                    -Z_az*p[8],           Y_ay*p[7]],
				           [0,               0,         0,          Z_az*p[8],                0,                -X_ax*p[6]],
				           [0,               0,         0,          -Y_ay*p[7],              X_ax*p[6],         0 ],
				           [0,           -Z_az*p[8],   Y_ay*p[7],    0,                     -N_alphaz*p[11],    M_alphay*p[10]],
				           [Z_az*p[8],        0,        -X_ax*p[6],   N_alphaz*p[11],           0,                -K_alphax*p[9]],
				           [-Y_ay*p[7],      X_ax*p[6],      0,        -M_alphay*p[10],        K_alphax*p[9],         0 ]])


		C = C_RB + C_A

		D = diag(X_vx+X_vx_vx*abs(p[6]) , Y_vy+Y_vy_vy*abs(p[7]), Z_vz+Z_vz_vz*abs(p[8]),
				          K_ox+K_ox_ox*abs(p[9]), M_oy+M_oy_oy*abs(p[10]), N_oz+N_oz_oz*abs(p[11])
				          )
		#-----------------------------------
		# De aqui en adelante aun no funciona
		#-----------------------------------
		
		gs = [  (W-B)*sin(p(5))                                      ;
		       -(W-B)*cos(p(5))*sin(p(4))                            ;
		       -(W-B)*cos(p(5))*cos(p(4))                            ;
		        y_B*B*cos(p(5))*cos(p(4)) - z_B*B*cos(p(5))*sin(p(4));
		       -z_B*B*sin(p(5))           - x_B*B*cos(p(5))*cos(p(4));
		        x_B*B*cos(p(5))*sin(p(4)) + y_B*B*sin(p(5))         
		        ];

		#dv = M\(tao-C*p(7:12)-D*p(7:12)-gs);

		dp = [ds1; ds2; dv];

		A = zeros(nx,nx);
		for i=1:nx
		   for j=1:nx 
		        derivada = diff(dp(i),p(j));
		        A(i,j) = subs(derivada, [p;tao], equils);
		   end
		end

		def linealizar(self,):
