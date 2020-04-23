import rospy
import math

import sys, unittest
import os, os.path, time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, PoseWithCovariance, TwistWithCovariance, Twist, Vector3, Wrench
from gazebo_msgs.srv import ApplyBodyWrench
import tf.transformations as tft
from numpy import float64




### NODO PRINCIPAL ###
def node_move_rov():
    #creacion del nodo
    rospy.init_node('node_move_rov',anonymous=True)
    #se subscribe al topico traction orders
	rospy.wait_for_service('/gazebo/apply_body_wrench')
	apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    

	wrench = Wrench()
	wrench.force.x = 0
	wrench.force.y = 0
	wrench.force.z = 0
	wrench.torque.x = 6
	wrench.torque.y = 0
	wrench.torque.z = 0
    rate = rospy.Rate (10)
    while not rospy.is_shutdown ():
        rate.sleep ()

def enviarMensajeInicializacion():
    global EnviarMensaje
    contador=0
    while EnviarMensaje:
        nMsg = 1  # 3
        WriteFPGA("A" + str(nMsg) + "#I0#I1#I2#I3#I4#I5#")
        time.sleep(1)
        contador=contador+1
        if contador>4:
            EnviarMensaje=False


###NODOS PARA TOPICOS ###
def traction_Orders_Callback(param):
    global traction_present
    traction_present=param
    procesarJoystick(traction_present.rpm_l, traction_present.rpm_r)
    pass

#def connection_Callback(param):
  #  pass

def arm_Orders_Callback(param):
    global arm_present
    arm_present=param.message
    almacenar=WriteFPGA(arm_present)
    pass


def handle_enable(param):
    mensage=param.message
    almacenar2=WriteFPGA(mensage)
    return []

###METODOS EXTERNOS A ROS####

if __name__ == '__main__':
    try:
        node_fpga()
    except rospy.ROSInterruptException:
        pass

