#!/usr/bin/env python
#Las librerias que se importan
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from robocol_rov.msg import LinkInfo, ConsensusInfo
import numpy as np

class graficador():
    def __init__(self,):
        global axs, flag_cons_1, flag_cons_2, flag_cons_3, hist_1,hist_2,hist_3
        self.fig = plt.figure()
        axs = self.fig.add_subplot(111)
        flag_cons_1 = False
        flag_cons_2 = False
        flag_cons_3 = False
        hist_1 = []
        hist_2 = []
        hist_3 = []

        x_gazebo, y_gazebo, z_gazebo = 0, 0, 0
        rospy.init_node('graficador', anonymous=True)
        rospy.Subscriber('/imu1/cons_info', ConsensusInfo, self.con_1)
        rospy.Subscriber('/imu2/cons_info', ConsensusInfo, self.con_2)
        rospy.Subscriber('/imu3/cons_info', ConsensusInfo, self.con_3)


        self.ani = animation.FuncAnimation(self.fig, self.animate)
        plt.show()

    def con_1(self, msg):
        global cons_1_value, flag_cons_1
        cons_1_value = msg.consensus
        flag_cons_1 =True

    def con_2(self, msg):
        global cons_2_value, flag_cons_2
        cons_2_value = msg.consensus
        flag_cons_2 = True

    def con_3(self, msg):
        global cons_3_value, flag_cons_3
        cons_3_value = msg.consensus
        flag_cons_3 = True



    def animate(i,j):
        global cons_1_value,cons_2_value,cons_3_value,flag_cons_1, flag_cons_2, flag_cons_3,hist_1,hist_2,hist_3

        if  flag_cons_1 and flag_cons_2 and flag_cons_3:
            hist_1 = np.concatenate((hist_1, cons_1_value), axis=None)
            hist_2 = np.concatenate((hist_2, cons_2_value), axis=None)
            hist_3 = np.concatenate((hist_3, cons_3_value), axis=None)
            print(len(cons_1_value))
            axs.clear()
            axs.plot(hist_1, c = 'r')
            axs.plot(hist_2, c = 'g')
            axs.plot(hist_3, c = 'y')
            plt.title("Valor de los consensus para cada IMU en la posicion x")
            axs.legend(("Consensus IM1 1","Consensus IM1 2", "Consensus IM1 3"))
            flag_cons_1 = False
            flag_cons_2 = False
            flag_cons_3 = False        
            plt.grid()


if __name__ == '__main__':
    try:
        graficador()
    except rospy.ROSInterruptException:
        pass
