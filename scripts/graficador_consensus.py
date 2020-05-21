#!/usr/bin/env python
#Las librerias que se importan
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from robocol_rov.msg import LinkInfo, ConsensusInfo
import numpy as np

class graficador_consensus():
    def __init__(self,):
        global axs, flag_cons_1, flag_cons_2, flag_cons_3,flag_cons_4, hist_1,hist_2,hist_3,hist_real,hist_4
        self.fig = plt.figure()
        axs = self.fig.add_subplot(111)
        flag_cons_1 = False
        flag_cons_2 = False
        flag_cons_3 = False
        flag_cons_4 = False

        hist_1 = []
        hist_2 = []
        hist_3 = []
        hist_4 = []
        hist_real = []
        x_gazebo, y_gazebo, z_gazebo = 0, 0, 0
        rospy.init_node('graficador', anonymous=True)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        rospy.Subscriber('/imu1/cons_info', ConsensusInfo, self.con_1)
        rospy.Subscriber('/imu2/cons_info', ConsensusInfo, self.con_2)
        rospy.Subscriber('/imu3/cons_info', ConsensusInfo, self.con_3)
        rospy.Subscriber('/imu4/cons_info', ConsensusInfo, self.con_4)


        self.ani = animation.FuncAnimation(self.fig, self.animate)
        plt.show()

    def con_1(self, msg):
        global cons_1_value, flag_cons_1
        cons_1_value = np.array(msg.consensus)
        flag_cons_1 =True

    def con_2(self, msg):
        global cons_2_value, flag_cons_2
        cons_2_value = np.array(msg.consensus)
        flag_cons_2 = True

    def con_3(self, msg):
        global cons_3_value, flag_cons_3, hist_real
        cons_3_value = np.array(msg.consensus)
        flag_cons_3 = True
    def con_4(self, msg):
        global cons_4_value, flag_cons_4
        cons_4_value = np.array(msg.consensus)
        flag_cons_4 = True

    def model_states_callback(self, msg):
        global x_gazebo, y_gazebo, z_gazebo,hist_real
        x_gazebo = msg.pose[1].position.x
        y_gazebo = msg.pose[1].position.y
        z_gazebo = msg.pose[1].position.z
        

    def animate(i,j):
        global cons_1_value,cons_2_value,cons_3_value,cons_4_value,flag_cons_1,flag_cons_4, flag_cons_2, flag_cons_3,hist_1,hist_2,hist_3,hist_real,hist_4

        if  flag_cons_1 and flag_cons_2 and flag_cons_3 and flag_cons_4:
            hist_real = np.concatenate((hist_real, z_gazebo), axis=None) 
            hist_1 = np.concatenate((hist_1, cons_1_value[-1]), axis=None)
            hist_2 = np.concatenate((hist_2, cons_2_value[-1]), axis=None)
            hist_3 = np.concatenate((hist_3, cons_3_value[-1]), axis=None)
            hist_4 = np.concatenate((hist_4, cons_4_value[-1]), axis=None)

            axs.plot(hist_real, c = 'b' )
            axs.plot(hist_1, c = 'r')
            axs.plot(hist_2, c = 'g')
            axs.plot(hist_3, c = 'y')
            axs.plot(hist_4)
            plt.title("Valor de los consensus para cada IMU en la posicion x")
            axs.legend(("Posicion real","Consensus IM1 1","Consensus IM1 2", "Consensus IM1 3", "Consensus IM1 4"))
            flag_cons_1 = False
            flag_cons_2 = False
            flag_cons_3 = False
            flag_cons_4 = False
            plt.grid()

if __name__ == '__main__':
    try:
        graficador_consensus()
    except rospy.ROSInterruptException:
        pass
