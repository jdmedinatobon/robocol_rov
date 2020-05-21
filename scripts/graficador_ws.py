#!/usr/bin/env python
#Las librerias que se importan
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from robocol_rov.msg import LinkInfo, ConsensusInfo,ImuInfo
import numpy as np

class graficador_ws():
    def __init__(self,):
        global axs, flag_w_1, flag_w_2, ws_1, ws_2
        self.fig = plt.figure()
        axs = self.fig.add_subplot(111)
        flag_w_1 = False
        flag_w_2 = False
        ws_1 = []
        ws_2 = []

        rospy.init_node('graficador_w', anonymous=True)
        rospy.Subscriber('/link1/info', LinkInfo, self.w_1)
        rospy.Subscriber('/link2/info', LinkInfo, self.w_2)

        self.ani = animation.FuncAnimation(self.fig, self.animate)
        plt.show()

    def w_1(self, msg):
        global ws_1, flag_w_1
        ws_1 = np.concatenate((ws_1,msg.price[4]), axis=None)
        flag_w_1 =True
        #print(np.array(msg.price[0]))

    def w_2(self, msg):
        global ws_2, flag_w_2
        ws_2 = np.concatenate((ws_2,msg.price[4]), axis=None)
        flag_w_2 =True

    def animate(i,j):
        global flag_w_1, flag_w_2, ws_1, ws_2

        if  flag_w_1 and flag_w_2:
            #axs.clear()
            axs.plot(ws_1, c = 'r')
            axs.plot(ws_2, c = 'g')
            plt.title("Valor de los ws para cada link en la posicion 0")
            axs.legend(("w link 1","w link 2"))
            plt.grid()


if __name__ == '__main__':
    try:
        graficador_ws()
    except rospy.ROSInterruptException:
        pass
