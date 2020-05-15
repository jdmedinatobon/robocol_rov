#!/usr/bin/env python
#Las librerias que se importan
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from gazebo_msgs.msg import ModelStates
import mpl_toolkits.mplot3d.axes3d as p3
from geometry_msgs.msg import Pose

class graficador():
    def __init__(self,):
        global axs, flag_1, flag_2, flag_3
        self.fig = plt.figure()
        axs = p3.Axes3D(self.fig)
        flag_1 = False
        flag_2 = False
        flag_3 = False
        rospy.init_node('graficador', anonymous=True)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        rospy.Subscriber('/imu1/pos', Pose, self.imu_1)
        rospy.Subscriber('/imu2/pos', Pose, self.imu_2)
        rospy.Subscriber('/imu3/pos', Pose, self.imu_3)
        self.ani = animation.FuncAnimation(self.fig, self.animate)
        plt.show()

    def model_states_callback(self, msg):
        global x_gazebo, y_gazebo, z_gazebo
        x_gazebo = msg.pose[1].position.x
        y_gazebo = msg.pose[1].position.y
        z_gazebo = msg.pose[1].position.z
        pass
    def imu_1(self, msg):
        global x_imu_1, y_imu_1, z_imu_1, flag_1
        x_imu_1 = msg.position.x
        y_imu_1 = msg.position.y
        z_imu_1 = msg.position.z
        flag_1 = True
        pass
    def imu_2(self, msg):
        global x_imu_2, y_imu_2, z_imu_2, flag_2
        x_imu_2 = msg.position.x
        y_imu_2 = msg.position.y
        z_imu_2 = msg.position.z
        flag_2 = True
        pass
    def imu_3(self, msg):
        global x_imu_3, y_imu_3, z_imu_3, flag_3
        x_imu_3 = msg.position.x
        y_imu_3 = msg.position.y
        z_imu_3 = msg.position.z
        flag_3 = True
        pass

    def animate(i,j):
        global axs, x_gazebo, y_gazebo, z_gazebo, x_imu_1, y_imu_1, z_imu_1, x_imu_2, y_imu_2, z_imu_2, x_imu_3, y_imu_3, z_imu_3, flag_1, flag_2, flag_3
        axs.clear()
        axs.set_xlim3d([-50, 50])
        axs.set_ylim3d([-50, 50])
        axs.set_zlim3d([-50, 0])
        axs.scatter(x_gazebo, y_gazebo,z_gazebo, c = 'b')
        if flag_1: axs.scatter(x_imu_1, y_imu_1, z_imu_1, c = 'r')
        if flag_2: axs.scatter(x_imu_2, y_imu_2, z_imu_2, c = 'g')
        if flag_3: axs.scatter(x_imu_3, y_imu_3, z_imu_3, c = 'y')
        plt.title('ROV real time position')
        plt.grid()

if __name__ == '__main__':
    try:
        graficador()
    except rospy.ROSInterruptException:
        pass
