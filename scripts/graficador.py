#!/usr/bin/env python
#Las librerias que se importan
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.msg import ModelStates
import mpl_toolkits.mplot3d.axes3d as p3



#En este metodo se inicializa el nodo y se suscribe a los topicos de la posicion actual del robot y los puntos detectados.
class graficardor():
    def __init__(self,):
        global axs
        self.fig = plt.figure()
        axs = p3.Axes3D(self.fig)
        rospy.init_node('graficador', anonymous=True)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        self.ani = animation.FuncAnimation(self.fig, self.animate)
        plt.show()
    
    def model_states_callback(self, msg):
        global x_gazebo, y_gazebo, z_gazebo
        x_gazebo = msg.pose[1].position.x
        y_gazebo = msg.pose[1].position.y
        z_gazebo = msg.pose[1].position.z
        pass

    def animate(i,j):
        global axs, x_gazebo, y_gazebo, z_gazebo
        axs.clear()
        axs.set_xlim3d([-50, 50])
        axs.set_ylim3d([-50, 50])
        axs.set_zlim3d([-50, 0])
        axs.scatter(x_gazebo, y_gazebo,z_gazebo, c = 'b')
        plt.title('Rover real time position')
        plt.grid()

if __name__ == '__main__':
    try:
        graficardor()
    except rospy.ROSInterruptException:
        pass