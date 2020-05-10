#!/usr/bin/env python
import rospy
import sys

import math

import sys, unittest
import os, os.path, time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, PoseWithCovariance, TwistWithCovariance, Twist, Vector3, Wrench
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates

class imu1_node:
    def __init__(self,):
    	print('imu1_node: initializing node')
    	self.Pose_IMU = POSE()
    	self.done = False
    	self.imu_vx = 0
        self.imu_vy = 0
        self.imu_vz = 0
        self.first = True
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
       	rospy.Subscriber('/imu1/imu', Imu, self.imu_callback)
       	imu_pos = rospy.Publisher('/imu1/pos', Pose, queue_size=10)
       	while not rospy.is_shutdown():
       		if self.done = False: 
       			PoseIMU.position.x = self.imu_x	
       			PoseIMU.position.y = self.imu_y
       			PoseIMU.position.z = self.imu_z
       			PoseIMU.orientation.x = 0
       			PoseIMU.orientation.y = 0
       			PoseIMU.orientation.z = 0
       			PoseIMU.orientation.w = 0
       			imu_pos.push(PoseIMU)
       			
       		rate.sleep()

    def imu_callback(self, msg):
    	if self.first = False:
	    	self.imu_vx = self.imu_vx + msg.linear_acceleration.x*(1/50)
	        self.imu_vy = self.imu_vy + msg.linear_acceleration.y*(1/50)
	        self.imu_vz = self.imu_vz + msg.linear_acceleration.z*(1/50)
	    	self.imu_x = self.imu_x + self.imu_vx*(1/50)
	        self.imu_y = self.imu_y + self.imu_vy*(1/50)
	        self.imu_z = self.imu_z + self.imu_vz*(1/50)
	        self.done = True
        pass

    def model_states_callback(self, msg):
    	if self.first:
    		self.imu_x = msg.pose[1].position.x
        	self.imu_y = msg.pose[1].position.y
        	self.imu_z = msg.pose[1].position.z
        	self.first = False
    	self.x_gazebo = msg.pose[1].position.x
    	self.y_gazebo = msg.pose[1].position.y
    	self.z_gazebo = msg.pose[1].position.z
    	pass

if __name__ == '__main__':
    rospy.init_node('imu1_node', anonymous=True) #SE inicia el nodo
    try:
        node = imu1_node()
        rospy.spin()
    except rospy.ROSInterruptException:

        print('caught exception')
