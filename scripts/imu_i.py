#!/usr/bin/env python
import rospy
import sys

import math

import sys, unittest
import os, os.path, time
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, PoseWithCovariance, TwistWithCovariance, Twist, Vector3, Wrench
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates

class imu1_node:
    def __init__(self, namespace):
    	self.namespace = namespace
    	print(self.namespace + '_node: initializing node')
    	PoseIMU = Pose()
    	self.imu_vx = 0
        self.imu_vy = 0
        self.imu_vz = 0
        self.imu_x	= 0
        self.imu_y = 0
        self.imu_z = 0
    	self.done = False
        self.first = True
        self.done = False
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
       	rospy.Subscriber('/'+ self.namespace +'/imu', Imu, self.imu_callback)
       	imu_pos = rospy.Publisher('/imu1/pos', Pose, queue_size=10)
       	rate = rospy.Rate(50)
       	while not rospy.is_shutdown():
       		if self.done: 
       			PoseIMU.position.x = self.imu_x	
       			PoseIMU.position.y = self.imu_y
       			PoseIMU.position.z = self.imu_z
       			PoseIMU.orientation.x = 0
       			PoseIMU.orientation.y = 0
       			PoseIMU.orientation.z = 0
       			PoseIMU.orientation.w = 0
       			imu_pos.publish(PoseIMU)
       			#print(PoseIMU.position.x)
       		rate.sleep()

    def imu_callback(self, msg):
    	if self.first == False:
	    	self.imu_vx = self.imu_vx + (msg.linear_acceleration.x+0.0021)*(1/50.0)
	        self.imu_vy = self.imu_vy + (msg.linear_acceleration.y+0.0076)*(1/50.0)
	        self.imu_vz = self.imu_vz + (msg.linear_acceleration.z-9.8)*(1/50.0)
	    	self.imu_x = self.imu_x + self.imu_vx/50.0 + (1/50.0)*(msg.linear_acceleration.x+0.0021)*(1/50.0)**2
	        self.imu_y = self.imu_y + self.imu_vy/50.0 + (1/50.0)*(msg.linear_acceleration.y+0.0076)*(1/50.0)**2
	        self.imu_z = self.imu_z + self.imu_vz/50.0 + (1/50.0)*(msg.linear_acceleration.z-9.8)*(1/50.0)**2
	        print(msg.linear_acceleration.x)
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
    try:
    	if len(sys.argv) < 2:
            print("usage needs 1 parameter for now, ejm: imu1")
        else:
        	rospy.init_node(sys.argv[1] + '_node', anonymous=True) #SE inicia el nodo
        node = imu1_node(sys.argv[1])
        rospy.spin()
    except rospy.ROSInterruptException:

        print('caught exception')
