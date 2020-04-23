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
from uuv_sensor_ros_plugins_msgs.msg import DVL

class kalman_filter_node:
    def __init__(self, namespace):
        print('kalman_filter_node: initializing node')
        self.namespace = namespace
        self.imu = rospy.Subscriber(self.namespace + '/imu', Imu, self.imu_callback)
        self.pos = rospy.Subscriber(self.namespace + '/dvl', DVL, self.pos_callback)
        

    def imu_callback(self, msg):
        print(msg)
        pass
    
    def pos_callback(slef, msg):
        #print(msg)
        pass


if __name__ == '__main__':
    rospy.init_node('kalman_filter_node', anonymous=True) #SE inicia el nodo
    try:
        print('Starting kalman_filter_node' )
        if len(sys.argv) < 2:
            print("usage needs parameters")
        else:
            node = kalman_filter_node(sys.argv[1])
            rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
