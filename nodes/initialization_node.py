#!/usr/bin/env python

import rospy
import serial
import string
import math
import sys
import numpy as np
import time

from razor_imu_9dof.msg import ImuStates
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from dynamic_reconfigure.server import Server
from razor_imu_9dof.cfg import imuConfig
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

#######################################################################

class Read_IMU:

  def __init__(self):

     self.imu_sub = rospy.Subscriber("imu",Imu, self.callback)
     self.imu_pub = rospy.Publisher("imu_msg", ImuStates, queue_size=1)

     self.acc_x = []
     self.acc_y = []
     self.theta = []
     self.t_0 = rospy.get_rostime()

     self.rad2degrees = 180.0/math.pi

  def callback(self,imu):

     quaternion = (
       imu.orientation.x,
       imu.orientation.y,
       imu.orientation.z,
       imu.orientation.w)
     (roll,pitch,yaw) = euler_from_quaternion(quaternion)


     t_duration = imu.header.stamp - self.t_0
     t_nsecs = int(t_duration.nsecs)

      
     if t_nsecs<=5000000000:
       self.acc_x.append(imu.linear_acceleration.x)
       self.acc_y.append(imu.linear_acceleration.y)


     init_acc_x = np.mean(self.acc_x)
     init_acc_y = np.mean(self.acc_y)
        
     msg = ImuStates()
     msg.header.stamp = t_duration
     msg.state_accx = (imu.linear_acceleration.x - init_acc_x)
     msg.state_accy = (imu.linear_acceleration.y - init_acc_y)
     msg.state_theta = (yaw)
     self.imu_pub.publish(msg)

#######################################################################
      
if __name__ == '__main__': 
    rospy.init_node('Read_IMU',anonymous=False)
    node = Read_IMU()
    rospy.spin()
