#!/usr/bin/env python

import logging, os
import time
import math, numpy as np, matplotlib.pyplot as plt
import rospy, tf, tf2_ros
import nav_msgs.msg, geometry_msgs, sensor_msgs, geometry_msgs, rosmip_control.msg

class RosNode:
    def __init__(self):
        rospy.init_node('TestController')
        self.vel_pub = rospy.Publisher("/rosmip_balance_controller/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        rospy.Subscriber("/rosmip_balance_controller/odom", nav_msgs.msg.Odometry, self.odom_callback)
        rospy.Subscriber("/imu", sensor_msgs.msg.Imu, self.imu_callback)
        rospy.Subscriber("/rosmip_balance_controller/debug", rosmip_control.msg.debug, self.debug_callback)
        self.rvels = []
        
    def odom_callback(self, msg):
        #print msg
        self.pos = msg.pose.pose.position

    def imu_callback(self, msg):
        #print msg.angular_velocity.y
        self.rvels.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        

    def debug_callback(self, msg):
        #print msg
        pass
        
    def publish_vel(self, lin, ang):
        msg = geometry_msgs.msg.Twist()
        msg.linear = geometry_msgs.msg.Vector3(lin, 0, 0)
        msg.angular = geometry_msgs.msg.Vector3(0, 0, ang)
        self.vel_pub.publish(msg)

        
    def straight_line(self, len=1.):
        pass
        
        
    def run(self):
        rate = rospy.Rate(25.)
        while not rospy.is_shutdown() and len(self.rvels) < 1000:
            self.publish_vel(0., 1.5)
            rate.sleep()
        self.plot()

    def plot(self):
        d = np.array(self.rvels)
        plt.plot(d[:,0])
        plt.plot(d[:,1])
        plt.plot(d[:,2])
        plt.show()


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    try:
        RosNode().run()
    except rospy.ROSInterruptException:
        print 'bye bye'
