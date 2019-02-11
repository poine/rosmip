#!/usr/bin/env python
import time, math, numpy as np, sys
import rospy, rospkg, sensor_msgs.msg, nav_msgs.msg
import matplotlib.pyplot as plt

'''
This is an attempt at identifying rosmip
'''

import pdb

import julie_misc.plot_utils as jpu, julie_misc.utils as jmu
import rosmip_control.msg
'''
Records raw odometry and simulator truth to npz file
'''

class Node:
    def __init__(self):
        rospy.init_node('record_odometry')
        # Robot sensors: encoders, IMU
        rospy.Subscriber('/rosmip_balance_controller/debug_io', rosmip_control.msg.msg_debug_io, self.raw_odom_callback)
        self.lw_angle, self.rw_angle = [], []
        self.lw_pwm, self.rw_pwm = [], []
        self.imu_pitch, self.imu_pitch_dot = [], []
        self.odom_stamp = []

        # truth
        rospy.Subscriber('/rosmip/base_link_truth', nav_msgs.msg.Odometry, self.gazebo_truth_callback)
        self.truth_pos = []
        self.truth_ori = []
        self.truth_lvel = []
        self.truth_rvel = []
        self.truth_stamp = []


    def raw_odom_callback(self, msg):
        self.lw_angle += msg.lw_angle[:msg.nb_data]
        self.rw_angle += msg.rw_angle[:msg.nb_data]
        self.lw_pwm += msg.lw_pwm[:msg.nb_data]
        self.rw_pwm += msg.rw_pwm[:msg.nb_data]
        self.imu_pitch += msg.pitch[:msg.nb_data]
        self.imu_pitch_dot += msg.pitch_dot[:msg.nb_data]
        self.odom_stamp += [_s.to_sec() for _s in msg.stamp[:msg.nb_data]]

    def gazebo_truth_callback(self, msg):
        self.truth_pos.append( jmu.list_of_xyz(msg.pose.pose.position))
        self.truth_ori.append( jmu.list_of_xyzw(msg.pose.pose.orientation))
        self.truth_lvel.append( jmu.list_of_xyz(msg.twist.twist.linear))
        self.truth_rvel.append( jmu.list_of_xyz(msg.twist.twist.angular))
        self.truth_stamp.append(msg.header.stamp.to_sec())

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            print('recorded {} odometry and {} thruth'.format( len(self.odom_stamp), len(self.truth_stamp)))
            rate.sleep()


    def save(self, filename):
        print('saving to {}'.format(filename))
        np.savez(filename,
                 encoders_lw = np.array(node.lw_angle),
                 encoders_rw = np.array(node.rw_angle),
                 pwm_lw = np.array(node.lw_pwm),
                 pwm_rw = np.array(node.rw_pwm),
                 imu_pitch = np.array(node.imu_pitch),
                 imu_pitch_dot = np.array(node.imu_pitch_dot),
                 encoders_stamp = np.array(node.odom_stamp),
                 truth_pos   = np.array(node.truth_pos),
                 truth_ori   = np.array(node.truth_ori),
                 truth_lvel  = np.array(node.truth_lvel),
                 truth_rvel  = np.array(node.truth_rvel),
                 truth_stamp = np.array(node.truth_stamp))


            
if __name__ == '__main__':
    node = Node()
    try:
        node.run()
    except rospy.ROSInterruptException:
        print('recorded {} odometry and {} thruth'.format( len(node.odom_stamp), len(node.truth_stamp)))
    output_filename = '/tmp/rosmip_io_1' if len(sys.argv) < 2 else sys.argv[1]
    node.save(output_filename)
