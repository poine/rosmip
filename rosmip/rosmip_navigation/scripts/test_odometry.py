#!/usr/bin/env python

import logging, os, time
import math, numpy as np, matplotlib.pyplot as plt
import rospy, tf

import pdb

import test_controller as tc

class RosNode(tc.Trajectory):

    def __init__(self):
        rospy.init_node('TestController')
        stages = (tc.Line(lin=0.1),  tc.Line(lin=0.), tc.Line(ang=math.pi/4),  tc.Line(lin=0.))
        tc.Trajectory.__init__(self, stages)
        self.true_pos, self.odom_pos = [], []
        self.true_ori, self.odom_ori = [], []
        self.true_vel, self.odom_vel = [], []
        self.true_rvel, self.odom_rvel = [], []
        self.tl, self.ol = tc.TruthListener(), tc.OdomListener()

    def record(self):
         if self.ol.pose is not None and self.tl.pose is not None:
             true_pos, odom_pos = [tc.list_of_position(p) for p in [self.tl.pose.position, self.ol.pose.position]]
             self.true_pos.append(true_pos); self.odom_pos.append(odom_pos)
             true_ori, odom_ori = [tc.list_of_orientation(o) for o in [self.tl.pose.orientation, self.ol.pose.orientation]]
             self.true_ori.append(true_ori); self.odom_ori.append(odom_ori)  
             true_vel, odom_vel =  [tc.list_of_position(v) for v in [self.tl.twist.linear, self.ol.twist.twist.linear]]
             self.true_vel.append(true_vel); self.odom_vel.append(odom_vel)
             true_rvel, odom_rvel =  [tc.list_of_position(v) for v in [self.tl.twist.angular, self.ol.twist.twist.angular]]
             self.true_rvel.append(true_rvel); self.odom_rvel.append(odom_rvel)

             print('{}'.format(len(self.true_pos)))


    def plot(self):
        self.plot_position()
        self.plot_orientation()

    def plot_position(self):
        fig = plt.figure()
        fig.canvas.set_window_title("position")
        tp, op = np.array(self.true_pos), np.array(self.odom_pos)
        for i, c in enumerate(["x", "y", "z"]):
            plt.subplot(3,1,i+1)
            plt.plot(tp[:,i])
            plt.plot(op[:,i])
            plt.title(c); plt.legend(["truth", "odom"])

    def plot_orientation(self):
        fig = plt.figure()
        fig.canvas.set_window_title("orientation")
        for i, c in enumerate(["roll", "pitch", "yaw"]):
            plt.subplot(3,1,i+1)
            plt.plot(self.true_euler[:,i])
            plt.plot(self.odom_euler[:,i])
            plt.title(c); plt.legend(["truth", "odom"])

    def plot_orientation_error(self):
        fig = plt.figure()
        fig.canvas.set_window_title("orientation error")
        # TODO

    def filter(self):
        self.true_euler = np.array([tc.euler_of_quat(q) for q in self.true_ori])
        self.odom_euler = np.array([tc.euler_of_quat(q) for q in self.odom_ori])

        true_yaw_dot, odom_yaw_dot = [],[]
        for i in range(len(self.true_euler)-1):
            true_yaw_dot.append(self.true_euler[i,2]-self.true_euler[i-1,2])
            odom_yaw_dot.append(self.odom_euler[i,2]-self.odom_euler[i-1,2])
        fig = plt.figure()
        fig.canvas.set_window_title("velocites")

        plt.subplot(2,1,1)
        plt.title('linear')
        tlv, olv = np.linalg.norm(np.array(self.true_vel), axis=1), np.linalg.norm(np.array(self.odom_vel), axis=1)
        plt.plot(tlv); plt.plot(olv)
        plt.legend(["truth", "odom"])

        plt.subplot(2,1,2)
        plt.title('angular')
        trv, orv = np.array(self.true_rvel), np.array(self.odom_rvel)
        plt.plot(trv[:,2]); plt.plot(orv[:,2])
        plt.legend(["truth", "odom"])
        
        
    def run(self, nrec=400):
        rate = rospy.Rate(25.)
        tc.Trajectory.start(self, rospy.Time.now())
        while not rospy.is_shutdown() and len(self.true_pos) < nrec:
            self.record()
            tc.Trajectory.run(self, rospy.Time.now())
            rate.sleep()
        self.filter()
        self.plot()
        plt.show()

    


            
if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    np.set_printoptions(precision=3, linewidth=300)
    try:
        RosNode().run(2000)
    except rospy.ROSInterruptException:
        print 'bye bye'
