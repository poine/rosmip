#!/usr/bin/env python

import logging, os
import time
import math, numpy as np, matplotlib.pyplot as plt
import rospy, tf, tf2_ros
import nav_msgs.msg, geometry_msgs, sensor_msgs, geometry_msgs, rosmip_control.msg, gazebo_msgs.msg
import pdb

def list_of_position(p): return (p.x, p.y, p.z)
def list_of_orientation(q): return (q.x, q.y, q.z, q.w)
def euler_of_quat(q): return tf.transformations.euler_from_matrix(tf.transformations.quaternion_matrix(q))

''' Subscribe to gazebo/model_states messages and store the robot pose and twist '''
class TruthListener:
    def __init__(self, model_name="rosmip"):
        self.model_name, self.model_idx = model_name, None
        rospy.Subscriber('/gazebo/model_states', gazebo_msgs.msg.ModelStates, self.callback)
        self.pose, self.twist = None, None

    def callback(self, msg):
        if self.model_idx is None:
            try:
                self.model_idx = msg.name.index(self.model_name)
            except:
                rospy.logerr('model {} not found in gazebo {}'.format(self.model_name, msg.name))
        if self.model_idx is not None:
            self.pose, self.twist = msg.pose[self.model_idx], msg.twist[self.model_idx]

''' Subscribe to odometry messages and store pose and twist'''
class OdomListener:
    def __init__(self):
        rospy.Subscriber("/rosmip_balance_controller/odom", nav_msgs.msg.Odometry, self.odom_callback)
        self.pose = None

    def odom_callback(self, msg):
        self.stamp = msg.header.stamp
        self.pose, self.twist = msg.pose.pose, msg.twist

        
class TrajStage:
    def __init__(self, **kwargs):
        self.lin = kwargs.get('lin', 0.)
        self.ang = kwargs.get('ang', 0.)
        self.d = kwargs.get('duration', 2.)

    def start(self, t):
        self.t_start = t

    def is_finished(self, t):
        return (t-self.t_start).to_sec() > self.d
        
    def run(self):
        return  self.lin, self.ang

class Line(TrajStage):
    pass
        
class Trajectory:
    def __init__(self, stages):
        self.stages = stages
        self.vel_pub = rospy.Publisher("/rosmip_balance_controller/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)

    def start(self, t):
        self.cur_stage = 0
        self.stages[self.cur_stage].start(t)

        
    def run(self, t):
        if self.stages[self.cur_stage].is_finished(t):
            self.cur_stage += 1
            self.cur_stage = self.cur_stage % len(self.stages)
            self.stages[self.cur_stage].start(t)
        self.publish(*self.stages[self.cur_stage].run())

    def publish(self, lin, ang):
        msg = geometry_msgs.msg.Twist()
        msg.linear = geometry_msgs.msg.Vector3(lin, 0, 0)
        msg.angular = geometry_msgs.msg.Vector3(0, 0, ang)
        self.vel_pub.publish(msg)
        
            
class RosNode:
    def __init__(self):
        rospy.init_node('TestController')
        self.vel_pub = rospy.Publisher("/rosmip_balance_controller/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        self.tl = TruthListener()
        self.ol = OdomListener()
        rospy.Subscriber("/imu", sensor_msgs.msg.Imu, self.imu_callback)
        rospy.Subscriber("/rosmip_balance_controller/debug", rosmip_control.msg.debug, self.debug_callback)
        self.rvels = []
        self.true_pos, self.odom_pos = [], []
        self.true_ori, self.odom_ori = [], []

    def imu_callback(self, msg):
        #print msg.angular_velocity.y
        self.rvels.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        

    def debug_callback(self, msg):
        #print msg
        pass

    def check_odom(self):
        if self.ol.pose is not None and self.tl.pose is not None:
            pos_odom, pos_true = [list_of_position(p) for p in [self.ol.pose.position, self.tl.pose.position]]
            print pos_odom, pos_true
            self.true_pos.append(pos_true); self.odom_pos.append(pos_odom)
            ori_odom, ori_true = [list_of_orientation(o) for o in [self.ol.pose.orientation, self.tl.pose.orientation]]
            self.true_ori.append(ori_true); self.odom_ori.append(ori_odom) 
            
    
    def publish_vel(self, lin, ang):
        msg = geometry_msgs.msg.Twist()
        msg.linear = geometry_msgs.msg.Vector3(lin, 0, 0)
        msg.angular = geometry_msgs.msg.Vector3(0, 0, ang)
        self.vel_pub.publish(msg)

        
    def straight_line(self, len=1.):
        pass
        
        
    def run(self):
        rate = rospy.Rate(25.)
        while not rospy.is_shutdown() and len(self.true_pos) < 1000:
            #self.publish_vel(0., 1.5)
            self.check_odom()
            rate.sleep()
        self.plot_odom()
        plt.show()

    def plot_rvel(self):
        d = np.array(self.rvels)
        plt.plot(d[:,0])
        plt.plot(d[:,1])
        plt.plot(d[:,2])

    def plot_odom(self):
        plt.gcf().canvas.set_window_title("position")
        tp, op = np.array(self.true_pos), np.array(self.odom_pos)
        for i, c in enumerate(["x", "y", "z"]):
            plt.subplot(3,1,i+1)
            plt.plot(tp[:,i])
            plt.plot(op[:,i])
            plt.title(c); plt.legend(["truth", "odom"])
        plt.figure()
        plt.gcf().canvas.set_window_title("orientation")
        to = np.array([euler_of_quat(q) for q in self.true_ori])
        oo = np.array([euler_of_quat(q) for q in self.odom_ori])
        #pdb.set_trace()
        for i, c in enumerate(["roll", "pitch", "yaw"]):
            plt.subplot(3,1,i+1)
            plt.plot(to[:,i])
            plt.plot(oo[:,i])
            plt.title(c); plt.legend(["truth", "odom"])

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    np.set_printoptions(precision=3, linewidth=300)
    try:
        RosNode().run()
    except rospy.ROSInterruptException:
        print 'bye bye'
