#!/usr/bin/env python

'''

'''

import math, time, numpy as np
import rospy
import gazebo_msgs.msg, std_msgs.msg, geometry_msgs, nav_msgs.msg, sensor_msgs.msg, rosmip_control.msg
import tf.transformations

import pdb

def list_of_position(p): return (p.x, p.y, p.z)
def list_of_orientation(q): return (q.x, q.y, q.z, q.w)
def rad_of_deg(d): return d/180.*math.pi
def deg_of_rad(r): return r*180./math.pi
def step(t, a=1., p=1., dt=0.): return a if math.fmod(t+dt, p) > p/2 else -a

s_theta, s_phiL, s_phiR, s_thetad, s_phidL, s_phidR = range(6)
w_l, w_r = range(2)
w_s = ['left', 'right']

''' Subscribe to gazebo/model_states messages and store the robot pose and twist '''
class ThruthListener:
    def __init__(self):
        self.car_model_name, self.car_model_idx = "rosmip", None
        rospy.Subscriber('/gazebo/model_states', gazebo_msgs.msg.ModelStates, self.callback)
        self.pose, self.twist = None, None 
        
    def callback(self, msg):
        if self.car_model_idx is None:
            try:
                self.car_model_idx = msg.name.index(self.car_model_name)
            except:
                rospy.logerr('model {} not found in gazebo {}'.format(self.car_model_name, msg.name))
        if self.car_model_idx is not None:
            self.pose, self.twist = msg.pose[self.car_model_idx], msg.twist[self.car_model_idx]
            self.eulers = tf.transformations.euler_from_quaternion(list_of_orientation(self.pose.orientation))
            #print self.eulers
            #print self.pose, self.twist
            self.vel_w = np.array([self.twist.linear.x, self.twist.linear.y, self.twist.linear.z])
            #self.nvel = np.linalg.norm(self.vel_w)
            #print('velw {:.2f} {:.2f} {:.2f} ({:.2f})'.format(vel_w[0], vel_w[1], vel_w[2], nvel))
            q = list_of_orientation(self.pose.orientation)
            R = tf.transformations.quaternion_matrix(list_of_orientation(self.pose.orientation))
            self.vel_b = np.dot(R[:3,:3].T, self.vel_w)
            #print('q {} velb {:.2f} {:.2f} {:.2f} ({:.2f})'.format(q, vel_b[0], vel_b[1], vel_b[2], nvel))
            #print 'velw {:.2f} {:.2f} {:.2f} velb {:.2f} {:.2f} {:.2f} ({:.2f})'.format(vel_w[0], vel_w[1], vel_w[2], vel_b[0], vel_b[1], vel_b[2], nvel)
            self.vel_x = np.sign(self.vel_b[0])*np.linalg.norm(self.vel_w)
            #self.om = self.twist.angular.z
            #print tf.transformations.quaternion_from_euler(0, 0, 0)
            #pdb.set_trace()
        #print msg

class JointListener:
    def __init__(self):
        rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self.callback)
        self.jp, self.jv, self.je = None, None, None

        
    def callback(self, msg):

        self.jp, self.jv, self.je = np.array(msg.position),  np.array(msg.velocity), np.array(msg.effort)

        #print msg
        #print self.jp, self.jv

class RobotState:
    def __init__(self):
        self.tl = ThruthListener()
        self.jl = JointListener()
        self.X = np.zeros(6)
        self.is_valid = False

    def update(self):
        if self.tl.pose is not None:
            self.X[s_theta]  = self.tl.eulers[1]
            self.X[s_thetad] = self.tl.twist.angular.y
        if self.jl.jp is not None:
            self.X[s_phiL] = self.jl.jp[w_l] +  self.X[s_theta]
            self.X[s_phiR] = self.jl.jp[w_r] + self.X[s_theta]
            self.X[s_phidL] = self.jl.jv[w_l] + self.X[s_thetad]
            self.X[s_phidR] = self.jl.jv[w_r] + self.X[s_thetad]
        if self.tl.pose is not None and self.jl.jp is not None:
            self.is_valid = True

class Balancer:
    
    def __init__(self):
        self.left_pub = rospy.Publisher("/rosmip_left_wheel_controller/command", std_msgs.msg.Float64, queue_size=2)
        self.right_pub = rospy.Publisher("/rosmip_right_wheel_controller/command", std_msgs.msg.Float64, queue_size=2)
        self.debug_pub = rospy.Publisher("/balancer_debug", rosmip_control.msg.debug, queue_size=2)
        self.sp = np.array([0, 0, 0, 0, 0, 0])
        R = 0.045
        k = [R*2.2, -R*-4.5,  R*2.0, -R*-1.0]
        #self.K = np.array([[k[0]/2, k[1]/2,      0, k[2]/2, k[3]/2,      0],
        #                   [k[0]/2,      0, k[1]/2, k[2]/2,      0, k[3]/2]])
        #self.K = np.array([[k[1]/2,      0,      0, k[3]/2,      0,      0],
        #                   [k[1]/2,      0,      0, k[3]/2,      0,      0]])
        #self.K = np.array([[0, k[0]/2,      0, 0, k[2]/2,      0],
        #                   [0,      0, k[0]/2, 0,      0, k[2]/2]])
        self.K = np.array([[-2.5,  0.2,     0,    -0.01,  0.0002,      0],
                           [-2.5,    0,   0.2,    -0.01,       0,      0.002]])
        self.running = True#False

        
    def run(self, robot_state, tau_sat=0.5):
        self.update_status(robot_state)
        if robot_state.is_valid:
            self.run_two_loops(robot_state)
        else:
            self.taus = [0, 0] 
        self.publish_control()
        if robot_state.is_valid:
            self.publish_debug(robot_state)

    def update_status(self, robot_state, tip_lim=rad_of_deg(30)):
        if self.running == False:
            if robot_state.is_valid and abs(robot_state.X[s_theta]) < tip_lim:
                self.running = True
                #self.sp[s_phiL] = robot_state.X[s_phiL]
                #self.sp[s_phiR] = robot_state.X[s_phiR]
                #        print "sp ", self.sp
        else:
            if abs(robot_state.X[s_theta]) > tip_lim:
                self.running = False
            
            
    def run_two_loops(self, robot_state, tau_sat=0.5):

        #self.sp_theta = 0
        self.sp_theta = step(time.time(), rad_of_deg(0.1))
        #self.sp_theta = -0.01*robot_state.tl.vel_x
        print robot_state.tl.vel_x, self.sp_theta
        e, ed = robot_state.X[s_theta] - self.sp_theta, robot_state.X[s_thetad]
        Kp, Kd = 0.75, 0.1
        tau = Kp*e+Kd*ed
        self.taus = [tau, tau]
        self.taus = np.clip(self.taus, -tau_sat, tau_sat)
 
    def run_state_feedback(self, robot_state):
        self.sp = np.zeros(6)
        if 0:
            now, period = time.time(), 4.
            phase = int(math.fmod(now, period)/period*4)
            sps = rad_of_deg(10)*np.array([1., 0., -1., 0.])
            self.sp[s_phiL] = sps[phase]
            self.sp[s_phiR] = sps[phase]
                
        if self.running:
            dX_clip = np.array([1, 1, 1, 1, rad_of_deg(200), rad_of_deg(200)])
            dX = robot_state.X - self.sp
            dX = np.clip(dX, -dX_clip, dX_clip)
            #print "dX ", dX
            
            self.taus = -np.dot(self.K, dX)
            self.taus = np.clip(taus, -tau_sat, tau_sat)
        else:
            self.taus = [0, 0]
        #taus = [0., sps[phase]]

    
    def publish_control(self):
        msg =  std_msgs.msg.Float64()
        msg.data = self.taus[0]
        self.left_pub.publish(msg)
        msg.data = self.taus[1]
        self.right_pub.publish(msg)
            
    def publish_debug(self, robot_state):
        m = rosmip_control.msg.debug()
        m.theta = deg_of_rad(robot_state.X[s_theta])
        m.thetad = deg_of_rad(robot_state.X[s_thetad])
        m.phiL =  deg_of_rad(robot_state.X[s_phiL])
        m.phiR =  deg_of_rad(robot_state.X[s_phiR])
        m.phidL =  deg_of_rad(robot_state.X[s_phidL])
        m.phidR =  deg_of_rad(robot_state.X[s_phidR])
        m.phiL_sp = deg_of_rad(self.sp[s_phiL])
        m.phiR_sp = deg_of_rad(self.sp[s_phiR])
        m.tauL = self.taus[0]
        m.tauR = self.taus[1]
        self.debug_pub.publish(m)
        
        

        
        
class Node:

    def run(self, freq=200):
        rospy.init_node('rosmip_sim_controller', anonymous=True)
        self.robot_state = RobotState()
        self.balancer = Balancer()
        rate = rospy.Rate(freq)
        while not rospy.is_shutdown():
            #if self.truth.pose is not None:
            self.robot_state.update()
            self.balancer.run(self.robot_state)
            rate.sleep()


    

if __name__ == '__main__':
    np.set_printoptions(precision=4, linewidth=300, suppress=True)
    try:
        Node().run()
    except rospy.ROSInterruptException:
        pass
