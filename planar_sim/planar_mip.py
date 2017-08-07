#!/usr/bin/env python
#-*- coding: utf-8 -*-

'''
  Dynamic model of a mobile inverted pendulum
'''

import numpy as np, math, scipy.integrate, matplotlib.pyplot as plt
import control.matlab
import my_utils as mu
#
# Parameters
#
class Param:
    def __init__(self, sat=None):
        self.R, self.L       = 0.04, 0.1      # radius of wheel and dist from wheel axis to body center of mass in m
        self.mw, self.mb     = 0.02, 0.2      # mass of the wheel and body in kg
        self.Iw, self.Ib     = 0.00025, 0.003 # inertia of wheel and body 
        self.tsat = float('inf') if sat is None else sat # max torque of the motor
        self.g = 9.81
        self.compute_aux()
        
    def compute_aux(self):
        self.b = self.Ib + self.mb*self.L**2
        self.c = self.Iw + (self.mw+self.mb)*self.R**2
        self.bc = self.b*self.c
        self.h = self.mb*self.R*self.L

#
# Components of the state vector
#
s_x      = 0  # horizontal pos of the wheel axis in m
s_theta  = 1  # orientation of the body in rad, 0 up
s_xd     = 2  # horizontal velocity of the wheel axis in m/s
s_thetad = 3  # rotational velocity of the body in rad/s
s_size   = 4  # dimension of the state vector

#
# Components of the input vector
#
iv_t    = 0  # torque on the wheel axis in N.m
iv_size = 1  # dimension of the input vector

# Dynamic model as state space representation
#
# X : state
# U : input
# P : param
#
# returns Xd, time derivative of state
#
def dyn(X, t, U, P):

    Xd = np.zeros(s_size)
    th, thd = X[s_theta], X[s_thetad]
    sth, cth = math.sin(th), math.cos(th)

    # saturate input
    tau = np.clip(U[iv_t], -P.tsat, P.tsat)
    # upper rows are found directly in state vector
    Xd[s_x]  = X[s_xd]
    Xd[s_theta] = thd
    # compute the lower rows
    a = P.h*cth
    i = 1./(a**2-P.bc)
    d, e = P.mb*P.g*P.L*sth - tau, P.mb*P.R*P.L*thd**2*sth + tau
    
    Xd[s_xd]  = -P.R*i*(a*d - P.b*e)
    Xd[s_thetad] = i*(-P.c*d + a*e)

    return Xd

#
# Numerical Jacobians of the Dynamic Model
#
def num_jacobian(X, U, P):
    dX = np.diag([0.01, 0.01, 0.01, 0.01])
    A = np.zeros((s_size, s_size))
    for i in range(0, s_size):
        dx = dX[i,:]
        delta_f = dyn(X+dx/2, 0, U, P) - dyn(X-dx/2, 0, U, P)
        delta_f = delta_f / dx[i]
        A[:,i] = delta_f

    dU = np.diag([0.01])
    delta_f = dyn(X, 0, U+dU/2, P) - dyn(X, 0, U-dU/2, P)
    delta_f = delta_f / dU
    B = np.zeros((s_size,iv_size))
    B[:,0] = delta_f
    return A,B

#
# a plot function
#
def plot(time, X, U, Y=None, figure=None, window_title="trajectory"):
    margins=(0.04, 0.05, 0.98, 0.96, 0.20, 0.34)
    figure = mu.prepare_fig(figure, window_title, figsize=(0.75*20.48, 0.75*10.24), margins=margins)
    plots = [("x", "m", X[:,s_x]),
             ("$\\theta$", "deg", mu.deg_of_rad(X[:,s_theta])),
             ("$\\tau$", "N.m", U[:,iv_t])]
    for i, (title, ylab, data) in enumerate(plots):
        ax = plt.subplot(3, 1, i+1)
        plt.plot(time, data, linewidth=2)
        mu.decorate(ax, title=title, ylab=ylab)
    if Y != None:
        plt.subplot(3,1,1)
        plt.plot(time, Y, 'r')

    return figure



def sim_open_loop(X0):
    P, U = Param(), [0]
    time = np.arange(0., 10, 0.01)
    X = scipy.integrate.odeint(dyn, X0, time, args=(U, P ))
    U = np.zeros((len(time), 1))
    return time, X, U

def sim_state_feedback(X0, K):
    P = Param()
    def cl_dyn(X, t): return dyn(X, t, [-np.dot(K, X)], P)
    time = np.arange(0., 5, 0.01)
    X = scipy.integrate.odeint(cl_dyn, X0, time)
    U =  np.array([[-np.dot(K, Xi)] for Xi in X])
    return time, X, U

def sim_place(X0, poles):
    P = Param()
    A, B = num_jacobian([0, 0, 0, 0], [0], P)
    print A, B, np.linalg.eig(A), control.matlab.ctrb(A, B)
    K = control.matlab.place(A, B, poles)
    print  np.linalg.eig(A-np.dot(B, K))[0]
    time, X, U = sim_state_feedback(X0, K)
    plot(time, X, U, window_title="place")

def sim_lqr(X0, Q, R, Yc):
    P = Param()
    A, B = num_jacobian([0, 0, 0, 0], [0], P)
    (K, X, E) = control.matlab.lqr(A, B, Q, R)
    print K
    print  np.linalg.eig(A-np.dot(B, K))[0]
    time, X, U = sim_state_feedback(X0, K)
    plot(time, X, U, window_title="LQR")

def step(t, a=.1, p=4., dt=0.): return a if math.fmod(t+dt, p) > p/2 else -a

def sim_two_loops_ctl(X0=[0, 0, 0, 0]):
    P = Param()
    def ctl(t, X):
        sp_x = 0.1
        e_x, e_xd = X[s_x]-sp_x,  X[s_xd]
        Kp, Kd = 2, 4
        sp_theta = Kp*e_x + Kd*e_xd
        e_theta, e_thetad = X[s_theta]-sp_theta,  X[s_thetad]
        Kp, Kd = 6, 6
        tau = Kp*e_theta + Kd*e_thetad
        return [tau]
    def cl_dyn(X, t): return dyn(X, t, ctl(t, X), P)
    time = np.arange(0., 5, 0.01)
    X = scipy.integrate.odeint(cl_dyn, X0, time)
    U =  np.array([ctl(ti, Xi) for ti, Xi in zip(time, X)])
    return time, X, U

    
if __name__ == "__main__":
    #time, X, U = sim_open_loop([0, 0.01, 0, 0])
    #plot(time, X, U)

    if 0:
        poles = [-2, -2, -2, -2]
        sim_place([0, 0.01, 0, 0], poles)
    if 0:
        Q, R = np.diag([10, 1, 1, 1]), np.diag([2])
        sim_lqr([0.1, 0.01, 0, 0], Q, R, step)
    if 1:
        time, X, U =  sim_two_loops_ctl([0, 0.1, 0, 0])
        plot(time, X, U, window_title="two loops")
    
    plt.show()
