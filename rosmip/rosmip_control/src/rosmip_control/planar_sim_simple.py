# -*- coding: utf-8 -*-
import logging, timeit, math, numpy as np, scipy.integrate, matplotlib.pyplot as plt, pickle
import control

import utils
import homere_control.utils as hcu
'''
   Planar mobile inverted pendulum (without motor dynamics)
'''


class PlantParam:
    
    def __init__(self, sat=None):
        self.R, self.L       = 0.04, 0.1                 # radius of wheel and dist from wheel axis to body center of mass in m
        self.mw, self.mb     = 0.02, 0.2                 # mass of the wheel and body in kg
        self.Iw, self.Ib     = 0.00025, 0.003            # inertia of wheel and body 
        self.tsat = float('inf') if sat is None else sat # max torque of the motor
        self.g = 9.81                                    # gravity
        self.compute_aux()

    def compute_aux(self):
        self.b = self.Ib + self.mb*self.L**2
        self.c = self.Iw + (self.mw+self.mb)*self.R**2
        self.bc = self.b*self.c
        self.h = self.mb*self.R*self.L



class Plant:
    # Components of the state vector
    s_x      = 0  # horizontal pos of the wheel axis (m)
    s_theta  = 1  # orientation of the body in rad, 0 up (rad)
    s_xd     = 2  # horizontal velocity of the wheel axis (m/s)
    s_thetad = 3  # rotational velocity of the body in (rad/s)
    s_size   = 4  # dimension of the state vector
    # Components of the input vector
    i_t    = 0  # torque on the wheel axis (N.m)
    i_size = 1  # dimension of the input vector

    def __init__(self, P=None):
        self.P = P if P is not None else PlantParam()
        
    def dyn_cont(self, X, t, U):
        ''' 
        Continuous time dynamics:
        Xdot = f(X, U)
        '''
        Xd = np.zeros(self.s_size)
        th, thd = X[self.s_theta], X[self.s_thetad]
        sth, cth = np.sin(th), np.cos(th)

        # saturate input
        tau = U[self.i_t]#np.clip(U[self.i_t], -self.P.tsat, self.P.tsat) #pb with casadi
        # upper rows are found directly in state vector
        Xd[self.s_x]  = X[self.s_xd]
        Xd[self.s_theta] = thd
        # compute the lower rows
        a = self.P.h*cth
        i = 1./(a**2-self.P.bc)
        d, e = self.P.mb*self.P.g*self.P.L*sth - tau, self.P.mb*self.P.R*self.P.L*thd*thd*sth + tau

        Xd[self.s_xd]  = -self.P.R*i*(a*d - self.P.b*e)
        Xd[self.s_thetad] = i*(-self.P.c*d + a*e)

        return Xd

    def dyn_disc(self, Xkm1, t, dt, Ukm1):
        _unused, Xk = scipy.integrate.odeint(self.dyn_cont, Xkm1, [t, t+dt], args=(Ukm1, ))
        return Xk


    
    def jac(self, Xe, Ue):
        return hcu.num_jacobian(Xe, Ue, self.dyn_cont)

    def disc_jac(self, Xe, Ue, dt):
        Ac, Bc = self.jac(Xe, Ue)
        ct_ss = control.ss(Ac, Bc, [[1, 0, 0, 0]], [[0]])
        dt_ss = control.sample_system(ct_ss, dt, method='zoh') #  ‘matched’, ‘tustin’, ‘zoh’
        return dt_ss.A, dt_ss.B
    
    def sim_with_input_vec(self, time, U, X0):
        X =  np.zeros((len(time), self.s_size))
        X[0] = X0
        for i in range(1, len(time)):
            X[i] = self.dyn_disc(X[i-1], time[i-1], time[i]-time[i-1], [U[i-1]])
        return X

    def sim_with_input_fun(self, time, ctl, X0):
        X = np.zeros((len(time), self.s_size))
        U = np.zeros((len(time), self.i_size))
        X[0] = X0
        for i in range(1, len(time)):
            U[i-1] = ctl.get(X[i-1], i-1)
            X[i] = self.dyn_disc(X[i-1], time[i-1], time[i]-time[i-1], [U[i-1]])
        U[-1] = U[-2]
        return X, U



def plot_short(time, X, U, figure=None, window_title="trajectory"):
    margins=(0.06, 0.05, 0.98, 0.96, 0.20, 0.34)
    figure = utils.prepare_fig(figure, window_title, figsize=(0.75*20.48, 0.75*10.24), margins=margins)
    plots = [("x", "m", X[:,Plant.s_x]),
             ("$\\theta$", "deg", utils.deg_of_rad(X[:,Plant.s_theta])),
             ("$\\tau$", "N.m", U[:,Plant.i_t])]
    for i, (title, ylab, data) in enumerate(plots):
        ax = plt.subplot(3, 1, i+1)
        plt.plot(time, data, linewidth=2)
        utils.decorate(ax, title=title, ylab=ylab)
    return figure

def plot_all(time, X, U, Yc=None, figure=None, window_title="trajectory"):
    margins=(0.04, 0.05, 0.98, 0.96, 0.20, 0.34)
    figure = utils.prepare_fig(figure, window_title, figsize=(0.75*20.48, 0.75*10.24), margins=margins)
    plots = [("x", "m", X[:,Plant.s_x]),
             ("xd", "m/s", X[:,Plant.s_xd]),
             ("$\\theta$", "deg", utils.deg_of_rad(X[:,Plant.s_theta])),
             ("$\\theta d$", "deg/s", utils.deg_of_rad(X[:,Plant.s_thetad])),
             ("$\\tau$", "N.m", U[:,Plant.i_t])]
    for i, (title, ylab, data) in enumerate(plots):
        ax = plt.subplot(len(plots), 1, i+1)
        plt.plot(time, data, linewidth=2)
        utils.decorate(ax, title=title, ylab=ylab)
    if Yc is not None:
        plt.subplot(len(plots), 1, 1)
        plt.plot(time, Yc, linewidth=2)
    return figure
