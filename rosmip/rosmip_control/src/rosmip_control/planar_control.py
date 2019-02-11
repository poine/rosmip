#! /usr/bin/env python
# -*- coding: utf-8 -*-

import logging, timeit, math, numpy as np, matplotlib.pyplot as plt
import control, control.matlab

#import mip_simple as ms#, utils as ut
import rosmip_control.planar_sim_simple as ms
import homere_control.utils as hcu
import pdb

'''
   rewrite of the roboticscape controller
'''

def debug_filter(f):
    print('step: {}'.format(f.step))
    print('input: {}'.format(f.in_buf))
    print('output: {}'.format(f.out_buf))
    

def test_filter(dt=1./50):
    if 0:
        f = DiscTimeFilter([-4.945, 8.862, -3.967], [1.000, -1.481, 0.4812], 1.)
        debug_filter(f)
        f.march(1)
        debug_filter(f)
        f.march(2)
        debug_filter(f)
        f.march(3)
        debug_filter(f)
        pdb.set_trace()
    else:
        fl = FirstOrderLowPass(dt, 0.3)
        fh = FirstOrderHighPass(dt, 0.3)
        #f = Integrator(dt)
        #f = PID(1., 0.3, 0.05, 4*dt, dt)
        time = np.arange(0, 5, dt)
        iv = hcu.step_input_vec(time)
        ovl = np.array([fl.march(i) for i in iv])
        ovh = np.array([fh.march(i) for i in iv])
        plt.plot(time, iv)
        plt.plot(time, ovl)
        plt.plot(time, ovh)
        plt.plot(time, ovl+ovh)
        plt.show()



class RingBuffer:
    def __init__(self, _n):
        self.b = np.zeros(_n)
        self.i = 0

    def put(self, _v):
        self.i = (self.i+1)%len(self.b)
        self.b[self.i] = _v

    def get(self, i): return self.b[self.i - i]

    def __str__(self): return '{} ({})'.format(self.b, self.i)
    
    
class DiscTimeFilter:

    def __init__(self, num, den, gain):
        self.num, self.den, self.gain = num, den, gain
        self.order, self.rel_deg = len(self.den)-1, len(self.den)-len(self.num)
        self.in_buf, self.out_buf = RingBuffer(len(self.den)), RingBuffer(len(self.den))
        self.step = 0

    def march(self, new_input):
        self.in_buf.put(new_input)
        new_output = 0.
        for i in range(len(self.num)):
            new_output += self.gain*self.num[i]*self.in_buf.get(i+self.rel_deg)
        for i in range(self.order):
             new_output -= self.den[i+1]*self.out_buf.get(i)
        new_output /= self.den[0]
        self.out_buf.put(new_output)
        self.step += 1
        return new_output

    def __str__(self):
        return '{}\n--------------\n{}'.format(self.num, self.den)


class FirstOrderLowPass(DiscTimeFilter):
    def __init__(self, dt, tau):
        lp_const = dt/tau
        num, den = [lp_const, 0], [1., lp_const-1.]
        DiscTimeFilter.__init__(self, num, den, 1.)

class FirstOrderHighPass(DiscTimeFilter):
    def __init__(self, dt, tau):
        hp_const = dt/tau
        num, den = [1.-hp_const, hp_const-1.], [1., hp_const-1.]
        DiscTimeFilter.__init__(self, num, den, 1.)

class Integrator(DiscTimeFilter):
    def __init__(self, dt):
        num, den = [dt], [1., -1.]
        DiscTimeFilter.__init__(self, num, den, 1.)
        
class PID(DiscTimeFilter):
    def __init__(self, kp, ki, kd, Tf, dt):
        if ki == 0.:
            num = [(kp*Tf+kd)/Tfm, -(((ki*dt-kp)*(dt-Tf))+kd)/Tf]
            den = [1.,  -(Tf-dt)/Tf]
        else:
            num = [(kp*Tf+kd)/Tf, (ki*dt*Tf + kp*(dt-Tf) - kp*Tf - 2.0*kd)/Tf, (((ki*dt-kp)*(dt-Tf))+kd)/Tf]
            den = [1., (dt-(2.0*Tf))/Tf, (Tf-dt)/Tf]
        DiscTimeFilter.__init__(self, num, den, 1.)
            
class CtlLegacy:
    def __init__(self, plant, dt):
        self.plant, self.dt = plant, dt
        Ac, Bc = hcu.num_jacobian([0, 0, 0, 0], [0], plant.dyn_cont)
        #print Ac
        #print Bc
        Cc, Dc = np.array([[0, 1, 0, 0]]), np.array([[0]]) # regulate theta
        self.sysc = control.ss(Ac, Bc, Cc, Dc)
        self.sysd = control.sample_system(self.sysc, dt)

        plant_tfd = control.tf(self.sysd)
        ctl_tfd = control.tf([-4.945, 8.862, -3.967], [1.000, -1.481, 0.4812], dt)

        gt = control.feedback(ctl_tfd * plant_tfd, 1)
        cl_polesd = gt.pole()
        cl_polesc = np.log(cl_polesd)/dt
        #y, t = control.step(gt, np.arange(0, 5, dt))
        #plt.plot(t, y[0])
        #plt.show()
        #self.il_ctl = self.synth_pid(0.5, 0.01, 0.05)
        #pdb.set_trace()
        
        if 1:
            #self.il_ctl = DiscTimeFilter([-4.945, 8.862, -3.967], [1.000, -1.481, 0.4812], 1.05)
            self.il_ctl = DiscTimeFilter([-4.945, 8.862, -3.967], [1.000, -1.481, 0.4812], 1.25)
        else:
            self.il_ctl = self.synth_pid(0.4, 0.04, 0.1)
            self.il_ctl.num = [-c for c in self.il_ctl.num]
            self.il_ctl.gain = 1.
        
        #self.ol_ctl = DiscTimeFilter([0.18856,  -0.37209,  0.18354], [1.00000,  -1.86046,   0.86046], 0.9)
        self.ol_ctl = DiscTimeFilter([0.18856,  -0.37209,  0.18354], [1.00000,  -1.86046,   0.86046], 0.4)
    
    def synth_pid(self, kp, ki, kd):
        flt = PID(kp, ki, kd, 2.*self.dt, self.dt)
        ctl_tfd = control.tf(flt.num, flt.den, self.dt)
        plant_tfd = control.tf(self.sysd)
        cl_tfd = control.feedback(-ctl_tfd * plant_tfd, 1)
        cl_polesd = cl_tfd.pole()
        cl_polesc = np.log(cl_polesd)/self.dt
        print flt
        print cl_polesc
        #y, t = control.step(cl_tfd, np.arange(0, 5, self.dt))
        #plt.plot(t, y[0])
        #plt.show()
        #pdb.set_trace()
        return flt

    def get(self, X, i, theta_sat=np.deg2rad(30)):
        phi = -X[ms.Plant.s_x]/self.plant.P.R
        phi_err = phi
        theta_sp = self.ol_ctl.march(-phi_err)
        self.theta_sp[i] = np.clip(theta_sp, -theta_sat, theta_sat)
        theta_err = X[ms.Plant.s_theta] - self.theta_sp[i]
        #self.theta_sp[i] = theta_sp
        u = self.il_ctl.march(-theta_err)

        return u

    def get_inner_loop(self, X, i):
        theta_err = X[ms.Plant.s_theta] - self.theta_sp[i]
        u = self.il_ctl.march(-theta_err)


class CtlPlaceTwoLoops:
    def __init__(self, plant, dt):
        A, B = hcu.num_jacobian([0, 0, 0, 0], [0], plant.dyn_cont)
        At = np.array([[A[plant.s_theta, plant.s_theta], A[plant.s_theta, plant.s_thetad]],
                       [A[plant.s_thetad, plant.s_theta], A[plant.s_thetad, plant.s_thetad]]])
        Bt = np.array([B[plant.s_theta], B[plant.s_thetad]])
        # extract non zero cefficients
        self.a1 = A[plant.s_xd, plant.s_theta]
        self.a2 = A[plant.s_thetad, plant.s_theta]
        self.b1 = B[plant.s_xd,0]
        self.b2 = B[plant.s_thetad,0]
        
        om_il, xi_il = 45, 0.7
        il_poles = hcu.get_lambdas(om_il, xi_il)
        if 0:
            self.Kil = control.matlab.place(At, Bt, il_poles) # fucking place is broken
        else: # let's do it by hand instead
            self.Ktheta, self.Kthetad = (om_il**2+self.a2)/self.b2, 2*xi_il*om_il/self.b2
            self.Htheta = (om_il**2)/self.b2
            self.Kil = np.array([[self.Ktheta, self.Kthetad]])
        Acl = At-np.dot(Bt, self.Kil)
        eva, eve = np.linalg.eig(Acl)
        print('inner loop wanted:{} obtained{}'.format(il_poles, eva))

        om_ol, xi_ol = 1., 0.9
        ol_poles = hcu.get_lambdas(om_ol, xi_ol)
        if 0:
            c = self.a1 - self.b1*self.Ktheta + self.b1*self.Htheta
            self.Kx, self.Kxd =  (om_ol**2)/c, 2*xi_ol*om_ol/c
            self.Hx = (om_ol**2)/c
        if 0:
            self.Kx, self.Kxd =  (om_ol**2)/self.b1, 2*xi_ol*om_ol/self.b1
            self.Kolt = self.a1/self.b1-self.Ktheta
            self.Koltd = -self.Kthetad
            #self.Kol = np.array([[self.Kx, self.Kxd]])
        if 1:
            self.Kx, self.Kxd = -0.75, -0.05
            self.Hx = self.Kx
        pdb.set_trace()

    def get_inner(self, X, i):
        #Xt = np.array([X[ms.Plant.s_theta], X[ms.Plant.s_thetad]])
        #dXt = Xt - [self.theta_sp[i], 0]
        #return -np.dot(self.Kil, dXt)
        tau = -self.Ktheta*X[ms.Plant.s_theta] -self.Kthetad*X[ms.Plant.s_thetad] + self.Htheta*self.theta_sp[i]
        return tau
        
    def get(self, X, i, theta_sat=np.deg2rad(30)):
        #Xx = np.array([X[ms.Plant.s_x], X[ms.Plant.s_xd]])
        #dXx = Xx - [self.x_sp[i], 0]
        #self.theta_sp[i] = -np.dot(self.Kol, dXx)

        theta_sp = -self.Kx*X[ms.Plant.s_x]-self.Kxd*X[ms.Plant.s_xd]+self.Hx*self.x_sp[i]
        #theta_sp = (-self.Kx*X[ms.Plant.s_x]-self.Kxd*(X[ms.Plant.s_xd]-self.x_sp[i]) - self.Kolt*X[ms.Plant.s_theta] -  self.Koltd*X[ms.Plant.s_thetad])/self.Htheta
        
        
        self.theta_sp[i] = np.clip(theta_sp, -theta_sat, theta_sat)
        return self.get_inner( X, i)

class CtlPlaceTwoLoops_inner(CtlPlaceTwoLoops):
    def get(self, X, i):
        return self.get_inner( X, i)
    

class CtlPlaceTwoLoops2:
    ''' let's try a full placement, then separate... '''

    def __init__(self, plant, dt):
        A, B = hcu.num_jacobian([0, 0, 0, 0], [0], plant.dyn_cont)
        #poles = [-12+12j, -12-12j, -6.5+0j, -6.5-0j]
        poles = [-18.36312687+0.j, -7.08908759+0.j, -6.64111168+1.52140048j, -6.64111168-1.52140048j]
        pdb.set_trace()
        self.K = control.matlab.place(A, B, poles)
        self.Ktheta = self.K[0, ms.Plant.s_theta]
        self.Kthetad = self.K[0, ms.Plant.s_thetad]
        self.Htheta = self.Ktheta
        self.Kx = self.K[0, ms.Plant.s_x]/self.Htheta
        self.Kxd = self.K[0, ms.Plant.s_xd]/self.Htheta
        self.Hx = self.Kx
         
    def get(self, X, i, theta_sat=np.deg2rad(30)):
        theta_sp = -self.Kx*X[ms.Plant.s_x]-self.Kxd*X[ms.Plant.s_xd]+self.Hx*self.x_sp[i]
        self.theta_sp[i] = theta_sp#np.clip(theta_sp, -theta_sat, theta_sat)
        tau = -self.Ktheta*X[ms.Plant.s_theta] -self.Kthetad*X[ms.Plant.s_thetad] + self.Htheta*self.theta_sp[i]
        return tau 
        
        
    
class CtlPlaceFull:
    def __init__(self, plant, dt):
        A, B = hcu.num_jacobian([0, 0, 0, 0], [0], plant.dyn_cont)
        poles = [-12+12j, -12-12j, -6.5+0j, -6.5-0j]
        self.K = control.matlab.place(A, B, poles)
        print('K {}'.format(self.K))
        print('cl poles {}'.format(np.linalg.eig(A-np.dot(B, self.K))[0]))

    def get(self, X, i):
        dX = X - [self.x_sp[i], 0, 0, 0]
        return -np.dot(self.K, dX)

class CtlPlaceFullLQR:
    def __init__(self, plant, dt):
        self.A, self.B = hcu.num_jacobian([0, 0, 0, 0], [0], plant.dyn_cont)
        Q, R = np.diag([20, 0.2, 0.05, 0.005]), np.diag([6])
        (self.K, X, E) = control.matlab.lqr(self.A, self.B, Q, R)
        print('K {}'.format(self.K))
        self.Acl = self.A-np.dot(self.B, self.K)
        print('cl poles {}'.format(np.linalg.eig(self.Acl)[0]))
        
    def get(self, X, i):
        dX = X - [self.x_sp[i], 0, 0, 0]
        return -np.dot(self.K, dX)




    
def test_inner_loop(time, dt, plant, ctl):
    ctl.theta_sp = hcu.step_input_vec(time, a0=-0.01, a1=0.01, dt=2., t0=.5)
    X0 = [0.1, 0.01, 0, 0]
    X, U = plant.sim_with_input_fun(time, ctl, X0)
    ms.plot_all(time, X, U)
    plt.subplot(5,1,3); plt.plot(time, np.rad2deg(np.array(ctl.theta_sp)))


def test_outer_loop(time, dt, plant, ctl):
    ctl.x_sp = hcu.step_input_vec(time, a0=-0.1, a1=0.1, dt=4., t0=1.)
    ctl.theta_sp = np.zeros(len(time)) 
    X0 = [0.1, 0.01, 0, 0]
    X, U = plant.sim_with_input_fun(time, ctl, X0)
    ms.plot_all(time, X, U)
    plt.subplot(5,1,1); plt.plot(time, ctl.x_sp)
    plt.subplot(5,1,3); plt.plot(time, np.rad2deg(ctl.theta_sp))
    
    
def main():
    dt = 1./100; time =  np.arange(0., 4.55, dt)
    plant = ms.Plant()
    #ctl = CtlLegacy(plant, dt)
    #ctl = CtlPlaceTwoLoops_inner(plant, dt)
    #test_inner_loop(time, dt, plant, ctl)
    #ctl = CtlPlaceFull(plant, dt)
    #ctl = CtlPlaceFullLQR(plant, dt)
    #ctl = CtlPlaceTwoLoops(plant, dt)
    ctl = CtlPlaceTwoLoops2(plant, dt)
    test_outer_loop(time, dt, plant, ctl)
    plt.show()
    
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    #test_filter()
    main()
