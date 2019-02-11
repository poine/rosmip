#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np, matplotlib.pyplot as plt
import scipy.signal, scipy.optimize, control
import pdb
#import keras

import homere_control.io_dataset as iod
import homere_control.utils as hcu
import rosmip_control.sim_planar_simple as sim
import cpp_rosmip_control
import planat_control

class Ctl:
    def __init__(self, time):
        self.cpp_law = cpp_rosmip_control.LegacyCtlLaw()
        # original
        self.cpp_law.set_inner_loop_coeffs(1.05, [-4.945, 8.862, -3.967], [1.000, -1.481, 0.4812])
        self.wheel_r = 0.03 # FIXME, put that in cpp
        self.sp_lvel = self.sp_rvel = np.zeros((len(time), 1))
    
    def get(self, X, k):
        _x, _theta, s_xd, _theta = X
        _lwa = _rwa = 1./self.wheel_r*_x
        pwm_l, pwm_r = self.cpp_law.update(_theta, _lwa, _rwa, self.sp_lvel[k-1], self.sp_rvel[k-1] )
        Uk = (pwm_l + pwm_r)/4
        return Uk


    def synth(self, plant, dt=0.01):
        lqr = planar_control.CtlPlaceFullLQR(plant, dt)
        B, C, D = [[0], [1], [0], [0]] , [[0, 1, 0, 0]], [[0]]
        #H = hcu.get_precommand(lqr.A, lqr.B, C, lqr.K)
        #pdb.set_trace()
        cl_ss = control.ss(lqr.Acl, B, C, D)
        dt_ss = control.sample_system(cl_ss, dt, method='zoh') #  ‘matched’, ‘tustin’, ‘zoh’
        print(dt_ss)
        dt_tf = control.tf(dt_ss)
        print('desired closed loop tf')
        print(dt_tf)
        
        def err_fun(p):
            ctl_num, ctl_den = p[:3], [1, p[3], p[4]]
            ctl_tf = control.tf(ctl_num, ctl_den, dt)
            cl_tf = control.feedback(dt_tf, ctl_tf, sign=-1)
            pdb.set_trace()

        p0 = [-4.945, 8.862, -3.967, -1.481, 0.4812]
        err_fun(p0)
        
        

    
    def synth2(self, plant, dt=0.01):
        Ac, Bc = hcu.num_jacobian([0, 0, 0, 0], [0], plant.dyn_cont)
        print('Ac\n{}\nBc\n{}'.format(Ac, Bc))
        ct_ss = control.ss(Ac, Bc, [[1, 0, 0, 0]], [[0]])
        print(ct_ss)
        dt_ss = control.sample_system(ct_ss, dt, method='zoh') #  ‘matched’, ‘tustin’, ‘zoh’
        print(dt_ss)
        dt_tf = control.tf(dt_ss)
        print(dt_tf)
        
        desired_cl_poles_c = np.array([-18.36312687+0.j, -7.08908759+0.j, -6.64111168+1.52140048j,  -6.64111168-1.52140048j])
        desired_cl_poles_d =  np.exp(desired_cl_poles_c*dt)
        print('desired poles\n{}\n{}'.format(desired_cl_poles_c, desired_cl_poles_d))


        def compute_cl_poles(ctl_num, ctl_den):
            ctl_tf = control.tf(ctl_num, ctl_den, dt)
            #print(ctl_tf)
            cl_tf = control.feedback(dt_tf, ctl_tf, sign=-1)
            #print control.damp(cl_tf)
            #print(cl_tf)
            cl_poles_d = control.pole(cl_tf)
            #print(cl_polesd)
            cl_poles_c = np.sort_complex(np.log(cl_poles_d)/dt)
            print('cl_poles_c\n{}'.format(cl_poles_c))
            return cl_poles_d

        compute_cl_poles([-4.945, 8.862, -3.967], [1.000, -1.481, 0.4812])
        compute_cl_poles([-4.945, 8.862, -3.967], [1.000, -1.2, 0.4812])

        def err_fun(p):
            ctl_num, ctl_den = p[:3], [1, p[3], p[4]]
            err = np.sum(np.abs(compute_cl_poles(ctl_num, ctl_den)[:4] - desired_cl_poles_d))
            print ctl_num, ctl_den, err
            return err

        p0 = [-4.945, 8.862, -3.967, -1.481, 0.4812]
        res = scipy.optimize.minimize(err_fun, p0)
        pdb.set_trace()
        
        #print(self.tf_disc.num)
        #print(self.tf_disc.den)
        #self.a1, self.a0 = self.tf_disc.num[0][0]
        #self.b2, self.b1, self.b0 = self.tf_disc.den[0][0]

        
    
def main():
    np.set_printoptions(precision=2, linewidth=600)

    dt = 1./100; time =  np.arange(0., 0.2, dt)
    plant = sim.Plant()
    X0 = [0.1, 0.01, 0, 0]
    ctl = Ctl(time)
    ctl.synth(plant)
    X, U = plant.sim_with_input_fun(time, ctl, X0)
    sim.plot_all(time, X, U)
    #plt.subplot(5,1,1); plt.plot(time, ctl.x_sp)
    plt.show()

if __name__ == "__main__":
    #logging.basicConfig(level=logging.INFO)
    main()
