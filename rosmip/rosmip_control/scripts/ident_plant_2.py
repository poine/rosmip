#!/usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import tf.transformations
import keras, control

import pdb

import julie_misc.plot_utils as jpu
import homere_control.io_dataset as iod
import homere_control.utils as hcu

import ident_plant as idp
'''

  Second attempt at identifying RosMip in Gazebo
  using pwm_sum and pwm_diff 

'''


class ANN(idp.ANN):
        # input Xk
        #   phi_k, gamma_k, phi_dot_k, gamma_dot_k, theta_k, theta_dot_k, pwm_sum_k, pwm_diff_k
        # output:
        #   phi_kp1, gamma_kp1, phi_dot_kp1, gamma_dot_kp1, theta_kp1, theta_dot_kp1

    def make_io(self, ds):
        _input, _output, _stamp = idp.ANN.make_io(self, ds)
        #pdb.set_trace()
        pwm_l, pwm_r = _input[:,6], _input[:,7]
        pwm_sum, pwm_diff = pwm_l + pwm_r, pwm_r - pwm_l
        _input[:,6], _input[:,7] = pwm_sum, pwm_diff
        return _input, _output, _stamp

if __name__ == '__main__':
    keras.backend.set_floatx('float64')
    np.set_printoptions(precision=2, linewidth=300)
    #filename, _type = '/mnt/mint18/home/poine/work/homere/homere_control/data/rosmip/gazebo/rosmip_io_07_random_2.npz', 'rosmip'
    filename, _type = '/mnt/mint18/home/poine/work/homere/homere_control/data/rosmip/gazebo/rosmip_2_io_08_random_2.npz', 'rosmip'
    ds = iod.DataSet(filename, _type)
    #plot_dataset(ds)
    #plt.show()
    ann = ANN()
    _input, _output, _time =  ann.make_io(ds)
    ann.train(_input, _output, _time)
    ann.plot_training_history()
    ann.report()
    plt.show()
