#!/usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import keras, control

import pdb

import julie_misc.plot_utils as jpu
import homere_control.io_dataset as iod
import homere_control.utils as hcu
import rosmip_control.planar_sim_simple as sim
import rosmip_control.planar_control as ctl


class SimulatedPlanarDataset(iod.DataSet):
    def __init__(self, _sim, dt=0.01, _nb=int(1e4), _type='random'):
        _ctl =  ctl.CtlPlaceFullLQR(_sim, dt)
        print('setpoint: {}'.format(_type))
        if _type == 'step':
            time =  np.arange(0., dt*_nb, dt)
            _ctl.x_sp = hcu.step_input_vec(time, a0=-0.1, a1=0.1, dt=4., t0=1.)
        else:
            time, _ctl.x_sp = hcu.make_random_pulses(dt, _nb, min_intensity=-0.1, max_intensity=0.1)
        print('making time simulated  dataset of size {} ({}s)'.format(len(time), time[-1]))
        
        X0 = [0.1, 0.01, 0, 0]
        self.X, self.U = _sim.sim_with_input_fun(time, _ctl, X0)
        sim.plot_all(time, self.X, self.U)
        plt.subplot(5,1,1); plt.plot(time, _ctl.x_sp)
        plt.show()
    
        #self.enc_lw = self.enc_rw = np.zeros((len(time))

    


class ANN:
    _in_size, _out_size = 5, 4
    def __init__(self, filename=None):
        # input Xk
        # x_k, x_dot_k, theta_k, theta_dot_k, pwm_k
        plant_i = keras.layers.Input((self._in_size,), name ="plant_i")
        # x_k+1, x_dot_k+1, theta_k+1, theta_dot_k+1
        plant_l = keras.layers.Dense(4, activation='linear', kernel_initializer='uniform', use_bias=False, name="plant")
        plant_o = plant_l(plant_i)
        self.plant_ann = keras.models.Model(inputs=plant_i, outputs=plant_o)
        if filename is not None: self.load(filename)

    def train(self, _input, _output, epochs=250, filename=None, verbose=False):
        opt = keras.optimizers.Adam(lr=0.005, beta_1=0.9, beta_2=0.999, epsilon=None, decay=0.0, amsgrad=True)
        self.plant_ann.compile(loss='mean_squared_error', optimizer=opt)

        early_stopping = keras.callbacks.EarlyStopping(monitor='val_loss', patience=10)
        self.history = self.plant_ann.fit(_input, _output, epochs=epochs, batch_size=32,
                                          verbose=verbose, shuffle=True, validation_split=0.1, callbacks=[early_stopping])
        if filename is not None: self.save(filename)
     
    def plot_training_history(self, filename=None):
        fig = jpu.prepare_fig(window_title='Training History')
        ax = plt.subplot(1,1,1)
        plt.plot(self.history.history['loss'], label='loss')
        plt.plot(self.history.history['val_loss'], label='val_loss')
        jpu.decorate(ax, title='loss', legend=True);

    def report(self, _sim, dt=0.01):
        txt = '''
# Network:
   input Xk:Uk: [xk, x_dot_k, theta_k, theta_dot_k, pwm_k]
   output Xkp1: [xk+1, x_dot_k+1, theta_k+1, theta_dot_k+1]
   Weights:'''
        print(txt)
        _w = self.plant_ann.get_layer(name="plant").get_weights()[0]
        print _w.T
        Ad, Bd = _sim.disc_jac([0, 0, 0, 0], [0], dt)
        sim_weights = np.hstack((Ad, Bd))
        print('   Jacobian of the simulator:\n{}'.format(sim_weights))
        abs_err = _w.T - sim_weights
        #rel_err = np.divide(abs_err, sim_weights)
        #pdb.set_trace()
        print('   Abs err:\n{}'.format(abs_err))
        
    def make_io_chronogram(self, ds):
        _input = np.hstack((ds.X, ds.U))[:-1]
        _output = ds.X[1:]
        return _input, _output

    def make_random_io(self, sim, _nb=int(1e4), dt=0.01, _range=0.1):
        print('making random state/input uniform distribution dataset of size {}'.format(_nb))
        # simulate with random state and input distribution
        _input = np.random.uniform(low=-_range, high=_range, size=(_nb, self._in_size))
        _output = np.array([sim.dyn_disc(_inp[:self._out_size], 0, dt, _inp[self._out_size:]) for _inp in _input])
        return _input, _output
    
        
if __name__ == '__main__':
    keras.backend.set_floatx('float64')
    np.set_printoptions(precision=2, linewidth=300)
    ann = ANN()
    _sim = sim.Plant()
    if 1: # simulate closed loop dynamics with random setpoints
        #_ds = SimulatedPlanarDataset(_sim, _nb=int(1e4), _type='step')
        _ds = SimulatedPlanarDataset(_sim, _nb=int(1e4), _type='random')
        _input, _output = ann.make_io_chronogram(_ds)
    if 0: # simulate with random state and input distribution
        _input, _output = ann.make_random_io(_sim, _nb=int(1e4))
    
    ann.train(_input, _output, verbose=False)
    ann.plot_training_history()
    ann.report(_sim)
    plt.show()
