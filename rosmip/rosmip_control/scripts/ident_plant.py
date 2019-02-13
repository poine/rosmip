#!/usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np, matplotlib.pyplot as plt
import keras, control

import pdb

import julie_misc.plot_utils as jpu
import homere_control.io_dataset as iod
import homere_control.utils as hcu

'''

  First attempt at identifying RosMip in Gazebo

'''


class ANN:
    def __init__(self, filename=None):

        self.wr, self.ws = 0.03, 0.083
        # phi =   (wheelAngleL+wheelAngleR)/2 + theta
        # gamma = (wheelAngleR-wheelAngleL) * WHEEL_RADIUS_M / WHEEL_TRACK_M

        # input Xk
        # phi_k, gamma_k, phi_dot_k, gamma_dot_k, theta_k, theta_dot_k, pwm_l_k, pwm_r_k
        plant_i = keras.layers.Input((8,), name ="plant_i")
        plant_l = keras.layers.Dense(6, activation='linear', kernel_initializer='uniform', use_bias=False, name="plant")
        # phi_kp1, gamma_kp1, phi_dot_kp1, gamma_dot_kp1, theta_kp1, theta_dot_kp1
        plant_o = plant_l(plant_i)
        self.plant_ann = keras.models.Model(inputs=plant_i, outputs=plant_o)
        if filename is not None:
            self.load(filename)
    
    def make_io(self, ds):
        theta = ds.pitch[:,np.newaxis]
        # for now imu pitch dot seems broken, get theta_dot from truth
        pitch_dot = ds.truth_rvel[:, 1]
        theta_dot = iod.interpolate(pitch_dot[:,np.newaxis], ds.truth_vel_stamp, ds.enc_stamp)
        lwa, rwa = ds.enc_lw[:,np.newaxis], ds.enc_rw[:,np.newaxis]
        phi, gamma = (lwa + rwa)/2.+theta, (rwa-lwa)*self.wr/self.ws
        lwrv = iod.interpolate(ds.enc_vel_lw[:,np.newaxis], ds.enc_vel_stamp, ds.enc_stamp)
        rwrv = iod.interpolate(ds.enc_vel_rw[:,np.newaxis], ds.enc_vel_stamp, ds.enc_stamp)
        phi_dot = (lwrv+rwrv)/2. + theta_dot
        gamma_dot = (rwrv-lwrv) * self.wr/self.ws

        Xs = np.hstack([phi, gamma, phi_dot, gamma_dot, theta, theta_dot])
        _input = np.hstack([Xs, ds.lw_pwm[:,np.newaxis], ds.rw_pwm[:,np.newaxis]])[:-1]
        _output = Xs[1:]
        #pdb.set_trace()
        return _input, _output, ds.enc_stamp[:-1]

    def train(self, _input, _output, _time, epochs=250):

        opt = keras.optimizers.Adam(lr=0.005, beta_1=0.9, beta_2=0.999, epsilon=None, decay=0.0, amsgrad=True)
        self.plant_ann.compile(loss='mean_squared_error', optimizer=opt)

        early_stopping = keras.callbacks.EarlyStopping(monitor='val_loss', patience=10)
        self.history = self.plant_ann.fit(_input, _output, epochs=epochs, batch_size=32,  verbose=1, shuffle=True, validation_split=0.1, callbacks=[early_stopping])

        
    def plot_training_history(self, filename=None):
        fig = jpu.prepare_fig(window_title='Training History')
        ax = plt.subplot(1,1,1)
        plt.plot(self.history.history['loss'], label='loss')
        plt.plot(self.history.history['val_loss'], label='val_loss')
        jpu.decorate(ax, title='loss', legend=True);



    def plot_io_chronogram(self, _input, _output, _time):
        figsize=(20.48, 5.12)
        fig = jpu.prepare_fig(window_title='Input/Output', figsize=figsize)
        plots = [(np.rad2deg(_input[:,0]), 'phi', 'deg'),
                 (np.rad2deg(_input[:,1]), 'gamma', 'deg'),
                 (np.rad2deg(_input[:,4]), 'theta', 'deg'),
                 (np.rad2deg(_input[:,2]), 'phi dot', 'deg/s'),
                 (np.rad2deg(_input[:,3]), 'gamma dot', 'deg/s'),
                 (np.rad2deg(_input[:,5]), 'theta dot', 'deg'),
                 (_input[:,6], 'pwm l', ''),
                 (_input[:,7], 'pwm r', '')
        ]
        for i, (_d, _n, _u) in enumerate(plots):
            ax = plt.subplot(3,3,i+1)
            #pdb.set_trace()
            plt.plot(_time, _d)
            jpu. decorate(ax, title=_n, xlab='time in s', ylab=_u)

    def predict(self, _input):
        #Xm[k] = ann.predict(np.array([[Xm[k-1,0], Xm[k-1,1], Xm[k-1,2], Xm[k-1,3], Um[k-1,0]]]))
        return self.plant_ann.predict(_input)


    def save(self, filename):
        # Save it to avoid retraining
        self.plant_ann.save(filename)
        
    def load(self, filename):
        # Load a previously trained ANN
        print('plant loading {}'.format(filename))
        self.plant_ann = keras.models.load_model(filename)

    def report(self):
        txt = '''
# Network:
   input Xk:Uk: [phi_k, gamma_k, phi_dot_k, gamma_dot_k, theta_k, theta_dot_k, pwm_l_k, pwm_r_k]
   output Xkp1: [phi_kp1, gamma_kP1, phi_dot_kp1, gamma_dot_kp1, theta_kp1, theta_dot_kp1]
'''
        print(txt)
        _w = self.plant_ann.get_layer(name="plant").get_weights()[0]
        print _w.T

        A, B = _w.T[:, :6], _w.T[:, 6:] 
        eva, eve = np.linalg.eig(A)
        txt = '''
# SSR
A
{}
B
{}
eva
{}
ev
{}
'''.format(A, B, eva, eve)
        print txt
        return A, B

def plot_dataset(_ds):
    #iod.plot_imu(_ds)
    integ_pitch = np.zeros(len(_ds.enc_stamp))
    integ_pitch[0] = _ds.pitch[0]
    for k in range(1, len(_ds.enc_stamp)):
        integ_pitch[k] = integ_pitch[k-1] + _ds.pitch_dot[k-1]*(_ds.enc_stamp[k]-_ds.enc_stamp[k-1])
    #pdb.set_trace()
    truth_pitch = np.array([hcu.rpy_of_q(_q) for _q in _ds.truth_ori])

    figsize=(20.48, 5.12)
    fig = jpu.prepare_fig(window_title='IMU', figsize=figsize)
    ax = plt.subplot(2,1,1)
    plt.plot(_ds.enc_stamp, np.rad2deg(_ds.pitch), '.', label="IMU")
    plt.plot(_ds.truth_stamp, np.rad2deg(truth_pitch[:,1]), label='truth')
    plt.plot(_ds.enc_stamp, np.rad2deg(integ_pitch), label='integrated IMU pitch dot')
    jpu. decorate(ax, title='Pitch', xlab='time in s', ylab='deg', legend=True)
        
    ax = plt.subplot(2,1,2)
    plt.plot(_ds.enc_stamp, np.rad2deg(_ds.pitch_dot), '.', label="IMU")
    plt.plot(_ds.truth_vel_stamp, np.rad2deg(_ds.truth_rvel[:,1]), label='truth')
    jpu. decorate(ax, title='Pitch dot', xlab='time in s', ylab='deg/s', legend=True)
    


def validate_on_ioset(ann, _input, _time):
    Xann = np.zeros((len(_time), 6))
    Xann[0] = _input[0,6]
    for k in range(1, len(_time)):
        in_km1 = np.concatenate((Xann[k-1], _input[k-1,6:]))[np.newaxis, :]
        Xann[k] = ann.predict(in_km1)
    
    pdb.set_trace()
    ann.plot_io_chronogram(np.hstack([Xann, _input[:,6:]])[:100], None, _time[:100])

    

if __name__ == '__main__':
    keras.backend.set_floatx('float64')
    np.set_printoptions(precision=2, linewidth=300)
    #filename, _type = '/home/poine/work/homere/homere_control/data/rosmip/gazebo/rosmip_io_01_step_lin.npz', 'rosmip'
    #filename, _type = '/home/poine/work/homere/homere_control/data/rosmip/gazebo/rosmip_io_04_sine_2.npz', 'rosmip'
    #filename, _type = '/home/poine/work/homere/homere_control/data/rosmip/gazebo/rosmip_io_05_random_2.npz', 'rosmip'
    #filename, _type = '/home/poine/work/homere/homere_control/data/rosmip/gazebo/rosmip_io_06_step_misc.npz', 'rosmip'
    #filename, _type = '/mnt/mint18/home/poine/work/homere/homere_control/data/rosmip/gazebo/rosmip_io_07_random_2.npz', 'rosmip'
    filename, _type = '/mnt/mint18/home/poine/work/homere/homere_control/data/rosmip/gazebo/rosmip_2_io_08_random_2.npz', 'rosmip'
    ds = iod.DataSet(filename, _type)
    # there's a bug in here
    # we use truth rather than IMU in simulation
    plot_dataset(ds)
    plt.show()
    ann = ANN()
    _input, _output, _time =  ann.make_io(ds)
    ann_filename = '/tmp/rosmip_ann.h5'
    if True:
        ann.train(_input, _output, _time)
        ann.save(ann_filename)
        ann.plot_training_history()

    else:
        ann.load(ann_filename)
        ann.report()
                
    # unstable, can not work...
    #validate_on_ioset(ann, _input, _time)
    
    ann.plot_io_chronogram(_input, _output, _time)
    plt.show()

    
