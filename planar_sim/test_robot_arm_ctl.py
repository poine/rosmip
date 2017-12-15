#! /usr/bin/env python
# -*- coding: utf-8 -*-


import logging, timeit, math, numpy as np, scipy.integrate, matplotlib.pyplot as plt, pickle
import sklearn.neural_network
import keras

''' let's try model reference control on the rorbot arm'''
LOG = logging.getLogger('test_keras2')

import test_robot_arm_plant_id as plant_model, my_utils as ut

import pdb

class NLI_Ctl:
    def __init__(self):
        pass

    def get(self, X, k):
        ome, xie = 10, 0.9
        o2, txo = ome**2, 2.*xie*ome
        U = 10*math.sin(X[0])-o2*X[0]+ X[1]*(2-txo)+ self.Xr[k, 2] + self.Xr[k, 1]*txo + self.Xr[k, 0]*o2 
        return U



class ANN_Ctl:
    x_km1, x_km2, xr_k, xr_km1, xr_km2, u_km1, input_size = range(7)
    
    def __init__(self):
        self.ann = keras.models.Sequential()
        self.ann.add(keras.layers.Dense(10, activation='linear', kernel_initializer='uniform', input_dim=7))
        self.ann.add(keras.layers.Dense( 1, activation='linear', kernel_initializer='uniform'))
        self.ann.compile(loss='mean_squared_error', optimizer='adam', metrics=['accuracy'])

    def make_input(self, X, Xr, U):
        _input = np.zeros(( len(X)- self.delay, self.input_size))
        for i in range(self.delay, len(X)):
            _input[i-self.delay, self.x_km1]   = X[i-1, 0]
            _input[i-self.delay, self.xd_km1]  = X[i-1, 1]
            _input[i-self.delay, self.u_km1]   = U[i-1]
        return _input

    def fit(self, time, X, U):
        LOG.info(' Fitting Plant ANN on {} data points'.format(len(time)))
        LOG.info('  preparing training set')
        ann_input, ann_output = self.make_input(X, U), X[self.delay:]
    
    def get(self, X, k):
        return 0




def test_ann_ctl():
    pass
    
        

class ANN_Full: # plant + controller network
    def __init__(self):
        ann_plant_filename = '/tmp/arm_plant_ann.pkl'
        LOG.info(' Loading ann from {}'.format(ann_plant_filename))
        with open(ann_plant_filename, "rb") as f:
            self.plant_ann, self.plant_scaler = pickle.load(f)
        self.plant_ann_keras = keras.models.load_model(ann_plant_filename+'.h5') 

        self.full_ann = keras.models.Sequential()
        
        #pdb.set_trace()
        #keras.utils.plot_model( self.plant_ann_keras, to_file='model.png', show_shapes=True)
        

def main( make_training_set=True, train=True, test=True):
    training_traj_filename = '/tmp/arm_ref_traj.pkl'
    dt = 1./100
    af = ANN_Full()
    ref = plant_model.Ref(dt)
    if train:
        if make_training_set:
            nsamples, max_nperiod = int(100*1e3), 10
            LOG.info('  Generating random setpoints')
            time, yc = ut.make_random_pulses(dt, nsamples, max_nperiod=max_nperiod,  min_intensity=-10, max_intensity=10.)
            LOG.info('   done. Generated {} random setpoints'.format(len(time)))
            LOG.info('  Simulating reference trajectory ({} s)'.format(time[-1]))
            Xr = ref.sim(time, [0, 0], yc)
            LOG.info('  Saving trajectory to {}'.format(training_traj_filename))
            desc = 'random setpoint trajectory. max_nperiod: {}'.format(max_nperiod)
            ut.save_trajectory(time, Xr, yc, desc, training_traj_filename)
        else:
            LOG.info('  Loading trajectory from {}'.format(training_traj_filename))
            time, X, U, desc = ut.load_trajectory(training_traj_filename)
            LOG.info('     {} samples ({} s)'.format(len(time), time[-1]))
            LOG.info('     desc: {}'.format(desc))
        plant_model.plot(time, X, U)


    if test:
        plant = plant_model.Plant()
        time =  np.arange(0., 15.05, dt)
        yc, d = ut.step_input_vec(time, dt=8), np.zeros(len(time))
        X0 = [0.5, 0.1]
        LOG.info(' Simulating test trajectory')
        Xr = ref.sim(time, X0, yc)
        ctl = NLI_Ctl()
        ctl.Xr = Xr
        X1, U1 = plant.sim(time, X0, ctl.get, d)
        plant_model.plot(time, Xr)
        plant_model.plot(time, X1, U1)

        ctl2 = ANN_Ctl()
        ctl2.fit(time, X, U)

        
        X2, U2 = plant.sim(time, X0, ctl2.get, d)
        plant_model.plot(time, X2, U2)
        
    plt.show()
            
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    np.set_printoptions(linewidth=300)
    main(make_training_set=False, train=False, test=True)
