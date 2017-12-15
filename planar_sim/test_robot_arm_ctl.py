#! /usr/bin/env python
# -*- coding: utf-8 -*-


import logging, timeit, math, numpy as np, scipy.integrate, matplotlib.pyplot as plt, pickle
import sklearn.neural_network
import keras

''' let's try model reference control on the rorbot arm'''
LOG = logging.getLogger('test_keras2')

import test_tf as plant_model, utils as ut

import pdb

class ANN_Full: # plant + controller network
    def __init__(self):
        ann_plant_filename = '/tmp/arm_plant_ann.pkl'
        LOG.info(' Loading ann from {}'.format(ann_plant_filename))
        with open(ann_plant_filename, "rb") as f:
            self.plant_ann, self.plant_scaler = pickle.load(f)
        self.plant_ann_keras = keras.models.load_model(ann_plant_filename+'.h5') 

        self.full_ann = keras.models.Sequential()
        
        #pdb.set_trace()


def main( make_training_set=True, train=True):
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
            
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    np.set_printoptions(linewidth=300)
    main()
