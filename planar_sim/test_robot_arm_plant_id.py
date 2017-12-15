#! /usr/bin/env python
# -*- coding: utf-8 -*-

import logging, timeit, math, numpy as np, scipy.integrate, matplotlib.pyplot as plt, pickle
import sklearn.neural_network
import keras

''' let's try model reference control on the rorbot arm'''
LOG = logging.getLogger('test_tf')

import my_utils as ut
# https://pyrenn.readthedocs.io/en/latest/train.html

class Plant:
    def dyn(self, X, t, u, d):
        Xd = np.array([X[1], -10*np.sin(X[0]) - 2*X[1] + u + d])
        return Xd


    def sim(self, time, X0, ctl, d):
        X, U = np.zeros((len(time), 2)),  np.zeros((len(time), 1))
        X[0] = X0
        for i in range(1, len(time)):
            U[i-1] = ctl(X[i-1], i-1)
            _unused, X[i] = scipy.integrate.odeint(self.dyn, X[i-1], [time[i-1], time[i]], args=(U[i-1], d[i]))
        U[-1] = U[-2]
        return X, U


class Ref:
    def __init__(self, dt):
        self.X = np.zeros(2)
        self.dt = dt

    def cdyn(self, X, t, rk): return np.array([X[1], -9*X[0]-6*X[1]+9*rk])
        
    def get(self, rk):
        _unused, self.X = scipy.integrate.odeint(self.cdyn, self.X, [0, self.dt], args=(rk,))
        return self.X

    def sim(self, time, X0, yc):
        X = np.zeros((len(time), 3))
        X[0,:2] = X0; self.X = np.array(X0)
        for i in range(1, len(time)):
            X[i,:2] = self.get(yc[i])
            X[i,2] = self.cdyn(X[i], 0, yc[i])[1]
        return X
        

class ANN_Plant:
    ''' full state, delay 1 '''
    delay = 1
    x_km1, xd_km1, u_km1, input_size = range(4)
    def __init__(self):
        params = {
            'hidden_layer_sizes':(10,),  # 
            'activation':'relu',         # ‘identity’, ‘logistic’, ‘tanh’, ‘relu’
            'solver': 'adam',            # ‘lbfgs’, ‘sgd’, ‘adam’
            'verbose':True, 
            'random_state':1, 
            'max_iter':500, 
            'tol':1e-20,
            'warm_start': True
        }
        self.ann = sklearn.neural_network.MLPRegressor(**params) 
        self.scaler = None

        self.ann2 = keras.models.Sequential()
        self.ann2.add(keras.layers.Dense(10, activation='linear', kernel_initializer='uniform', input_dim=3))
        self.ann2.add(keras.layers.Dense( 2, activation='linear', kernel_initializer='uniform'))
        self.ann2.compile(loss='mean_squared_error', optimizer='adam', metrics=['accuracy'])

        
    def make_input(self, X, U):
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
        LOG.info('  done. Now scaling training set')
        self.scaler = sklearn.preprocessing.StandardScaler()
        scaled_input = self.scaler.fit_transform(ann_input)
        LOG.info('  done. Now fitting set')
        self.ann2.fit(scaled_input, ann_output, epochs=10, batch_size=128,  verbose=2)
        self.ann.fit(scaled_input , ann_output)
        LOG.info('  done')
        LOG.info('  score: {:f}'.format(self.ann.score(scaled_input , ann_output)))

    def get(self, x_km1, xd_km1, u_km1):
        r1 = self.ann.predict(self.scaler.transform([[x_km1, xd_km1, u_km1]]))
        r2 = self.ann2.predict(self.scaler.transform([[x_km1, xd_km1, u_km1]]))
        return r2

    def sim(self, time, X0, ctl, d):
        X, U = np.zeros((len(time), 2)),  np.zeros((len(time), 1))
        X[0] = X0
        for i in range(1, len(time)):
            U[i-1] = ctl(X[i-1], i-1)
            inp_im1 = (X[i-1, 0], X[i-1, 1], U[i-1])
            X[i] = self.get(*inp_im1).squeeze()
        U[-1] = U[-2]
        return X, U
    
    def save(self, filename):
        LOG.info('  saving ann to {}'.format(filename))
        with open(filename, "wb") as f:
            pickle.dump([self.ann, self.scaler], f)
        self.ann2.save(filename+'.h5')

    def load(self, filename):
        LOG.info(' Loading ann from {}'.format(filename))
        with open(filename, "rb") as f:
            self.ann, self.scaler = pickle.load(f)
        self.ann2 = keras.models.load_model(filename+'.h5')

            
class CtlNone:
    def get(self, X, k):
        return self.yc[k]
    
def test_ann_plant(dt, make_training_set=True, train=True, test=True):
    training_traj_filename = '/tmp/arm_training_traj.pkl'
    ann_plant_filename = '/tmp/arm_plant_ann.pkl'
    plant = Plant()
    ctl = CtlNone()
    ann = ANN_Plant()
    if train:
        if make_training_set:
            nsamples, max_nperiod = int(100*1e3), 10
            LOG.info('  Generating random setpoints')
            time, ctl.yc = ut.make_random_pulses(dt, nsamples, max_nperiod=max_nperiod,  min_intensity=-10, max_intensity=10.)
            LOG.info('   done. Generated {} random setpoints'.format(len(time)))
            LOG.info('  Simulating trajectory ({} s)'.format(time[-1]))
            X0 = [0., 0.]
            X, U = plant.sim(time, X0, ctl.get, np.zeros(len(time)))
            LOG.info('   done')
            LOG.info('  Saving trajectory to {}'.format(training_traj_filename))
            desc = 'random setpoint trajectory. max_nperiod: {}'.format(max_nperiod)
            ut.save_trajectory(time, X, U, desc, training_traj_filename)
        else:
            LOG.info('  Loading trajectory from {}'.format(training_traj_filename))
            time, X, U, desc = ut.load_trajectory(training_traj_filename)
            LOG.info('     {} samples ({} s)'.format(len(time), time[-1]))
            LOG.info('     desc: {}'.format(desc))

        #plot(time, X, U)
        ann.fit(time, X, U)
        ann.save(ann_plant_filename)
    else:
        ann.load(ann_plant_filename)

    if test:
        time =  np.arange(0., 15.05, dt)
        ctl.yc = ut.step_input_vec(time)
        d = np.zeros(len(time))
        X0 = [0.5, 0.1]
        LOG.info(' Simulating test trajectory')
        X1, U1 = plant.sim(time, X0, ctl.get, d)
        X2, U2 = ann.sim(time, X0, ctl.get, d)
        plot(time, X1, U1)
        plot(time, X2, U2)
        plt.legend(['plant','ANN'])
            
def plot(time, X, U=None, Yc=None):
    ax = plt.subplot(3,1,1)
    plt.plot(time, X[:,0])
    if Yc is not None: plt.plot(time, Yc, 'k')
    ut.decorate(ax, title="$x_1$", ylab='time')
    ax = plt.subplot(3,1,2)
    plt.plot(time, X[:,1])
    ut.decorate(ax, title="$x_2$", ylab='time')
    if U is not None:
        ax = plt.subplot(3,1,3)
        plt.plot(time, U)
        ut.decorate(ax, title="$u$", ylab='time')
    

def main():
    dt = 1./100
    test_ann_plant(dt, make_training_set=False, train=False, test=True)
    plt.show()
    return
    time = np.arange(0., 15.1, dt)
    X0 = [0.5, 0.]
    def ctl(X, k): return 0
    d = np.zeros(len(time))
    plant = Plant()
    X, U = plant.sim(time, X0, ctl, d)
    plot(time, X, U)

    yc = ut.step_input_vec(time, dt=8, t0=0)  # input setpoint
    ref = Ref(dt)
    Xref = np.array([np.array(ref.get(yci)) for yci in yc])
    plot(time, Xref)


    plt.show()
    
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    np.set_printoptions(linewidth=300)
    main()

