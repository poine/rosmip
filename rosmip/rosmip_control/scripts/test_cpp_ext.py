#!/usr/bin/env python

import numpy as np, matplotlib.pyplot as plt
import scipy.signal
import pdb
import keras

import homere_control.io_dataset as iod
import cpp_rosmip_control
import ident_plant

'''
running the identified plant on real control
'''


if __name__ == '__main__':
    keras.backend.set_floatx('float64')
    np.set_printoptions(precision=2, linewidth=300)

    ann = ident_plant.ANN()
    ann_filename = '/tmp/rosmip_ann.h5'
    ann.load(ann_filename)
    ann.report()
    
    filename, _type = '/home/poine/work/homere/homere_control/data/rosmip/gazebo/rosmip_io_04_sine_2.npz', 'rosmip'
    ds = iod.DataSet(filename, _type)
    
    # test control vs ann
    ctl = cpp_rosmip_control.LegacyCtlLaw()
    # original
    ctl.set_inner_loop_coeffs(1.05, [-4.945, 8.862, -3.967], [1.000, -1.481, 0.4812])
    # test
    #ctl.set_inner_loop_coeffs(1.05, [-3.945, 8.862, -3.967], [1.000, -1.481, 0.8])

    time = np.arange(0, 10, 0.01)
    U = np.zeros((len(time), 2))
    X = np.zeros((len(time), 6))
    Sp = np.zeros((len(time), 2))
    Sp[:,0] = scipy.signal.square(time)
    #Sp[:,1] = scipy.signal.square(time)
    
    for k in range(1, len(time)):
        phi, gamma, theta = X[k-1, 0], X[k-1, 1], X[k-1, 4]
        rwa_p_lwa = (phi-theta)*2
        rwa_m_lwa = gamma/ann.wr*ann.ws
        lwa = (rwa_p_lwa - rwa_m_lwa)/2
        rwa = (rwa_p_lwa + rwa_m_lwa)/2
        U[k-1] = ctl.update(theta, lwa, rwa, Sp[k-1,0], Sp[k-1,1] )
        X[k] = ann.predict(np.hstack([X[k-1], U[k-1]])[np.newaxis,:])
    U[-1] = U[-2]
    pdb.set_trace()
    _in = np.hstack([X, U])
    ann.plot_io_chronogram(_in, None, time)
    #plt.plot(X)
    plt.show()


