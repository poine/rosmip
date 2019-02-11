#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np, matplotlib.pyplot as plt
import scipy.signal, scipy.optimize, control, control.matlab
import keras
import pdb

import julie_misc.plot_utils as jpu
import ident_plant

# https://programtalk.com/vs2/?source=python/10610/python-control/external/yottalab.py

def d2c(sys,method='zoh'):
    """Continous to discrete conversion with ZOH method
    Call:
    sysc=c2d(sys,method='log')
    Parameters
    ----------
    sys :   System in statespace or Tf form 
    method: 'zoh' or 'bi'
    Returns
    -------
    sysc: continous system ss or tf
    
    """
    flag = 0
    if isinstance(sys, control.TransferFunction):
        sys=tf2ss(sys)
        flag=1

    a=sys.A
    b=sys.B
    c=sys.C
    d=sys.D
    Ts=sys.dt
    n=np.shape(a)[0]
    nb=np.shape(b)[1]
    nc=np.shape(c)[0]
    tol=1e-12
    
    if method=='zoh':
        if n==1:
            if b[0,0]==1:
                A=0
                B=b/sys.dt
                C=c
                D=d
        else:
            tmp1=np.hstack((a,b))
            tmp2=np.hstack((np.zeros((nb,n)),np.eye(nb)))
            tmp=np.vstack((tmp1,tmp2))
            s=scipy.linalg.logm(tmp)
            s=s/Ts
            if np.linalg.norm(np.imag(s), np.inf) > np.sqrt(np.finfo(float).eps):
                print "Warning: accuracy may be poor"
            s=np.real(s)
            A=s[0:n,0:n]
            B=s[0:n,n:n+nb]
            C=c
            D=d
    elif method=='foh':
        a=mat(a)
        b=mat(b)
        c=mat(c)
        d=mat(d)
        Id = mat(eye(n))
        A = logm(a)/Ts
        A = real(around(A,12))
        Amat = mat(A)
        B = (a-Id)**(-2)*Amat**2*b*Ts
        B = real(around(B,12))
        Bmat = mat(B)
        C = c
        D = d - C*(Amat**(-2)/Ts*(a-Id)-Amat**(-1))*Bmat
        D = real(around(D,12))
    elif method=='bi':
        a=mat(a)
        b=mat(b)
        c=mat(c)
        d=mat(d)
        poles=eigvals(a)
        if any(abs(poles-1)<200*sp.finfo(float).eps):
            print "d2c: some poles very close to one. May get bad results."
        
        I=mat(eye(n,n))
        tk = 2 / sqrt (Ts)
        A = (2/Ts)*(a-I)*inv(a+I)
        iab = inv(I+a)*b
        B = tk*iab
        C = tk*(c*inv(I+a))
        D = d- (c*iab)
    else:
        print "Method not supported"
        return
    
    sysc=control.StateSpace(A,B,C,D)
    if flag==1:
        sysc=ss2tf(sysc)
    return sysc


def dlqr(A, B, Q, R):
    """Linear quadratic regulator design for discrete systems
 
    Usage
    =====
    [K, S, E] = dlqr(A, B, Q, R)
 
    The dlqr() function computes the optimal state feedback controller
    that minimizes the quadratic cost
 
        J = \sum_0^\infty x' Q x + u' R u + 2 x' N u
 
    Inputs
    ------
    A, B: 2-d arrays with dynamics and input matrices
    Q, R: 2-d array with state and input weight matrices
 
    Outputs
    -------
    K: 2-d array with state feedback gains
    S: 2-d array with solution to Riccati equation
    E: 1-d array with eigenvalues of the closed loop system
    """
 
    # Check dimensions for consistency
    nstates = B.shape[0];
    ninputs = B.shape[1];
    if (A.shape[0] != nstates or A.shape[1] != nstates):
        raise ControlDimension("inconsistent system dimensions")
 
    elif (Q.shape[0] != nstates or Q.shape[1] != nstates or
          R.shape[0] != ninputs or R.shape[1] != ninputs):
        raise ControlDimension("incorrect weighting matrix dimensions")
 
  
    Ao=A
    Qo=Q
     
    #Solve the riccati equation
    (X,L,G) = control.dare(Ao,B,Qo,R)
 
    # Now compute the return value
    Phi=np.mat(A)
    H=np.mat(B)
    K=np.linalg.inv(H.T*X*H+R)*(H.T*X*Phi)
    L=np.linalg.eig(Phi-H*K)
    return K,X,L



def sim_cl_ann(ann, K, dt=0.01):
    time = np.arange(0, 15, dt)
    X, U = np.zeros((len(time), 6)), np.zeros((len(time), 2))
    phi0, gamma0, theta0 = np.deg2rad(1), np.deg2rad(1), np.deg2rad(1)
    X[0] = [phi0, gamma0, 0, 0, theta0, 0]
    for k in range(1, len(time)):
        U[k-1] = -np.dot(K, X[k-1])
        _in_km1 = np.hstack((X[k-1], U[k-1]))[np.newaxis,:]
        X[k] = ann.predict(_in_km1)

    ax = plt.subplot(3,1,1)
    plt.plot(time, np.rad2deg(X[:,0]))
    jpu. decorate(ax, title='phi', xlab='time in s', ylab='deg', legend=True)
    ax = plt.subplot(3,1,2)
    plt.plot(time, np.rad2deg(X[:,1]))
    jpu. decorate(ax, title='gamma', xlab='time in s', ylab='deg', legend=True)
    ax = plt.subplot(3,1,3)
    plt.plot(time, np.rad2deg(X[:,4]))
    jpu. decorate(ax, title='theta', xlab='time in s', ylab='deg', legend=True)
    plt.show()

def main(dt=0.01):
    ann = ident_plant.ANN('/tmp/rosmip_ann.h5')
    Ad, Bd = ann.report()
    # phi, gamma, phi_dot, gamma_dot, theta, theta_dot

    if 0: # continuous time LQR
        C, D = [1, 0, 0, 0, 0, 0], [0, 0]
        ss_d = control.StateSpace(Ad,Bd,C,D, dt)
        ss_c = d2c(ss_d)
        #Ac = scipy.linalg.logm(Ad)/dt
        #tmp = np.linalg.inv(Ad-np.eye(Ad.shape[0]))
        #Bc = np.dot(tmp, np.dot(Ac, Bd))
        Q = np.diag([1., 1., 0.1, 0.1, 0.1, 0.01])
        R = np.diag([6, 6])
        (K, X, E) = control.matlab.lqr(ss_c.A, ss_c.B, Q, R)
        print('gain\n{}'.format(K))
        Acl = ss_c.A - np.dot(ss_c.B, K)
        eva, eve = np.linalg.eig(Acl)
        print('continuous time closed loop poles\n{}'.format(eva))
        #pdb.set_trace()

    if 1: # discrete time LQR
        Q = np.diag([20., 20., 0.1, 0.5, 0.1, 0.01])
        R = np.diag([6, 6])
        (K, X, E) = dlqr(Ad, Bd, Q, R)
        print('gain\n{}'.format(K))
        print('closed loop discrete time poles {}'.format(E[0]))
        cl_polesc = np.log(E[0])/dt
        print('closed loop continuous time poles {}'.format(cl_polesc))
        #pdb.set_trace()
        
        
    if 0: # discrete time place
        poles = [-12+12j, -12-12j, -6.5+1j, -6.5-1j, -1-1j, -1+1j]
        K = control.matlab.place(Ad, Bd, poles)
    
    sim_cl_ann(ann, K)
              


    
if __name__ == "__main__":
    #logging.basicConfig(level=logging.INFO)
    keras.backend.set_floatx('float64')
    np.set_printoptions(precision=2, linewidth=300)
    main()
