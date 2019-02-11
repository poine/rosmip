# distutils: language = c++
#
#
# RosmipControl is a python interface using cython 
#
# Author: Poine-2019
#

# I should look at that https://github.com/longjie/ros_cython_example

import numpy as np
import tf.transformations

cimport numpy as np
from libcpp cimport bool
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.map cimport map
from libc.string cimport memcpy
from cython.operator cimport dereference as deref, preincrement as inc
from cpython cimport array

##
# this is to remove the ‘int _import_array()’ defined but not used [-Wunused-function] warning
cdef void foo():
    np.import_array()
def bar():
    foo()



cdef extern from "rosmip_control/legacy_ctl_law.h" namespace "rosmip_controller":
    cdef cppclass c_LegacyCtlLaw "rosmip_controller::LegacyCtlLaw":
        c_LegacyCtlLaw()
        void reset()
        void init()
        void starting()
        void update(double theta, double lwa, double rwa, double lin_sp, double ang_sp)
        double get_pwm_left()
        double get_pwm_right()
        void set_d1_params(const double gain, double* _num, double* den)

cdef class LegacyCtlLaw:
    cdef c_LegacyCtlLaw *thisptr   

    def __cinit__(self):
        self.thisptr = new c_LegacyCtlLaw()

    def init(self):
        self.thisptr.init()

    def update(self, theta, lwa, rwa, lin_sp, ang_sp):
        self.thisptr.update(theta, lwa, rwa, lin_sp, ang_sp)
        return self.thisptr.get_pwm_left(), self.thisptr.get_pwm_right()

    def set_inner_loop_coeffs(self, d1_gain, d1_num, d1_den):
        print('setting inner loop coeffs', d1_gain, d1_num, d1_den)
        cdef array.array _num = array.array('d', d1_num)
        cdef array.array _den = array.array('d', d1_den)
        self.thisptr.set_d1_params(d1_gain, _num.data.as_doubles, _den.data.as_doubles)
