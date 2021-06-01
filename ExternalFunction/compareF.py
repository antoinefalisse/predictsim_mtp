# -*- coding: utf-8 -*-
"""
Created on Tue Jun  1 12:05:28 2021

@author: u0101727
"""

import casadi as ca
import numpy as np

F = ca.external('F','PredSim_no_mtpPin_cm0.dll')
baseConfig = 'b'
F1 = ca.external('F','PredSim_no_mtpPin_cm0{}.dll'.format(baseConfig))

vec = -np.ones((87, 1))
res = (F(vec)).full()
res1 = (F1(vec)).full()

assert np.alltrue(np.abs(res - res1) < 1e-6), 'test'