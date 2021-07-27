# -*- coding: utf-8 -*-
"""
Created on Tue Jun  1 12:05:28 2021

@author: u0101727
"""

import casadi as ca
import numpy as np

F = ca.external('F','PredSim_no_mtpPin_cm0c.dll')
baseConfig = 'f'
# F1 = ca.external('F','PredSim_no_mtpPin_cm0{}.dll'.format(baseConfig))
F1 = ca.external('F','s2_withoutMTP_ge_19{}.dll'.format(baseConfig))

vec = -np.zeros((87, 1))
res = (F(vec)).full()
res1 = (F1(vec)).full()

temp = np.abs(res - res1)

assert np.alltrue(np.abs(res - res1) < 1e-10), 'test'