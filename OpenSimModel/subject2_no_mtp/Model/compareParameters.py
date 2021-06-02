# -*- coding: utf-8 -*-
"""
Created on Wed Jun  2 06:55:46 2021

@author: u0101727
"""

import numpy as np

old_parameters = np.load('mtParameters.npy')
new_parameters = np.load('mtParameters_temp.npy')

diff_parameters = np.max(np.abs(old_parameters - new_parameters))