# -*- coding: utf-8 -*-
"""
Created on Mon Mar 22 07:40:29 2021

@author: u0101727
"""

import os
import opensim
import numpy as np
import sys
sys.path.append("./../../../") # utilities in parent directory



scriptDir = os.getcwd()

suffix = "_generic"
modelName = "subject2_withoutMTP_weldRadius_scaled_FK_contactsAsForces" + suffix
pathModel = os.path.join(scriptDir, modelName + ".osim")
coordinates = ['pelvis_tilt', 'pelvis_list', 'pelvis_rotation', 'pelvis_tx', 
              'pelvis_ty', 'pelvis_tz', 'hip_flexion_l', 'hip_adduction_l', 
              'hip_rotation_l', 'hip_flexion_r', 'hip_adduction_r', 
              'hip_rotation_r', 'knee_angle_l', 'knee_angle_r', 'ankle_angle_l', 
              'ankle_angle_r', 'subtalar_angle_l', 'subtalar_angle_r', 
              'lumbar_extension', 'lumbar_bending', 'lumbar_rotation', 
              'arm_flex_l', 'arm_add_l', 'arm_rot_l', 'arm_flex_r', 'arm_add_r', 
              'arm_rot_r', 'elbow_flex_l', 'elbow_flex_r']
nCoordinates = len(coordinates)

# Run ID with the .osim file
pathGenericIDFolder = os.path.join(scriptDir, "ExternalFunction") 
pathGenericIDSetupFile = os.path.join(pathGenericIDFolder, "SetupID.xml")

idTool = opensim.InverseDynamicsTool(pathGenericIDSetupFile)
idTool.setName("ID_withOsimAndIDTool")
idTool.setModelFileName(pathModel)
idTool.setResultsDir(pathGenericIDFolder)
idTool.setCoordinatesFileName(os.path.join(pathGenericIDFolder,
                                           "DefaultPosition.mot"))
idTool.setOutputGenForceFileName("ID_withOsimAndIDTool" + suffix + ".sto")       
pathSetupID = os.path.join(pathGenericIDFolder, "SetupID" + suffix + ".xml")
idTool.printToXML(pathSetupID)

command = 'opensim-cmd' + ' run-tool ' + pathSetupID
os.system(command)

# Extract torques from .osim + ID tool.    
headers = []
for coord in coordinates:    
    if (coord == "pelvis_tx" or 
        coord == "pelvis_ty" or 
        coord == "pelvis_tz"):
        suffix_header = "_force"
    else:
        suffix_header = "_moment"
    headers.append(coord + suffix_header)
    
from variousFunctions import storage2df    
ID_osim_df = storage2df(os.path.join(pathGenericIDFolder,
                              "ID_withOsimAndIDTool" + suffix + ".sto"), headers)
ID_osim = np.zeros((nCoordinates))
# for count, coordinateOrder in enumerate(coordinatesOrder):
#     if (coordinateOrder == "pelvis_tx" or 
#         coordinateOrder == "pelvis_ty" or 
#         coordinateOrder == "pelvis_tz"):
#         suffix_header = "_force"
#     else:
#         suffix_header = "_moment"

for count, header in enumerate(headers):
    ID_osim[count] = ID_osim_df.iloc[0][header]

# Extract torques from external function
import casadi as ca
os.chdir(pathGenericIDFolder)
F = ca.external('F', modelName + '.dll') 
os.chdir(scriptDir)

vec1 = np.zeros((len(headers)*2, 1))
vec1[::2, :] = -1     
vec2 = np.zeros((len(headers), 1))
vec3 = np.concatenate((vec1,vec2))
ID_F = (F(vec3)).full().flatten()[:nCoordinates]  

assert(np.max(np.abs(ID_osim - ID_F)) < 1e-1), "error F vs ID tool & osim"
