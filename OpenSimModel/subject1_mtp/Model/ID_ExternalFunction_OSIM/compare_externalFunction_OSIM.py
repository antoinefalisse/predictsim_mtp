# This script tests if the model pragrammatically built in the external
# function dynamically matches the corresponding model in the .osim file.
# This is tested by comparing inverse dynamics (ID) results.

import os
import casadi as ca
import numpy as np

# %% User inputs.
# Name of the external function.
F_name = "PredSim_mtpPin_cm7"  
ID_sto_name = "subject1_mtp_OS4_cm7"  
# Number of degrees of freedom in the model.
F_N = 31
# Order of the joints in the external function.
joints = ['pelvis_tilt', 'pelvis_list', 'pelvis_rotation', 'pelvis_tx', 
          'pelvis_ty', 'pelvis_tz', 'hip_flexion_l', 'hip_adduction_l', 
          'hip_rotation_l', 'hip_flexion_r', 'hip_adduction_r', 
          'hip_rotation_r', 'knee_angle_l', 'knee_angle_r', 'ankle_angle_l', 
          'ankle_angle_r', 'subtalar_angle_l', 'subtalar_angle_r', 
          'mtp_angle_l', 'mtp_angle_r', 
          'lumbar_extension', 'lumbar_bending', 'lumbar_rotation', 
          'arm_flex_l', 'arm_add_l', 'arm_rot_l', 'arm_flex_r', 'arm_add_r', 
          'arm_rot_r', 'elbow_flex_l', 'elbow_flex_r']
    
# %% Get ID from external function.
pathMain = os.getcwd()
pathModels = os.path.dirname(pathMain)
pathSubject = os.path.dirname(pathModels)
pathOpenSim = os.path.dirname(pathSubject)
pathRepo = os.path.dirname(pathOpenSim)

pathExternalFunction = os.path.join(pathRepo, 'ExternalFunction')
os.chdir(pathExternalFunction)
F = ca.external('F', F_name + ".dll")
os.chdir(pathMain)

vec1 = np.zeros((F_N*2, 1))
vec1[::2, :] = -1
vec2 = np.zeros((F_N, 1))
vec3 = np.concatenate((vec1,vec2))
ID_F = (F(vec3)).full() 

# %% Get ID from .osim file.
def storage2numpy(storage_file, excess_header_entries=0):
    """Returns the data from a storage file in a numpy format. Skips all lines
    up to and including the line that says 'endheader'.
    Parameters
    ----------
    storage_file : str
        Path to an OpenSim Storage (.sto) file.
    Returns
    -------
    data : np.ndarray (or numpy structure array or something?)
        Contains all columns from the storage file, indexable by column name.
    excess_header_entries : int, optional
        If the header row has more names in it than there are data columns.
        We'll ignore this many header row entries from the end of the header
        row. This argument allows for a hacky fix to an issue that arises from
        Static Optimization '.sto' outputs.
    Examples
    --------
    Columns from the storage file can be obtained as follows:
        >>> data = storage2numpy('<filename>')
        >>> data['ground_force_vy']
    """
    # What's the line number of the line containing 'endheader'?
    f = open(storage_file, 'r')

    header_line = False
    for i, line in enumerate(f):
        if header_line:
            column_names = line.split()
            break
        if line.count('endheader') != 0:
            line_number_of_line_containing_endheader = i + 1
            header_line = True
    f.close()

    # With this information, go get the data.
    if excess_header_entries == 0:
        names = True
        skip_header = line_number_of_line_containing_endheader
    else:
        names = column_names[:-excess_header_entries]
        skip_header = line_number_of_line_containing_endheader + 1
    data = np.genfromtxt(storage_file, names=names,
            skip_header=skip_header)

    return data
ID_OSIM = storage2numpy("ID_" + ID_sto_name + ".sto")

# %% Compare both ID results.
for j, joint in enumerate(joints):
    if ((joint == 'pelvis_tx') or (joint == 'pelvis_ty') or 
        (joint == 'pelvis_tz')): 
        print(np.abs(ID_F[j] - ID_OSIM[joint + "_force"][0]))
        # assert np.alltrue(np.abs(ID_F[j] - ID_OSIM[joint + "_force"][0]) < 1e-6), "forces"
    else:
        print(np.abs(ID_F[j] - ID_OSIM[joint + "_moment"][0]))
        # assert np.alltrue(np.abs(ID_F[j] - ID_OSIM[joint + "_moment"][0]) < 1e-6), "moments" 
