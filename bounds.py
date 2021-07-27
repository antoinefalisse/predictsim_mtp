import scipy.interpolate as interpolate
import pandas as pd
import numpy as np

class bounds:
    
    def __init__(self, Qs, joints, muscles, armJoints, targetSpeed, 
                 mtpJoints=0):
        
        self.Qs = Qs
        self.joints = joints
        self.targetSpeed = targetSpeed
        self.muscles = muscles
        self.armJoints = armJoints
        self.mtpJoints = mtpJoints
        
    def splineQs(self):
        
        self.Qs_spline = self.Qs.copy()
        self.Qdots_spline = self.Qs.copy()
        self.Qdotdots_spline = self.Qs.copy()

        for joint in self.joints:
            spline = interpolate.InterpolatedUnivariateSpline(self.Qs['time'], 
                                                              self.Qs[joint],
                                                              k=3)
            self.Qs_spline[joint] = spline(self.Qs['time'])
            splineD1 = spline.derivative(n=1)
            self.Qdots_spline[joint] = splineD1(self.Qs['time'])
            splineD2 = spline.derivative(n=2)
            self.Qdotdots_spline[joint] = splineD2(self.Qs['time'])
    
    def getBoundsPosition(self):
        self.splineQs()
        upperBoundsPosition = pd.DataFrame()   
        lowerBoundsPosition = pd.DataFrame() 
        scalingPosition = pd.DataFrame() 
        for count, joint in enumerate(self.joints):  
            if (joint == 'mtp_angle_l') or (joint == 'mtp_angle_r'):
                upperBoundsPosition.insert(count, joint, [1.05])
                lowerBoundsPosition.insert(count, joint, [-0.5])                 
            else:              
                if (self.joints.count(joint[:-1] + 'l')) == 1:        
                    ub = max(max(self.Qs_spline[joint[:-1] + 'l']), 
                             max(self.Qs_spline[joint[:-1] + 'r']))
                    lb = min(min(self.Qs_spline[joint[:-1] + 'l']), 
                             min(self.Qs_spline[joint[:-1] + 'r']))                              
                else:
                    ub = max(self.Qs_spline[joint])
                    lb = min(self.Qs_spline[joint])
                r = abs(ub - lb)
                ub = ub + 2*r
                lb = lb - 2*r                        
                upperBoundsPosition.insert(count, joint, [ub])
                lowerBoundsPosition.insert(count, joint, [lb]) 
                # Special cases
                if joint == 'pelvis_tx':
                    upperBoundsPosition[joint] = [2]
                    lowerBoundsPosition[joint] = [0]
                elif joint == 'pelvis_ty':
                    upperBoundsPosition[joint] = [1.1]
                    lowerBoundsPosition[joint] = [0.75]
                elif joint == 'pelvis_tz':
                    upperBoundsPosition[joint] = [0.1]
                    lowerBoundsPosition[joint] = [-0.1]
                elif (joint == 'elbow_flex_l') or (joint == 'elbow_flex_r'):
                    lowerBoundsPosition[joint] = [0]
                elif ((joint == 'arm_add_l') or (joint == 'arm_rot_l') or 
                      (joint == 'arm_add_r') or (joint == 'arm_rot_r')):
                    ub = max(max(self.Qs_spline[joint[:-1] + 'l']), 
                             max(self.Qs_spline[joint[:-1] + 'r']))
                    upperBoundsPosition[joint] = [ub]
                # Running cases
                if self.targetSpeed > 1.33:
                    if joint == 'pelvis_tilt':
                        lowerBoundsPosition[joint] = [-20*np.pi/180]
                    elif joint == 'arm_flex_r':
                        lowerBoundsPosition[joint] = [-50*np.pi/180]
                    elif joint == 'arm_flex_l':
                        lowerBoundsPosition[joint] = [-50*np.pi/180]
                
            # Scaling                       
            s = pd.concat([abs(upperBoundsPosition[joint]), 
                           abs(lowerBoundsPosition[joint])]).max(level=0)
            scalingPosition.insert(count, joint, s)
            lowerBoundsPosition[joint] /= scalingPosition[joint]
            upperBoundsPosition[joint] /= scalingPosition[joint]
            
        # Hard bounds at initial position
        lowerBoundsPositionInitial = lowerBoundsPosition.copy()
        lowerBoundsPositionInitial['pelvis_tx'] = [0]
        upperBoundsPositionInitial = upperBoundsPosition.copy()
        upperBoundsPositionInitial['pelvis_tx'] = [0]
                
        return (upperBoundsPosition, lowerBoundsPosition, scalingPosition,
                upperBoundsPositionInitial, lowerBoundsPositionInitial)
    
    def getBoundsPosition_extended(self):
        self.splineQs()
        upperBoundsPosition = pd.DataFrame()   
        lowerBoundsPosition = pd.DataFrame() 
        scalingPosition = pd.DataFrame() 
        for count, joint in enumerate(self.joints):  
            if (joint == 'mtp_angle_l') or (joint == 'mtp_angle_r'):
                upperBoundsPosition.insert(count, joint, [1.05])
                lowerBoundsPosition.insert(count, joint, [-0.5])                 
            else:              
                if (self.joints.count(joint[:-1] + 'l')) == 1:        
                    ub = max(max(self.Qs_spline[joint[:-1] + 'l']), 
                             max(self.Qs_spline[joint[:-1] + 'r']))
                    lb = min(min(self.Qs_spline[joint[:-1] + 'l']), 
                             min(self.Qs_spline[joint[:-1] + 'r']))                              
                else:
                    ub = max(self.Qs_spline[joint])
                    lb = min(self.Qs_spline[joint])
                r = abs(ub - lb)
                ub = ub + 2*r
                lb = lb - 2*r                        
                upperBoundsPosition.insert(count, joint, [ub])
                lowerBoundsPosition.insert(count, joint, [lb]) 
                # Special cases
                if joint == 'pelvis_tx':
                    upperBoundsPosition[joint] = [2]
                    lowerBoundsPosition[joint] = [0]
                elif joint == 'pelvis_ty':
                    upperBoundsPosition[joint] = [1.1]
                    lowerBoundsPosition[joint] = [0.75]
                elif joint == 'pelvis_tz':
                    upperBoundsPosition[joint] = [0.1]
                    lowerBoundsPosition[joint] = [-0.1]
                elif (joint == 'elbow_flex_l') or (joint == 'elbow_flex_r'):
                    lowerBoundsPosition[joint] = [0]
                elif ((joint == 'arm_add_l') or (joint == 'arm_rot_l') or 
                      (joint == 'arm_add_r') or (joint == 'arm_rot_r')):
                    ub = max(max(self.Qs_spline[joint[:-1] + 'l']), 
                             max(self.Qs_spline[joint[:-1] + 'r']))
                    upperBoundsPosition[joint] = [ub]
                elif joint == 'pelvis_tilt':
                    lowerBoundsPosition[joint] = [-20*np.pi/180]
                # Running cases
                if self.targetSpeed > 1.33:
                    if joint == 'arm_flex_r':
                        lowerBoundsPosition[joint] = [-50*np.pi/180]
                    if joint == 'arm_flex_l':
                        lowerBoundsPosition[joint] = [-50*np.pi/180]
                
            # Scaling                       
            s = pd.concat([abs(upperBoundsPosition[joint]), 
                           abs(lowerBoundsPosition[joint])]).max(level=0)
            scalingPosition.insert(count, joint, s)
            lowerBoundsPosition[joint] /= scalingPosition[joint]
            upperBoundsPosition[joint] /= scalingPosition[joint]
            
        # Hard bounds at initial position
        lowerBoundsPositionInitial = lowerBoundsPosition.copy()
        lowerBoundsPositionInitial['pelvis_tx'] = [0]
        upperBoundsPositionInitial = upperBoundsPosition.copy()
        upperBoundsPositionInitial['pelvis_tx'] = [0]
                
        return (upperBoundsPosition, lowerBoundsPosition, scalingPosition,
                upperBoundsPositionInitial, lowerBoundsPositionInitial) 
    
    def getBoundsVelocity(self):
        self.splineQs()
        upperBoundsVelocity = pd.DataFrame()   
        lowerBoundsVelocity = pd.DataFrame() 
        scalingVelocity = pd.DataFrame() 
        for count, joint in enumerate(self.joints):  
            if (joint == 'mtp_angle_l') or (joint == 'mtp_angle_r'):
                upperBoundsVelocity.insert(count, joint, [13])
                lowerBoundsVelocity.insert(count, joint, [-13]) 
            else:            
                if (self.joints.count(joint[:-1] + 'l')) == 1:        
                    ub = max(max(self.Qdots_spline[joint[:-1] + 'l']), 
                             max(self.Qdots_spline[joint[:-1] + 'r']))
                    lb = min(min(self.Qdots_spline[joint[:-1] + 'l']), 
                             min(self.Qdots_spline[joint[:-1] + 'r']))                              
                else:
                    ub = max(self.Qdots_spline[joint])
                    lb = min(self.Qdots_spline[joint])
                r = abs(ub - lb)
                ub = ub + 3*r
                lb = lb - 3*r                        
                upperBoundsVelocity.insert(count, joint, [ub])
                lowerBoundsVelocity.insert(count, joint, [lb])
    
                # Running cases
                if self.targetSpeed > 1.33:
                    upperBoundsVelocity['pelvis_tx'] = [4]

            # Scaling                       
            s = pd.concat([abs(upperBoundsVelocity[joint]), 
                           abs(lowerBoundsVelocity[joint])]).max(level=0)
            scalingVelocity.insert(count, joint, s)
            upperBoundsVelocity[joint] /= scalingVelocity[joint]
            lowerBoundsVelocity[joint] /= scalingVelocity[joint]

        return upperBoundsVelocity, lowerBoundsVelocity, scalingVelocity
    
    def getBoundsAcceleration(self):
        self.splineQs()
        upperBoundsAcceleration = pd.DataFrame()   
        lowerBoundsAcceleration = pd.DataFrame() 
        scalingAcceleration = pd.DataFrame() 
        for count, joint in enumerate(self.joints):     
            if (joint == 'mtp_angle_l') or (joint == 'mtp_angle_r'):
                upperBoundsAcceleration.insert(count, joint, [500])
                lowerBoundsAcceleration.insert(count, joint, [-500]) 
            else:           
                if (self.joints.count(joint[:-1] + 'l')) == 1:        
                    ub = max(max(self.Qdotdots_spline[joint[:-1] + 'l']), 
                             max(self.Qdotdots_spline[joint[:-1] + 'r']))
                    lb = min(min(self.Qdotdots_spline[joint[:-1] + 'l']), 
                             min(self.Qdotdots_spline[joint[:-1] + 'r']))                              
                else:
                    ub = max(self.Qdotdots_spline[joint])
                    lb = min(self.Qdotdots_spline[joint])
                r = abs(ub - lb)
                ub = ub + 3*r
                lb = lb - 3*r                        
                upperBoundsAcceleration.insert(count, joint, [ub])
                lowerBoundsAcceleration.insert(count, joint, [lb])   
            
            # Scaling                       
            s = pd.concat([abs(upperBoundsAcceleration[joint]), 
                           abs(lowerBoundsAcceleration[joint])]).max(level=0)
            scalingAcceleration.insert(count, joint, s)
            upperBoundsAcceleration[joint] /= scalingAcceleration[joint]
            lowerBoundsAcceleration[joint] /= scalingAcceleration[joint]

        return (upperBoundsAcceleration, lowerBoundsAcceleration, 
                scalingAcceleration)
    
    def getBoundsActivation(self):
        lb = [0.05] 
        lb_vec = lb * len(self.muscles)
        ub = [1]
        ub_vec = ub * len(self.muscles)
        s = [1]
        s_vec = s * len(self.muscles)
        upperBoundsActivation = pd.DataFrame([ub_vec], columns=self.muscles)   
        lowerBoundsActivation = pd.DataFrame([lb_vec], columns=self.muscles) 
        scalingActivation = pd.DataFrame([s_vec], columns=self.muscles)
        upperBoundsActivation = upperBoundsActivation.div(scalingActivation)
        lowerBoundsActivation = lowerBoundsActivation.div(scalingActivation)
        for count, muscle in enumerate(self.muscles):
            upperBoundsActivation.insert(count + len(self.muscles), 
                                         muscle[:-1] + 'l', ub)
            lowerBoundsActivation.insert(count + len(self.muscles), 
                                         muscle[:-1] + 'l', lb)  

            # Scaling                       
            scalingActivation.insert(count + len(self.muscles), 
                                     muscle[:-1] + 'l', s)  
            upperBoundsActivation[
                    muscle[:-1] + 'l'] /= scalingActivation[muscle[:-1] + 'l']
            lowerBoundsActivation[
                    muscle[:-1] + 'l'] /= scalingActivation[muscle[:-1] + 'l']
        
        return upperBoundsActivation, lowerBoundsActivation, scalingActivation
    
    def getBoundsForce(self):
        lb = [0] 
        lb_vec = lb * len(self.muscles)
        ub = [5]
        ub_vec = ub * len(self.muscles)
        s = max([abs(lbi) for lbi in lb], [abs(ubi) for ubi in ub])
        s_vec = s * len(self.muscles)
        upperBoundsForce = pd.DataFrame([ub_vec], columns=self.muscles)   
        lowerBoundsForce = pd.DataFrame([lb_vec], columns=self.muscles) 
        scalingForce = pd.DataFrame([s_vec], columns=self.muscles)
        upperBoundsForce = upperBoundsForce.div(scalingForce)
        lowerBoundsForce = lowerBoundsForce.div(scalingForce)
        for count, muscle in enumerate(self.muscles):
            upperBoundsForce.insert(count + len(self.muscles), 
                                    muscle[:-1] + 'l', ub)
            lowerBoundsForce.insert(count + len(self.muscles), 
                                    muscle[:-1] + 'l', lb)  

            # Scaling                       
            scalingForce.insert(count + len(self.muscles), 
                                         muscle[:-1] + 'l', s)   
            upperBoundsForce[
                    muscle[:-1] + 'l'] /= scalingForce[muscle[:-1] + 'l']
            lowerBoundsForce[
                    muscle[:-1] + 'l'] /= scalingForce[muscle[:-1] + 'l']
        
        return upperBoundsForce, lowerBoundsForce, scalingForce
    
    def getBoundsActivationDerivative(self):
        activationTimeConstant = 0.015
        deactivationTimeConstant = 0.06
        lb = [-1 / deactivationTimeConstant] 
        lb_vec = lb * len(self.muscles)
        ub = [1 / activationTimeConstant]
        ub_vec = ub * len(self.muscles)
        s = [100]
        s_vec = s * len(self.muscles)
        upperBoundsActivationDerivative = pd.DataFrame([ub_vec], 
                                                       columns=self.muscles)   
        lowerBoundsActivationDerivative = pd.DataFrame([lb_vec], 
                                                       columns=self.muscles) 
        scalingActivationDerivative = pd.DataFrame([s_vec], 
                                                   columns=self.muscles)
        upperBoundsActivationDerivative = upperBoundsActivationDerivative.div(
                scalingActivationDerivative)
        lowerBoundsActivationDerivative = lowerBoundsActivationDerivative.div(
                scalingActivationDerivative)
        for count, muscle in enumerate(self.muscles):
            upperBoundsActivationDerivative.insert(count + len(self.muscles), 
                                                   muscle[:-1] + 'l', ub)
            lowerBoundsActivationDerivative.insert(count + len(self.muscles), 
                                                   muscle[:-1] + 'l', lb) 

            # Scaling                       
            scalingActivationDerivative.insert(count + len(self.muscles), 
                                               muscle[:-1] + 'l', s)  
            upperBoundsActivationDerivative[muscle[:-1] + 'l'] /= (
                    scalingActivationDerivative[muscle[:-1] + 'l'])
            lowerBoundsActivationDerivative[muscle[:-1] + 'l'] /= (
                    scalingActivationDerivative[muscle[:-1] + 'l'])             
        
        return (upperBoundsActivationDerivative, 
                lowerBoundsActivationDerivative, scalingActivationDerivative)
    
    def getBoundsForceDerivative(self):
        lb = [-100] 
        lb_vec = lb * len(self.muscles)
        ub = [100]
        ub_vec = ub * len(self.muscles)
        s = [100]
        s_vec = s * len(self.muscles)
        upperBoundsForceDerivative = pd.DataFrame([ub_vec], 
                                                  columns=self.muscles)   
        lowerBoundsForceDerivative = pd.DataFrame([lb_vec], 
                                                  columns=self.muscles) 
        scalingForceDerivative = pd.DataFrame([s_vec], 
                                                   columns=self.muscles)
        upperBoundsForceDerivative = upperBoundsForceDerivative.div(
                scalingForceDerivative)
        lowerBoundsForceDerivative = lowerBoundsForceDerivative.div(
                scalingForceDerivative)
        for count, muscle in enumerate(self.muscles):
            upperBoundsForceDerivative.insert(count + len(self.muscles), 
                                              muscle[:-1] + 'l', ub)
            lowerBoundsForceDerivative.insert(count + len(self.muscles), 
                                              muscle[:-1] + 'l', lb)   
            
            # Scaling                       
            scalingForceDerivative.insert(count + len(self.muscles), 
                                               muscle[:-1] + 'l', s)  
            upperBoundsForceDerivative[muscle[:-1] + 'l'] /= (
                    scalingForceDerivative[muscle[:-1] + 'l'])
            lowerBoundsForceDerivative[muscle[:-1] + 'l'] /= (
                    scalingForceDerivative[muscle[:-1] + 'l']) 
        
        return (upperBoundsForceDerivative, lowerBoundsForceDerivative, 
                scalingForceDerivative)
    
    def getBoundsArmExcitation(self):
        lb = [-1] 
        lb_vec = lb * len(self.armJoints)
        ub = [1]
        ub_vec = ub * len(self.armJoints)
        s = [150]
        s_vec = s * len(self.armJoints)
        upperBoundsArmExcitation = pd.DataFrame([ub_vec], 
                                                columns=self.armJoints)   
        lowerBoundsArmExcitation = pd.DataFrame([lb_vec], 
                                                columns=self.armJoints)            
        scalingArmExcitation = pd.DataFrame([s_vec], columns=self.armJoints)
        
        return (upperBoundsArmExcitation, lowerBoundsArmExcitation,
                scalingArmExcitation)
    
    def getBoundsArmActivation(self):
        lb = [-1] 
        lb_vec = lb * len(self.armJoints)
        ub = [1]
        ub_vec = ub * len(self.armJoints)
        s = [150]
        s_vec = s * len(self.armJoints)
        upperBoundsArmActivation = pd.DataFrame([ub_vec], 
                                                columns=self.armJoints)   
        lowerBoundsArmActivation = pd.DataFrame([lb_vec], 
                                                columns=self.armJoints) 
        scalingArmActivation = pd.DataFrame([s_vec], columns=self.armJoints)                  
        
        return (upperBoundsArmActivation, lowerBoundsArmActivation, 
                scalingArmActivation)
        
    def getBoundsMtpExcitation(self):
        lb = [-1] 
        lb_vec = lb * len(self.mtpJoints)
        ub = [1]
        ub_vec = ub * len(self.mtpJoints)
        s = [30]
        s_vec = s * len(self.mtpJoints)
        upperBoundsMtpExcitation = pd.DataFrame([ub_vec], 
                                                columns=self.mtpJoints)   
        lowerBoundsMtpExcitation = pd.DataFrame([lb_vec], 
                                                columns=self.mtpJoints)            
        scalingMtpExcitation = pd.DataFrame([s_vec], columns=self.mtpJoints)
        
        return (upperBoundsMtpExcitation, lowerBoundsMtpExcitation,
                scalingMtpExcitation)
    
    def getBoundsMtpActivation(self):
        lb = [-1] 
        lb_vec = lb * len(self.mtpJoints)
        ub = [1]
        ub_vec = ub * len(self.mtpJoints)
        s = [30]
        s_vec = s * len(self.mtpJoints)
        upperBoundsMtpActivation = pd.DataFrame([ub_vec], 
                                                columns=self.mtpJoints)   
        lowerBoundsMtpActivation = pd.DataFrame([lb_vec], 
                                                columns=self.mtpJoints) 
        scalingMtpActivation = pd.DataFrame([s_vec], columns=self.mtpJoints)                  
        
        return (upperBoundsMtpActivation, lowerBoundsMtpActivation, 
                scalingMtpActivation)
    
    def getBoundsFinalTime(self):
        upperBoundsFinalTime = pd.DataFrame([1], columns=['time'])   
        lowerBoundsFinalTime = pd.DataFrame([0.1], columns=['time'])  
        
        return upperBoundsFinalTime, lowerBoundsFinalTime
    
# %%
class bounds_2D:
    
    def __init__(self, joints, muscles):
        self.joints = joints
        self.muscles = muscles       
            
    def getBoundsPosition(self):
        upperBoundsPosition = pd.DataFrame()   
        lowerBoundsPosition = pd.DataFrame() 
        scalingPosition = pd.DataFrame() 
        
        upperBoundsPosition['pelvis_tilt'] = [30 * np.pi / 180]
        upperBoundsPosition['pelvis_tx'] = [2]
        upperBoundsPosition['pelvis_ty'] = [1.1]
        upperBoundsPosition['hip_flexion_l'] = [70 * np.pi / 180]
        upperBoundsPosition['hip_flexion_r'] = [70 * np.pi / 180]
        upperBoundsPosition['knee_angle_l'] = [10 * np.pi / 180]
        upperBoundsPosition['knee_angle_r'] = [10 * np.pi / 180]
        upperBoundsPosition['ankle_angle_l'] = [45 * np.pi / 180]
        upperBoundsPosition['ankle_angle_r'] = [45 * np.pi / 180]
        upperBoundsPosition['mtp_angle_l'] = [30 * np.pi / 180]
        upperBoundsPosition['mtp_angle_r'] = [30 * np.pi / 180]
        upperBoundsPosition['lumbar_extension'] = [30 * np.pi / 180]
        
        lowerBoundsPosition['pelvis_tilt'] = [-30 * np.pi / 180]
        lowerBoundsPosition['pelvis_tx'] = [-2]
        lowerBoundsPosition['pelvis_ty'] = [0.75]
        lowerBoundsPosition['hip_flexion_l'] = [-70 * np.pi / 180]
        lowerBoundsPosition['hip_flexion_r'] = [-70 * np.pi / 180]
        lowerBoundsPosition['knee_angle_l'] = [-120 * np.pi / 180]
        lowerBoundsPosition['knee_angle_r'] = [-120 * np.pi / 180]
        lowerBoundsPosition['ankle_angle_l'] = [-45 * np.pi / 180]
        lowerBoundsPosition['ankle_angle_r'] = [-45 * np.pi / 180]
        lowerBoundsPosition['mtp_angle_l'] = [-60 * np.pi / 180]
        lowerBoundsPosition['mtp_angle_r'] = [-60 * np.pi / 180]
        lowerBoundsPosition['lumbar_extension'] = [-30 * np.pi / 180]
        
        for count, joint in enumerate(self.joints):         
            # Scaling                       
            s = pd.concat([abs(upperBoundsPosition[joint]), 
                           abs(lowerBoundsPosition[joint])]).max(level=0)
            scalingPosition.insert(count, joint, s)
            lowerBoundsPosition[joint] /= scalingPosition[joint]
            upperBoundsPosition[joint] /= scalingPosition[joint]
            
        # Hard bounds at initial position
        lowerBoundsPositionInitial = lowerBoundsPosition.copy()
        lowerBoundsPositionInitial['pelvis_tx'] = [0]
        upperBoundsPositionInitial = upperBoundsPosition.copy()
        upperBoundsPositionInitial['pelvis_tx'] = [0]
                
        return (upperBoundsPosition, lowerBoundsPosition, scalingPosition,
                upperBoundsPositionInitial, lowerBoundsPositionInitial) 
    
    def getBoundsVelocity(self):
        upperBoundsVelocity = pd.DataFrame()   
        lowerBoundsVelocity = pd.DataFrame() 
        scalingVelocity = pd.DataFrame()         
        
        upperBoundsVelocity['pelvis_tilt'] = [5]
        upperBoundsVelocity['pelvis_tx'] = [2]
        upperBoundsVelocity['pelvis_ty'] = [2]
        upperBoundsVelocity['hip_flexion_l'] = [10]
        upperBoundsVelocity['hip_flexion_r'] = [10]
        upperBoundsVelocity['knee_angle_l'] = [20]
        upperBoundsVelocity['knee_angle_r'] = [20]
        upperBoundsVelocity['ankle_angle_l'] = [10]
        upperBoundsVelocity['ankle_angle_r'] = [10]
        upperBoundsVelocity['mtp_angle_l'] = [20]
        upperBoundsVelocity['mtp_angle_r'] = [20]
        upperBoundsVelocity['lumbar_extension'] = [5]
        
        lowerBoundsVelocity['pelvis_tilt'] = [-5]
        lowerBoundsVelocity['pelvis_tx'] = [0]
        lowerBoundsVelocity['pelvis_ty'] = [-2]
        lowerBoundsVelocity['hip_flexion_l'] = [-10]
        lowerBoundsVelocity['hip_flexion_r'] = [-10]
        lowerBoundsVelocity['knee_angle_l'] = [-20]
        lowerBoundsVelocity['knee_angle_r'] = [-20]
        lowerBoundsVelocity['ankle_angle_l'] = [-10]
        lowerBoundsVelocity['ankle_angle_r'] = [-10]
        lowerBoundsVelocity['mtp_angle_l'] = [-20]
        lowerBoundsVelocity['mtp_angle_r'] = [-20]
        lowerBoundsVelocity['lumbar_extension'] = [-5]  
        
        for count, joint in enumerate(self.joints):         
            # Scaling                       
            s = pd.concat([abs(upperBoundsVelocity[joint]), 
                           abs(lowerBoundsVelocity[joint])]).max(level=0)
            scalingVelocity.insert(count, joint, s)
            lowerBoundsVelocity[joint] /= scalingVelocity[joint]
            upperBoundsVelocity[joint] /= scalingVelocity[joint]
            
        return upperBoundsVelocity, lowerBoundsVelocity, scalingVelocity
    
    def getBoundsAcceleration(self):
        upperBoundsAcceleration = pd.DataFrame()   
        lowerBoundsAcceleration = pd.DataFrame() 
        scalingAcceleration = pd.DataFrame() 
        
        upperBoundsAcceleration['pelvis_tilt'] = [50]
        upperBoundsAcceleration['pelvis_tx'] = [30]
        upperBoundsAcceleration['pelvis_ty'] = [30]
        upperBoundsAcceleration['hip_flexion_l'] = [150]
        upperBoundsAcceleration['hip_flexion_r'] = [150]
        upperBoundsAcceleration['knee_angle_l'] = [250]
        upperBoundsAcceleration['knee_angle_r'] = [250]
        upperBoundsAcceleration['ankle_angle_l'] = [250]
        upperBoundsAcceleration['ankle_angle_r'] = [250]
        upperBoundsAcceleration['mtp_angle_l'] = [250]
        upperBoundsAcceleration['mtp_angle_r'] = [250]
        upperBoundsAcceleration['lumbar_extension'] = [50]
        
        lowerBoundsAcceleration['pelvis_tilt'] = [-50]
        lowerBoundsAcceleration['pelvis_tx'] = [-30]
        lowerBoundsAcceleration['pelvis_ty'] = [-30]
        lowerBoundsAcceleration['hip_flexion_l'] = [-150]
        lowerBoundsAcceleration['hip_flexion_r'] = [-150]
        lowerBoundsAcceleration['knee_angle_l'] = [-250]
        lowerBoundsAcceleration['knee_angle_r'] = [-250]
        lowerBoundsAcceleration['ankle_angle_l'] = [-250]
        lowerBoundsAcceleration['ankle_angle_r'] = [-250]
        lowerBoundsAcceleration['mtp_angle_l'] = [-250]
        lowerBoundsAcceleration['mtp_angle_r'] = [-250]
        lowerBoundsAcceleration['lumbar_extension'] = [-50]
        
        for count, joint in enumerate(self.joints):             
            # Scaling                       
            s = pd.concat([abs(upperBoundsAcceleration[joint]), 
                           abs(lowerBoundsAcceleration[joint])]).max(level=0)
            scalingAcceleration.insert(count, joint, s)
            upperBoundsAcceleration[joint] /= scalingAcceleration[joint]
            lowerBoundsAcceleration[joint] /= scalingAcceleration[joint]

        return (upperBoundsAcceleration, lowerBoundsAcceleration, 
                scalingAcceleration)
    
    def getBoundsActivation(self):
        lb = [0.05] 
        lb_vec = lb * len(self.muscles)
        ub = [1]
        ub_vec = ub * len(self.muscles)
        s = [1]
        s_vec = s * len(self.muscles)
        upperBoundsActivation = pd.DataFrame([ub_vec], columns=self.muscles)   
        lowerBoundsActivation = pd.DataFrame([lb_vec], columns=self.muscles) 
        scalingActivation = pd.DataFrame([s_vec], columns=self.muscles)
        upperBoundsActivation = upperBoundsActivation.div(scalingActivation)
        lowerBoundsActivation = lowerBoundsActivation.div(scalingActivation)
        for count, muscle in enumerate(self.muscles):
            upperBoundsActivation.insert(count + len(self.muscles), 
                                         muscle[:-1] + 'l', ub)
            lowerBoundsActivation.insert(count + len(self.muscles), 
                                         muscle[:-1] + 'l', lb)  

            # Scaling                       
            scalingActivation.insert(count + len(self.muscles), 
                                     muscle[:-1] + 'l', s)  
            upperBoundsActivation[
                    muscle[:-1] + 'l'] /= scalingActivation[muscle[:-1] + 'l']
            lowerBoundsActivation[
                    muscle[:-1] + 'l'] /= scalingActivation[muscle[:-1] + 'l']
        
        return upperBoundsActivation, lowerBoundsActivation, scalingActivation
    
    def getBoundsForce(self):
        lb = [0] 
        lb_vec = lb * len(self.muscles)
        ub = [5]
        ub_vec = ub * len(self.muscles)
        s = max([abs(lbi) for lbi in lb], [abs(ubi) for ubi in ub])
        s_vec = s * len(self.muscles)
        upperBoundsForce = pd.DataFrame([ub_vec], columns=self.muscles)   
        lowerBoundsForce = pd.DataFrame([lb_vec], columns=self.muscles) 
        scalingForce = pd.DataFrame([s_vec], columns=self.muscles)
        upperBoundsForce = upperBoundsForce.div(scalingForce)
        lowerBoundsForce = lowerBoundsForce.div(scalingForce)
        for count, muscle in enumerate(self.muscles):
            upperBoundsForce.insert(count + len(self.muscles), 
                                    muscle[:-1] + 'l', ub)
            lowerBoundsForce.insert(count + len(self.muscles), 
                                    muscle[:-1] + 'l', lb)  

            # Scaling                       
            scalingForce.insert(count + len(self.muscles), 
                                         muscle[:-1] + 'l', s)   
            upperBoundsForce[
                    muscle[:-1] + 'l'] /= scalingForce[muscle[:-1] + 'l']
            lowerBoundsForce[
                    muscle[:-1] + 'l'] /= scalingForce[muscle[:-1] + 'l']
        
        return upperBoundsForce, lowerBoundsForce, scalingForce
    
    def getBoundsActivationDerivative(self):
        activationTimeConstant = 0.015
        deactivationTimeConstant = 0.06
        lb = [-1 / deactivationTimeConstant] 
        lb_vec = lb * len(self.muscles)
        ub = [1 / activationTimeConstant]
        ub_vec = ub * len(self.muscles)
        s = [100]
        s_vec = s * len(self.muscles)
        upperBoundsActivationDerivative = pd.DataFrame([ub_vec], 
                                                       columns=self.muscles)   
        lowerBoundsActivationDerivative = pd.DataFrame([lb_vec], 
                                                       columns=self.muscles) 
        scalingActivationDerivative = pd.DataFrame([s_vec], 
                                                   columns=self.muscles)
        upperBoundsActivationDerivative = upperBoundsActivationDerivative.div(
                scalingActivationDerivative)
        lowerBoundsActivationDerivative = lowerBoundsActivationDerivative.div(
                scalingActivationDerivative)
        for count, muscle in enumerate(self.muscles):
            upperBoundsActivationDerivative.insert(count + len(self.muscles), 
                                                   muscle[:-1] + 'l', ub)
            lowerBoundsActivationDerivative.insert(count + len(self.muscles), 
                                                   muscle[:-1] + 'l', lb) 

            # Scaling                       
            scalingActivationDerivative.insert(count + len(self.muscles), 
                                               muscle[:-1] + 'l', s)  
            upperBoundsActivationDerivative[muscle[:-1] + 'l'] /= (
                    scalingActivationDerivative[muscle[:-1] + 'l'])
            lowerBoundsActivationDerivative[muscle[:-1] + 'l'] /= (
                    scalingActivationDerivative[muscle[:-1] + 'l'])             
        
        return (upperBoundsActivationDerivative, 
                lowerBoundsActivationDerivative, scalingActivationDerivative)
    
    def getBoundsForceDerivative(self):
        lb = [-100] 
        lb_vec = lb * len(self.muscles)
        ub = [100]
        ub_vec = ub * len(self.muscles)
        s = [100]
        s_vec = s * len(self.muscles)
        upperBoundsForceDerivative = pd.DataFrame([ub_vec], 
                                                  columns=self.muscles)   
        lowerBoundsForceDerivative = pd.DataFrame([lb_vec], 
                                                  columns=self.muscles) 
        scalingForceDerivative = pd.DataFrame([s_vec], 
                                                   columns=self.muscles)
        upperBoundsForceDerivative = upperBoundsForceDerivative.div(
                scalingForceDerivative)
        lowerBoundsForceDerivative = lowerBoundsForceDerivative.div(
                scalingForceDerivative)
        for count, muscle in enumerate(self.muscles):
            upperBoundsForceDerivative.insert(count + len(self.muscles), 
                                              muscle[:-1] + 'l', ub)
            lowerBoundsForceDerivative.insert(count + len(self.muscles), 
                                              muscle[:-1] + 'l', lb)   
            
            # Scaling                       
            scalingForceDerivative.insert(count + len(self.muscles), 
                                               muscle[:-1] + 'l', s)  
            upperBoundsForceDerivative[muscle[:-1] + 'l'] /= (
                    scalingForceDerivative[muscle[:-1] + 'l'])
            lowerBoundsForceDerivative[muscle[:-1] + 'l'] /= (
                    scalingForceDerivative[muscle[:-1] + 'l']) 
        
        return (upperBoundsForceDerivative, lowerBoundsForceDerivative, 
                scalingForceDerivative)
    
    def getBoundsLumbarExcitation(self, lumbarJoints):
        lb = [-1] 
        lb_vec = lb * len(lumbarJoints)
        ub = [1]
        ub_vec = ub * len(lumbarJoints)
        s = [150]
        s_vec = s * len(lumbarJoints)
        upperBoundsLumbarExcitation = pd.DataFrame([ub_vec], 
                                                columns=lumbarJoints)   
        lowerBoundsLumbarExcitation = pd.DataFrame([lb_vec], 
                                                columns=lumbarJoints)            
        scalingLumbarExcitation = pd.DataFrame([s_vec], columns=lumbarJoints)
        
        return (upperBoundsLumbarExcitation, lowerBoundsLumbarExcitation,
                scalingLumbarExcitation)
    
    def getBoundsLumbarActivation(self, lumbarJoints):
        lb = [-1] 
        lb_vec = lb * len(lumbarJoints)
        ub = [1]
        ub_vec = ub * len(lumbarJoints)
        s = [150]
        s_vec = s * len(lumbarJoints)
        upperBoundsLumbarActivation = pd.DataFrame([ub_vec], 
                                                columns=lumbarJoints)   
        lowerBoundsLumbarActivation = pd.DataFrame([lb_vec], 
                                                columns=lumbarJoints) 
        scalingLumbarActivation = pd.DataFrame([s_vec], columns=lumbarJoints)                  
        
        return (upperBoundsLumbarActivation, lowerBoundsLumbarActivation, 
                scalingLumbarActivation)
        
    def getBoundsMtpExcitation(self, mtpJoints):
        lb = [-1] 
        lb_vec = lb * len(mtpJoints)
        ub = [1]
        ub_vec = ub * len(mtpJoints)
        s = [100]
        s_vec = s * len(mtpJoints)
        upperBoundsMtpExcitation = pd.DataFrame([ub_vec], 
                                                columns=mtpJoints)   
        lowerBoundsMtpExcitation = pd.DataFrame([lb_vec], 
                                                columns=mtpJoints)            
        scalingMtpExcitation = pd.DataFrame([s_vec], columns=mtpJoints)
        
        return (upperBoundsMtpExcitation, lowerBoundsMtpExcitation,
                scalingMtpExcitation)
    
    def getBoundsMtpActivation(self, mtpJoints):
        lb = [-1] 
        lb_vec = lb * len(mtpJoints)
        ub = [1]
        ub_vec = ub * len(mtpJoints)
        s = [100]
        s_vec = s * len(mtpJoints)
        upperBoundsMtpActivation = pd.DataFrame([ub_vec], 
                                                columns=mtpJoints)   
        lowerBoundsMtpActivation = pd.DataFrame([lb_vec], 
                                                columns=mtpJoints) 
        scalingMtpActivation = pd.DataFrame([s_vec], columns=mtpJoints)                  
        
        return (upperBoundsMtpActivation, lowerBoundsMtpActivation, 
                scalingMtpActivation)
    
    def getBoundsFinalTime(self):
        upperBoundsFinalTime = pd.DataFrame([1], columns=['time'])   
        lowerBoundsFinalTime = pd.DataFrame([0.1], columns=['time'])  
        
        return upperBoundsFinalTime, lowerBoundsFinalTime
    
    def getScalingContacts(self):
        
        scalingGRF = 1000
        scalingGRM = 1000
        
        return scalingGRF, scalingGRM
    
    # %%
class bounds_2D_uniformedScaling:
    
    def __init__(self, joints, rotationalJoints, translationalJoints, muscles):
        self.joints = joints
        self.rotationalJoints = rotationalJoints
        self.translationalJoints = translationalJoints
        self.muscles = muscles       
            
    def getBoundsPosition(self):
        upperBoundsPosition = pd.DataFrame()   
        lowerBoundsPosition = pd.DataFrame() 
        scalingPosition = pd.DataFrame() 
        
        upperBoundsPosition['pelvis_tilt'] = [30 * np.pi / 180]
        upperBoundsPosition['pelvis_tx'] = [5]
        upperBoundsPosition['pelvis_ty'] = [1.25]
        upperBoundsPosition['hip_flexion_l'] = [70 * np.pi / 180]
        upperBoundsPosition['hip_flexion_r'] = [70 * np.pi / 180]
        upperBoundsPosition['knee_angle_l'] = [10 * np.pi / 180]
        upperBoundsPosition['knee_angle_r'] = [10 * np.pi / 180]
        upperBoundsPosition['ankle_angle_l'] = [45 * np.pi / 180]
        upperBoundsPosition['ankle_angle_r'] = [45 * np.pi / 180]
        if 'mtp_angle_l' in self.joints:
            upperBoundsPosition['mtp_angle_l'] = [60 * np.pi / 180]
            upperBoundsPosition['mtp_angle_r'] = [60 * np.pi / 180]
        upperBoundsPosition['lumbar_extension'] = [45 * np.pi / 180]
        
        lowerBoundsPosition['pelvis_tilt'] = [-30 * np.pi / 180]
        lowerBoundsPosition['pelvis_tx'] = [-5]
        lowerBoundsPosition['pelvis_ty'] = [0.75]
        lowerBoundsPosition['hip_flexion_l'] = [-70 * np.pi / 180]
        lowerBoundsPosition['hip_flexion_r'] = [-70 * np.pi / 180]
        lowerBoundsPosition['knee_angle_l'] = [-120 * np.pi / 180]
        lowerBoundsPosition['knee_angle_r'] = [-120 * np.pi / 180]
        lowerBoundsPosition['ankle_angle_l'] = [-45 * np.pi / 180]
        lowerBoundsPosition['ankle_angle_r'] = [-45 * np.pi / 180]
        if 'mtp_angle_l' in self.joints:
            lowerBoundsPosition['mtp_angle_l'] = [-60 * np.pi / 180]
            lowerBoundsPosition['mtp_angle_r'] = [-60 * np.pi / 180]
        lowerBoundsPosition['lumbar_extension'] = [-45 * np.pi / 180]        
        
        # Rotational joints
        # The goal is to use the same scaling for each joint
        max_rotationalJoints = np.zeros(len(self.rotationalJoints))
        for count, rotationalJoint in enumerate(self.rotationalJoints): 
            max_rotationalJoints[count] = pd.concat(
                    [abs(upperBoundsPosition[rotationalJoint]),
                     abs(lowerBoundsPosition[rotationalJoint])]).max(level=0)[0]
        maxAll_rotationalJoints = np.max(max_rotationalJoints)      
            
        # Translational joints
        # The goal is to use the same scaling for each joint
        max_translationalJoints = np.zeros(len(self.translationalJoints))
        for count, translationalJoint in enumerate(self.translationalJoints): 
            max_translationalJoints[count] = pd.concat(
                    [abs(upperBoundsPosition[translationalJoint]),
                     abs(lowerBoundsPosition[translationalJoint])]).max(level=0)[0]
        maxAll_translationalJoints = np.max(max_translationalJoints)    
        
        for count, joint in enumerate(self.joints):         
            # Scaling            
            if joint in self.rotationalJoints:
                scalingPosition.insert(count, joint, [maxAll_rotationalJoints])
            elif joint in self.translationalJoints:
                scalingPosition.insert(count, joint, [maxAll_translationalJoints])
            else:
                 raise ValueError('Unknown joint')   
            lowerBoundsPosition[joint] /= scalingPosition[joint]
            upperBoundsPosition[joint] /= scalingPosition[joint]
            
        # Hard bounds at initial position
        lowerBoundsPositionInitial = lowerBoundsPosition.copy()
        lowerBoundsPositionInitial['pelvis_tx'] = [0]
        upperBoundsPositionInitial = upperBoundsPosition.copy()
        upperBoundsPositionInitial['pelvis_tx'] = [0]
                
        return (upperBoundsPosition, lowerBoundsPosition, scalingPosition,
                upperBoundsPositionInitial, lowerBoundsPositionInitial) 
    
    def getBoundsVelocity(self):
        upperBoundsVelocity = pd.DataFrame()   
        lowerBoundsVelocity = pd.DataFrame() 
        scalingVelocity = pd.DataFrame()         
        
        upperBoundsVelocity['pelvis_tilt'] = [5]
        upperBoundsVelocity['pelvis_tx'] = [2]
        upperBoundsVelocity['pelvis_ty'] = [2]
        upperBoundsVelocity['hip_flexion_l'] = [10]
        upperBoundsVelocity['hip_flexion_r'] = [10]
        upperBoundsVelocity['knee_angle_l'] = [20]
        upperBoundsVelocity['knee_angle_r'] = [20]
        upperBoundsVelocity['ankle_angle_l'] = [10]
        upperBoundsVelocity['ankle_angle_r'] = [10]
        if 'mtp_angle_l' in self.joints:
            upperBoundsVelocity['mtp_angle_l'] = [20]
            upperBoundsVelocity['mtp_angle_r'] = [20]
        upperBoundsVelocity['lumbar_extension'] = [5]
        
        lowerBoundsVelocity['pelvis_tilt'] = [-5]
        lowerBoundsVelocity['pelvis_tx'] = [-2]
        lowerBoundsVelocity['pelvis_ty'] = [-2]
        lowerBoundsVelocity['hip_flexion_l'] = [-10]
        lowerBoundsVelocity['hip_flexion_r'] = [-10]
        lowerBoundsVelocity['knee_angle_l'] = [-20]
        lowerBoundsVelocity['knee_angle_r'] = [-20]
        lowerBoundsVelocity['ankle_angle_l'] = [-10]
        lowerBoundsVelocity['ankle_angle_r'] = [-10]
        if 'mtp_angle_l' in self.joints:
            lowerBoundsVelocity['mtp_angle_l'] = [-20]
            lowerBoundsVelocity['mtp_angle_r'] = [-20]
        lowerBoundsVelocity['lumbar_extension'] = [-5]  
        
        # Rotational joints
        # The goal is to use the same scaling for each joint
        max_rotationalJoints = np.zeros(len(self.rotationalJoints))
        for count, rotationalJoint in enumerate(self.rotationalJoints): 
            max_rotationalJoints[count] = pd.concat(
                    [abs(upperBoundsVelocity[rotationalJoint]),
                     abs(lowerBoundsVelocity[rotationalJoint])]).max(level=0)[0]
        maxAll_rotationalJoints = np.max(max_rotationalJoints)       
            
        # Translational joints
        # The goal is to use the same scaling for each joint
        max_translationalJoints = np.zeros(len(self.translationalJoints))
        for count, translationalJoint in enumerate(self.translationalJoints): 
            max_translationalJoints[count] = pd.concat(
                    [abs(upperBoundsVelocity[translationalJoint]),
                     abs(lowerBoundsVelocity[translationalJoint])]).max(level=0)[0]
        maxAll_translationalJoints = np.max(max_translationalJoints) 
        
        for count, joint in enumerate(self.joints):         
            # Scaling            
            if joint in self.rotationalJoints:
                scalingVelocity.insert(count, joint, [maxAll_rotationalJoints])
            elif joint in self.translationalJoints:
                scalingVelocity.insert(count, joint, [maxAll_translationalJoints])
            else:
                 raise ValueError('Unknown joint')   
            lowerBoundsVelocity[joint] /= scalingVelocity[joint]
            upperBoundsVelocity[joint] /= scalingVelocity[joint]
            
        return upperBoundsVelocity, lowerBoundsVelocity, scalingVelocity
    
    def getBoundsAcceleration(self):
        upperBoundsAcceleration = pd.DataFrame()   
        lowerBoundsAcceleration = pd.DataFrame() 
        scalingAcceleration = pd.DataFrame() 
        
        upperBoundsAcceleration['pelvis_tilt'] = [50]
        upperBoundsAcceleration['pelvis_tx'] = [30]
        upperBoundsAcceleration['pelvis_ty'] = [30]
        upperBoundsAcceleration['hip_flexion_l'] = [150]
        upperBoundsAcceleration['hip_flexion_r'] = [150]
        upperBoundsAcceleration['knee_angle_l'] = [250]
        upperBoundsAcceleration['knee_angle_r'] = [250]
        upperBoundsAcceleration['ankle_angle_l'] = [250]
        upperBoundsAcceleration['ankle_angle_r'] = [250]
        if 'mtp_angle_l' in self.joints:
            upperBoundsAcceleration['mtp_angle_l'] = [250]
            upperBoundsAcceleration['mtp_angle_r'] = [250]
        upperBoundsAcceleration['lumbar_extension'] = [50]
        
        lowerBoundsAcceleration['pelvis_tilt'] = [-50]
        lowerBoundsAcceleration['pelvis_tx'] = [-30]
        lowerBoundsAcceleration['pelvis_ty'] = [-30]
        lowerBoundsAcceleration['hip_flexion_l'] = [-150]
        lowerBoundsAcceleration['hip_flexion_r'] = [-150]
        lowerBoundsAcceleration['knee_angle_l'] = [-250]
        lowerBoundsAcceleration['knee_angle_r'] = [-250]
        lowerBoundsAcceleration['ankle_angle_l'] = [-250]
        lowerBoundsAcceleration['ankle_angle_r'] = [-250]
        if 'mtp_angle_l' in self.joints:
            lowerBoundsAcceleration['mtp_angle_l'] = [-250]
            lowerBoundsAcceleration['mtp_angle_r'] = [-250]
        lowerBoundsAcceleration['lumbar_extension'] = [-50]
        
        # Rotational joints
        # The goal is to use the same scaling for each joint
        max_rotationalJoints = np.zeros(len(self.rotationalJoints))
        for count, rotationalJoint in enumerate(self.rotationalJoints): 
            max_rotationalJoints[count] = pd.concat(
                    [abs(upperBoundsAcceleration[rotationalJoint]),
                     abs(lowerBoundsAcceleration[rotationalJoint])]).max(level=0)[0]
        maxAll_rotationalJoints = np.max(max_rotationalJoints)          
            
        # Translational joints
        # The goal is to use the same scaling for each joint
        max_translationalJoints = np.zeros(len(self.translationalJoints))
        for count, translationalJoint in enumerate(self.translationalJoints): 
            max_translationalJoints[count] = pd.concat(
                    [abs(upperBoundsAcceleration[translationalJoint]),
                     abs(lowerBoundsAcceleration[translationalJoint])]).max(level=0)[0]
        maxAll_translationalJoints = np.max(max_translationalJoints)        
        
        for count, joint in enumerate(self.joints):         
            # Scaling            
            if joint in self.rotationalJoints:
                scalingAcceleration.insert(count, joint, [maxAll_rotationalJoints])
            elif joint in self.translationalJoints:
                scalingAcceleration.insert(count, joint, [maxAll_translationalJoints])
            else:
                 raise ValueError('Unknown joint')   
            lowerBoundsAcceleration[joint] /= scalingAcceleration[joint]
            upperBoundsAcceleration[joint] /= scalingAcceleration[joint]

        return (upperBoundsAcceleration, lowerBoundsAcceleration, 
                scalingAcceleration)
    
    def getBoundsActivation(self):
        lb = [0.05] 
        lb_vec = lb * len(self.muscles)
        ub = [1]
        ub_vec = ub * len(self.muscles)
        s = [1]
        s_vec = s * len(self.muscles)
        upperBoundsActivation = pd.DataFrame([ub_vec], columns=self.muscles)   
        lowerBoundsActivation = pd.DataFrame([lb_vec], columns=self.muscles) 
        scalingActivation = pd.DataFrame([s_vec], columns=self.muscles)
        upperBoundsActivation = upperBoundsActivation.div(scalingActivation)
        lowerBoundsActivation = lowerBoundsActivation.div(scalingActivation)
        for count, muscle in enumerate(self.muscles):
            upperBoundsActivation.insert(count + len(self.muscles), 
                                         muscle[:-1] + 'l', ub)
            lowerBoundsActivation.insert(count + len(self.muscles), 
                                         muscle[:-1] + 'l', lb)  

            # Scaling                       
            scalingActivation.insert(count + len(self.muscles), 
                                     muscle[:-1] + 'l', s)  
            upperBoundsActivation[
                    muscle[:-1] + 'l'] /= scalingActivation[muscle[:-1] + 'l']
            lowerBoundsActivation[
                    muscle[:-1] + 'l'] /= scalingActivation[muscle[:-1] + 'l']
        
        return upperBoundsActivation, lowerBoundsActivation, scalingActivation
    
    def getBoundsForce(self):
        lb = [0] 
        lb_vec = lb * len(self.muscles)
        ub = [5]
        ub_vec = ub * len(self.muscles)
        s = max([abs(lbi) for lbi in lb], [abs(ubi) for ubi in ub])
        s_vec = s * len(self.muscles)
        upperBoundsForce = pd.DataFrame([ub_vec], columns=self.muscles)   
        lowerBoundsForce = pd.DataFrame([lb_vec], columns=self.muscles) 
        scalingForce = pd.DataFrame([s_vec], columns=self.muscles)
        upperBoundsForce = upperBoundsForce.div(scalingForce)
        lowerBoundsForce = lowerBoundsForce.div(scalingForce)
        for count, muscle in enumerate(self.muscles):
            upperBoundsForce.insert(count + len(self.muscles), 
                                    muscle[:-1] + 'l', ub)
            lowerBoundsForce.insert(count + len(self.muscles), 
                                    muscle[:-1] + 'l', lb)  

            # Scaling                       
            scalingForce.insert(count + len(self.muscles), 
                                         muscle[:-1] + 'l', s)   
            upperBoundsForce[
                    muscle[:-1] + 'l'] /= scalingForce[muscle[:-1] + 'l']
            lowerBoundsForce[
                    muscle[:-1] + 'l'] /= scalingForce[muscle[:-1] + 'l']
        
        return upperBoundsForce, lowerBoundsForce, scalingForce
    
    def getBoundsActivationDerivative(self):
        activationTimeConstant = 0.015
        deactivationTimeConstant = 0.06
        lb = [-1 / deactivationTimeConstant] 
        lb_vec = lb * len(self.muscles)
        ub = [1 / activationTimeConstant]
        ub_vec = ub * len(self.muscles)
        s = [100]
        s_vec = s * len(self.muscles)
        upperBoundsActivationDerivative = pd.DataFrame([ub_vec], 
                                                       columns=self.muscles)   
        lowerBoundsActivationDerivative = pd.DataFrame([lb_vec], 
                                                       columns=self.muscles) 
        scalingActivationDerivative = pd.DataFrame([s_vec], 
                                                   columns=self.muscles)
        upperBoundsActivationDerivative = upperBoundsActivationDerivative.div(
                scalingActivationDerivative)
        lowerBoundsActivationDerivative = lowerBoundsActivationDerivative.div(
                scalingActivationDerivative)
        for count, muscle in enumerate(self.muscles):
            upperBoundsActivationDerivative.insert(count + len(self.muscles), 
                                                   muscle[:-1] + 'l', ub)
            lowerBoundsActivationDerivative.insert(count + len(self.muscles), 
                                                   muscle[:-1] + 'l', lb) 

            # Scaling                       
            scalingActivationDerivative.insert(count + len(self.muscles), 
                                               muscle[:-1] + 'l', s)  
            upperBoundsActivationDerivative[muscle[:-1] + 'l'] /= (
                    scalingActivationDerivative[muscle[:-1] + 'l'])
            lowerBoundsActivationDerivative[muscle[:-1] + 'l'] /= (
                    scalingActivationDerivative[muscle[:-1] + 'l'])             
        
        return (upperBoundsActivationDerivative, 
                lowerBoundsActivationDerivative, scalingActivationDerivative)
    
    def getBoundsForceDerivative(self):
        lb = [-100] 
        lb_vec = lb * len(self.muscles)
        ub = [100]
        ub_vec = ub * len(self.muscles)
        s = [100]
        s_vec = s * len(self.muscles)
        upperBoundsForceDerivative = pd.DataFrame([ub_vec], 
                                                  columns=self.muscles)   
        lowerBoundsForceDerivative = pd.DataFrame([lb_vec], 
                                                  columns=self.muscles) 
        scalingForceDerivative = pd.DataFrame([s_vec], 
                                                   columns=self.muscles)
        upperBoundsForceDerivative = upperBoundsForceDerivative.div(
                scalingForceDerivative)
        lowerBoundsForceDerivative = lowerBoundsForceDerivative.div(
                scalingForceDerivative)
        for count, muscle in enumerate(self.muscles):
            upperBoundsForceDerivative.insert(count + len(self.muscles), 
                                              muscle[:-1] + 'l', ub)
            lowerBoundsForceDerivative.insert(count + len(self.muscles), 
                                              muscle[:-1] + 'l', lb)   
            
            # Scaling                       
            scalingForceDerivative.insert(count + len(self.muscles), 
                                               muscle[:-1] + 'l', s)  
            upperBoundsForceDerivative[muscle[:-1] + 'l'] /= (
                    scalingForceDerivative[muscle[:-1] + 'l'])
            lowerBoundsForceDerivative[muscle[:-1] + 'l'] /= (
                    scalingForceDerivative[muscle[:-1] + 'l']) 
        
        return (upperBoundsForceDerivative, lowerBoundsForceDerivative, 
                scalingForceDerivative)
    
    def getBoundsLumbarExcitation(self, lumbarJoints):
        lb = [-1] 
        lb_vec = lb * len(lumbarJoints)
        ub = [1]
        ub_vec = ub * len(lumbarJoints)
        s = [150]
        s_vec = s * len(lumbarJoints)
        upperBoundsLumbarExcitation = pd.DataFrame([ub_vec], 
                                                columns=lumbarJoints)   
        lowerBoundsLumbarExcitation = pd.DataFrame([lb_vec], 
                                                columns=lumbarJoints)            
        scalingLumbarExcitation = pd.DataFrame([s_vec], columns=lumbarJoints)
        
        return (upperBoundsLumbarExcitation, lowerBoundsLumbarExcitation,
                scalingLumbarExcitation)
    
    def getBoundsLumbarActivation(self, lumbarJoints):
        lb = [-1] 
        lb_vec = lb * len(lumbarJoints)
        ub = [1]
        ub_vec = ub * len(lumbarJoints)
        s = [150]
        s_vec = s * len(lumbarJoints)
        upperBoundsLumbarActivation = pd.DataFrame([ub_vec], 
                                                columns=lumbarJoints)   
        lowerBoundsLumbarActivation = pd.DataFrame([lb_vec], 
                                                columns=lumbarJoints) 
        scalingLumbarActivation = pd.DataFrame([s_vec], columns=lumbarJoints)                  
        
        return (upperBoundsLumbarActivation, lowerBoundsLumbarActivation, 
                scalingLumbarActivation)
        
    def getBoundsMtpExcitation(self, mtpJoints):
        lb = [-1] 
        lb_vec = lb * len(mtpJoints)
        ub = [1]
        ub_vec = ub * len(mtpJoints)
        s = [100]
        s_vec = s * len(mtpJoints)
        upperBoundsMtpExcitation = pd.DataFrame([ub_vec], 
                                                columns=mtpJoints)   
        lowerBoundsMtpExcitation = pd.DataFrame([lb_vec], 
                                                columns=mtpJoints)            
        scalingMtpExcitation = pd.DataFrame([s_vec], columns=mtpJoints)
        
        return (upperBoundsMtpExcitation, lowerBoundsMtpExcitation,
                scalingMtpExcitation)
    
    def getBoundsMtpActivation(self, mtpJoints):
        lb = [-1] 
        lb_vec = lb * len(mtpJoints)
        ub = [1]
        ub_vec = ub * len(mtpJoints)
        s = [100]
        s_vec = s * len(mtpJoints)
        upperBoundsMtpActivation = pd.DataFrame([ub_vec], 
                                                columns=mtpJoints)   
        lowerBoundsMtpActivation = pd.DataFrame([lb_vec], 
                                                columns=mtpJoints) 
        scalingMtpActivation = pd.DataFrame([s_vec], columns=mtpJoints)                  
        
        return (upperBoundsMtpActivation, lowerBoundsMtpActivation, 
                scalingMtpActivation)
    
    def getBoundsFinalTime(self):
        upperBoundsFinalTime = pd.DataFrame([1], columns=['time'])   
        lowerBoundsFinalTime = pd.DataFrame([0.1], columns=['time'])  
        
        return upperBoundsFinalTime, lowerBoundsFinalTime
    
    def getScalingContacts(self):
        
        scalingGRF = 1000
        scalingGRM = 1000
        
        return scalingGRF, scalingGRM    
    
    def getScalingID(self):
        
        scalingID = 150
        
        return scalingID   
    
    def getBoundsContactParameters_3s_all(self):
        
        lbLocation_s1 = np.array([0, -0.02, -0.02])
        lbLocation_s2 = np.array([0.05, -0.02, -0.02])
        lbLocation_s3 = np.array([0, -0.02, -0.02])
        
        ubLocation_s1 = np.array([0.02, 0.02, 0.02])
        ubLocation_s2 = np.array([0.2, 0.02, 0.02])
        ubLocation_s3 = np.array([0.035, 0.02, 0.02])
        
        lbRadius_s1 = 0.015
        lbRadius_s2 = 0.015
        lbRadius_s3 = 0.015
        
        ubRadius_s1 = 0.035
        ubRadius_s2 = 0.035
        ubRadius_s3 = 0.035
        
        lbStiffness_s1 = 1000000
        lbStiffness_s2 = 1000000
        lbStiffness_s3 = 1000000
        
        ubStiffness_s1 = 3000000
        ubStiffness_s2 = 3000000
        ubStiffness_s3 = 3000000
        
        lbContactParameters_unsc = np.concatenate((lbLocation_s1, lbLocation_s2, lbLocation_s3, [lbRadius_s1], [lbRadius_s2], [lbRadius_s3], [lbStiffness_s1], [lbStiffness_s2], [lbStiffness_s3]))
        ubContactParameters_unsc = np.concatenate((ubLocation_s1, ubLocation_s2, ubLocation_s3, [ubRadius_s1], [ubRadius_s2], [ubRadius_s3], [ubStiffness_s1], [ubStiffness_s2], [ubStiffness_s3]))
        
        scalingContactParameters_v = 1 / (ubContactParameters_unsc - lbContactParameters_unsc)
        scalingContactParameters_r = 0.5 - ubContactParameters_unsc / (ubContactParameters_unsc - lbContactParameters_unsc)
        
        lowerBoundsContactParameters = -0.5 * np.ones((1, len(lbContactParameters_unsc)))
        upperBoundsContactParameters = 0.5 * np.ones((1, len(ubContactParameters_unsc)))
        
        return upperBoundsContactParameters, lowerBoundsContactParameters, scalingContactParameters_v, scalingContactParameters_r       
    
    def getBoundsContactParameters_2s_option1(self):
        
        lbLocation_s1 = np.array([0, -0.02, -0.02])
        lbLocation_s2 = np.array([0, -0.02, -0.02])
        
        ubLocation_s1 = np.array([0.02, 0.02, 0.02])
        ubLocation_s2 = np.array([0.035, 0.02, 0.02])
        
        lbRadius_s1 = 0.015
        lbRadius_s2 = 0.015
        
        ubRadius_s1 = 0.035
        ubRadius_s2 = 0.035
        
        lbStiffness = 1000000        
        ubStiffness = 3000000
        
        lbContactParameters_unsc = np.concatenate((lbLocation_s1, lbLocation_s2, [lbRadius_s1], [lbRadius_s2], [lbStiffness]))
        ubContactParameters_unsc = np.concatenate((ubLocation_s1, ubLocation_s2, [ubRadius_s1], [ubRadius_s2], [ubStiffness]))
        
        scalingContactParameters_v = 1 / (ubContactParameters_unsc - lbContactParameters_unsc)
        scalingContactParameters_r = 0.5 - ubContactParameters_unsc / (ubContactParameters_unsc - lbContactParameters_unsc)
        
        lowerBoundsContactParameters = -0.5 * np.ones((1, len(lbContactParameters_unsc)))
        upperBoundsContactParameters = 0.5 * np.ones((1, len(ubContactParameters_unsc)))
        
        return upperBoundsContactParameters, lowerBoundsContactParameters, scalingContactParameters_v, scalingContactParameters_r  
    
    def getBoundsContactParameters_2s_option2(self):
        
        lbLocation_s1 = np.array([0, -0.02])
        lbLocation_s2 = np.array([0, -0.02])        
        
        ubLocation_s1 = np.array([0.02, 0.02])
        ubLocation_s2 = np.array([0.035, 0.02])        
        
        lbRadius = 0.015        
        ubRadius = 0.035
        
        lbContactParameters_unsc = np.concatenate((lbLocation_s1, lbLocation_s2, [lbRadius]))
        ubContactParameters_unsc = np.concatenate((ubLocation_s1, ubLocation_s2, [ubRadius]))
        
        scalingContactParameters_v = 1 / (ubContactParameters_unsc - lbContactParameters_unsc)
        scalingContactParameters_r = 0.5 - ubContactParameters_unsc / (ubContactParameters_unsc - lbContactParameters_unsc)
        
        lowerBoundsContactParameters = -0.5 * np.ones((1, len(lbContactParameters_unsc)))
        upperBoundsContactParameters = 0.5 * np.ones((1, len(ubContactParameters_unsc)))
        
        return upperBoundsContactParameters, lowerBoundsContactParameters, scalingContactParameters_v, scalingContactParameters_r  
    
    def getBoundsContactParameters_3s_option2(self):
        
        lbLocation_s1 = np.array([0, -0.02])
        lbLocation_s2 = np.array([0, -0.02])
        lbLocation_s3 = np.array([0.05, -0.02])
        
        ubLocation_s1 = np.array([0.02, 0.02])
        ubLocation_s2 = np.array([0.035, 0.02])
        ubLocation_s3 = np.array([0.06, 0.02])
        
        lbRadius = 0.015        
        ubRadius = 0.035
        
        lbContactParameters_unsc = np.concatenate((lbLocation_s1, lbLocation_s2, lbLocation_s3, [lbRadius], [lbRadius]))
        ubContactParameters_unsc = np.concatenate((ubLocation_s1, ubLocation_s2, ubLocation_s3, [ubRadius], [ubRadius]))
        
        scalingContactParameters_v = 1 / (ubContactParameters_unsc - lbContactParameters_unsc)
        scalingContactParameters_r = 0.5 - ubContactParameters_unsc / (ubContactParameters_unsc - lbContactParameters_unsc)
        
        lowerBoundsContactParameters = -0.5 * np.ones((1, len(lbContactParameters_unsc)))
        upperBoundsContactParameters = 0.5 * np.ones((1, len(ubContactParameters_unsc)))
        
        return upperBoundsContactParameters, lowerBoundsContactParameters, scalingContactParameters_v, scalingContactParameters_r  
    