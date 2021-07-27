import numpy as np
import scipy.interpolate as interpolate
import casadi as ca

class smoothBhargava2004:
    
    def __init__(self, excitation, activation, normFiberLength, fiberVelocity,
                 activeFiberForce, passiveFiberForce, 
                 normActiveFiberLengthForce, slowTwitchRatio, 
                 maximalIsometricForce, muscleMass,  smoothingConstant):
        
        self.excitation = excitation
        self.activation = activation
        self.normFiberLength = normFiberLength
        self.fiberVelocity = fiberVelocity
        self.activeFiberForce = activeFiberForce
        self.passiveFiberForce = passiveFiberForce
        self.normActiveFiberLengthForce = normActiveFiberLengthForce
        self.slowTwitchRatio = slowTwitchRatio
        self.maximalIsometricForce = maximalIsometricForce
#        self.modelMass = modelMass
        self.muscleMass = muscleMass
        self.smoothingConstant = smoothingConstant
        
    def getTwitchExcitation(self):        
        fastTwitchRatio = 1 - self.slowTwitchRatio        
        self.slowTwitchExcitation = self.slowTwitchRatio * np.sin(
                np.pi / 2 * self.excitation)
        self.fastTwitchExcitation = fastTwitchRatio * (
                1 - np.cos(np.pi / 2 * self.excitation))
        
        return self.slowTwitchExcitation, self.fastTwitchExcitation
        
    def getActivationHeatRate(self):
        self.getTwitchExcitation()
        decay_function_value = 1 
        activation_constant_slowTwitch = 40 # default (Bhargava et al. 2004)
        activation_constant_fastTwitch = 133 # default (Bhargava et al. 2004)
        
        self.Adot = self.muscleMass * decay_function_value * (
                (activation_constant_slowTwitch * self.slowTwitchExcitation) + 
                (activation_constant_fastTwitch * self.fastTwitchExcitation))
        
        return self.Adot
    
    def getMaintenanceHeatRate(self, use_fiber_length_dep_curve=False):
        self.getTwitchExcitation()
        
        if use_fiber_length_dep_curve == True:
    #        x_forceDep = np.array(([0.0, 0.5, 1.0, 1.5, 2.0, 2.5]))
    #        y_forceDep = np.array(([0.5, 0.5, 1.0, 0.0, 0.0, 0.0]))
    #        x_forceDep = np.array(([0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5]))
    #        y_forceDep = np.array(([0.5, 0.5,  0.5, 0.75, 1.0, 0.5,  0.0, 0.0,  0.0, 0.0,  0.0]))    
            x_forceDep = np.array(([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7,
                                    0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5,
                                    1.6, 1.7, 1.8, 1.9, 2.0, 2.1, 2.2, 2.3,
                                    2.4, 2.5]))
            y_forceDep = np.array(([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.6, 0.7,
                                    0.8, 0.9, 1.0, 0.8, 0.6, 0.4, 0.2, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0]))
            spline = ca.interpolant('spline','bspline',
                                    [x_forceDep], y_forceDep)                 
            fiber_length_dep = spline(self.normFiberLength) 
#            spline = interpolate.InterpolatedUnivariateSpline(x_forceDep, 
#                                                              y_forceDep,
#                                                              k=3)       
                   
#            temp = np.zeros((2,26))
#            for i, normFiberLength in enumerate(np.arange(0.0,2.6,0.1)):
#                temp[0,i] = (normFiberLength)
#                temp[1,i] = spline(normFiberLength)
#                print(str(spline(normFiberLength)))
#                
#            import matplotlib.pyplot as plt  
#            plt.figure(1)
#            plt.clf()
#            plt.plot(x_forceDep,y_forceDep, label='original')
#            plt.plot(temp[0,:], temp[1,:], label='spline')
        else:
            fiber_length_dep = self.normFiberLength
        maintenance_constant_slowTwitch = 74 # default (Bhargava et al. 2004)
        maintenance_constant_fastTwitch = 111 # default (Bhargava et al. 2004)
        
        self.Mdot =self.muscleMass * fiber_length_dep * (
                (maintenance_constant_slowTwitch * self.slowTwitchExcitation) +
                (maintenance_constant_fastTwitch * self.fastTwitchExcitation))
        
        return self.Mdot
    
    def getShorteningHeatRate(
            self,
            use_force_dependent_shortening_prop_constant=True):
        
        fiber_force_total = self.activeFiberForce + self.passiveFiberForce
        # vM is positive (muscle lengthening)
        vM_pos = 0.5 + 0.5 * np.tanh(self.smoothingConstant * 
                                     self.fiberVelocity)
        # vM is negative (muscle shortening)
        self.vM_neg = 1 - vM_pos
        
        if use_force_dependent_shortening_prop_constant == True:
            # F_iso that would be developed at the current activation and fiber 
            # length under isometric conditions (different than models of Umberger 
            # and Uchida). Fiso is getActiveForceLengthMultiplier in OpenSim. To 
            # minimize the difference between the models, we keep the same input, 
            # i.e., Fiso, that is used in the models of Umberger and Uchida.        
            F_iso = (self.activation * self.maximalIsometricForce * 
                 self.normActiveFiberLengthForce)        
            alpha = (0.16 * F_iso) + (0.18 * fiber_force_total)            
            alpha = alpha + (-alpha + 0.157 * fiber_force_total) * vM_pos        
        else:
            alpha = 0.25 * fiber_force_total
            alpha = alpha + (-alpha) * vM_pos
        self.Sdot = -alpha * self.fiberVelocity
        
        return self.Sdot
    
    def getMechanicalWork(self, include_negative_mechanical_work=False):
        if include_negative_mechanical_work==True:
            self.Wdot = - self.activeFiberForce * self.fiberVelocity
        else:
            self.Wdot = - self.activeFiberForce * self.fiberVelocity * self.vM_neg
        
        return self.Wdot
    
    def getTotalHeatRate(self):
        self.getActivationHeatRate()
        self.getMaintenanceHeatRate()
        self.getShorteningHeatRate()
        self.getMechanicalWork()
        # Total power is non-negative
        # If necessary, increase the shortening heat rate
        # https://github.com/opensim-org/opensim-core/blob/master/OpenSim/Simulation/Model/Bhargava2004MuscleMetabolicsProbe.cpp#L393
        Edot_W_beforeClamp = self.Adot + self.Mdot + self.Sdot + self.Wdot
        Edot_W_beforeClamp_neg = 0.5 + (0.5 * np.tanh(self.smoothingConstant * 
                                                      (-Edot_W_beforeClamp)))
        SdotClamp = self.Sdot - Edot_W_beforeClamp * Edot_W_beforeClamp_neg 
        # The total heat rate is not allowed to drop below 1(W/kg)
        # https://github.com/opensim-org/opensim-core/blob/master/OpenSim/Simulation/Model/Bhargava2004MuscleMetabolicsProbe.cpp#L400
        self.totalHeatRate = self.Adot + self.Mdot + SdotClamp
        # We first express in W/kg 
        self.totalHeatRate /= self.muscleMass
        self.totalHeatRate += (-self.totalHeatRate + 1) * (0.5 + 
                              0.5 * np.tanh(self.smoothingConstant * 
                                            (1 - self.totalHeatRate)))
        # We then express back in W
        self.totalHeatRate *= self.muscleMass
        
        return self.totalHeatRate
    
    def getMetabolicEnergyRate(self):
        self.getTotalHeatRate()
        self.metabolicEnergyDot = self.totalHeatRate + self.Wdot
        
        return self.metabolicEnergyDot
    
# Test model
#excitation = 0.8*np.ones((NMuscles, 1))
#activation = 0.7*np.ones((NMuscles, 1))
#normFiberLength = 0.7*np.ones((NMuscles, 1))
#fiberVelocity = 1.7*np.ones((NMuscles, 1))
#activeFiberForce = 5*np.ones((NMuscles, 1))
#passiveFiberForce = 0.8*np.ones((NMuscles, 1))
#normActiveFiberLengthForce = 0.5*np.ones((NMuscles, 1))
#
#metabolicEnergyRateT = f_metabolicsBhargava(excitation, activation, 
#                                     normFiberLength, fiberVelocity, 
#                                     activeFiberForce, passiveFiberForce, 
#                                     normActiveFiberLengthForce)[5]
#print(metabolicEnergyRateT)