import numpy as np
import matplotlib.pyplot as plt

# %% This class evaluates polynomial approximations given the coefficients,
# the dimension, and the order.
class polynomials:
    
    def __init__(self, coefficients, dimension, order):
        
        self.coefficients = coefficients
        self.dimension = dimension
        self.order = order
        
        nq = [0, 0, 0, 0]
        NCoeff = 0
        for nq[0] in range(order + 1):
            if (dimension < 2):
                nq2_s = 0
            else:
                nq2_s = order - nq[0]
            for nq[1] in range(nq2_s + 1):
                if (dimension < 3):
                    nq3_s = 0
                else:
                    nq3_s = order - nq[0] - nq[1]
                for nq[2] in range(nq3_s + 1):
                    if (dimension < 4):
                        nq4_s = 0
                    else:
                        nq4_s = order - nq[0] - nq[1] - nq[2]
                    for nq[3] in range(nq4_s + 1):
                        NCoeff += 1
        
        if len(coefficients) != NCoeff:
            raise Exception('Expected: {}'.format(NCoeff), 'coefficients', 
                            'but got: {}'.format(len(coefficients)))
            
    def calcValue(self, x):        
        nq = [0, 0, 0, 0]
        coeff_nr = 0
        value = 0
        for nq[0] in range(self.order + 1):
            if (self.dimension < 2):
                nq2_s = 0
            else:
                nq2_s = self.order - nq[0]
            for nq[1] in range(nq2_s + 1):
                if (self.dimension < 3):
                    nq3_s = 0
                else:
                    nq3_s = self.order - nq[0] - nq[1]
                for nq[2] in range(nq3_s + 1):
                    if (self.dimension < 4):
                        nq4_s = 0
                    else:
                        nq4_s = self.order - nq[0] - nq[1] - nq[2]
                    for nq[3] in range(nq4_s + 1):
                        valueP = 1
                        for d in range(self.dimension):
                            valueP *= pow(x[d], nq[d])                            
                        value += valueP * self.coefficients[coeff_nr]
                        coeff_nr += 1
                        
        return value
    
    def calcDerivative(self, x, derivComponent):
        nq = [0, 0, 0, 0]
        coeff_nr = 0
        value = 0
        for nq[0] in range(self.order + 1):
            if (self.dimension < 2):
                nq2_s = 0
            else:
                nq2_s = self.order - nq[0]
            for nq[1] in range(nq2_s + 1):
                if (self.dimension < 3):
                    nq3_s = 0
                else:
                    nq3_s = self.order - nq[0] - nq[1]
                for nq[2] in range(nq3_s + 1):
                    if (self.dimension < 4):
                        nq4_s = 0
                    else:
                        nq4_s = self.order - nq[0] - nq[1] - nq[2]
                    for nq[3] in range(nq4_s + 1):
                        if (derivComponent == 0):
                            nqNonNegative = nq[0] - 1
                            if (nqNonNegative < 0):
                                nqNonNegative = 0
                            valueP = nq[0] * pow(x[0], nqNonNegative);
                            for d in range(self.dimension):
                                if (d == derivComponent):
                                    continue
                                valueP *= pow(x[d], nq[d])
                            value += valueP * self.coefficients[coeff_nr]
                        elif (derivComponent == 1):
                            nqNonNegative = nq[1] - 1
                            if (nqNonNegative < 0):
                                nqNonNegative = 0
                            valueP = nq[1] * pow(x[1], nqNonNegative);
                            for d in range(self.dimension):
                                if (d == derivComponent):
                                    continue
                                valueP *= pow(x[d], nq[d])
                            value += valueP * self.coefficients[coeff_nr]
                        elif (derivComponent == 2):
                            nqNonNegative = nq[2] - 1
                            if (nqNonNegative < 0):
                                nqNonNegative = 0
                            valueP = nq[2] * pow(x[2], nqNonNegative);
                            for d in range(self.dimension):
                                if (d == derivComponent):
                                    continue
                                valueP *= pow(x[d], nq[d])
                            value += valueP * self.coefficients[coeff_nr]
                        elif (derivComponent == 3):
                            nqNonNegative = nq[3] - 1
                            if (nqNonNegative < 0):
                                nqNonNegative = 0
                            valueP = nq[3] * pow(x[3], nqNonNegative);
                            for d in range(self.dimension):
                                if (d == derivComponent):
                                    continue
                                valueP *= pow(x[d], nq[d])
                            value += valueP * self.coefficients[coeff_nr]
                        coeff_nr += 1                 
                                                
        return value
    
# %% This class evaluates the terms of the polynomial approximations, given
# the dimension and the order. It is used when fitting the coefficients.        
class polynomial_estimation:
    
    def __init__(self, dimension, order):
        
        self.dimension = dimension
        self.order = order
        
        nq = [0, 0, 0, 0]
        self.NCoeff = 0
        for nq[0] in range(order + 1):
            if (dimension < 2):
                nq2_s = 0
            else:
                nq2_s = order - nq[0]
            for nq[1] in range(nq2_s + 1):
                if (dimension < 3):
                    nq3_s = 0
                else:
                    nq3_s = order - nq[0] - nq[1]
                for nq[2] in range(nq3_s + 1):
                    if (dimension < 4):
                        nq4_s = 0
                    else:
                        nq4_s = order - nq[0] - nq[1] - nq[2]
                    for nq[3] in range(nq4_s + 1):
                        self.NCoeff += 1
                    
    def calcValue(self, x):        
        nq = [0, 0, 0, 0]
        coeff_nr = 0
        value = np.zeros((x.shape[0], self.NCoeff))
        for nq[0] in range(self.order + 1):
            if (self.dimension < 2):
                nq2_s = 0
            else:
                nq2_s = self.order - nq[0]
            for nq[1] in range(nq2_s + 1):
                if (self.dimension < 3):
                    nq3_s = 0
                else:
                    nq3_s = self.order - nq[0] - nq[1]
                for nq[2] in range(nq3_s + 1):
                    if (self.dimension < 4):
                        nq4_s = 0
                    else:
                        nq4_s = self.order - nq[0] - nq[1] - nq[2]
                    for nq[3] in range(nq4_s + 1):
                        valueP = 1
                        for d in range(self.dimension):
                            valueP *= pow(x[:,d], nq[d])                            
                        value[:,coeff_nr ] = valueP
                        coeff_nr += 1
                        
        return value
    
    def calcDerivative(self, x, derivComponent):
        nq = [0, 0, 0, 0]
        coeff_nr = 0
        value = np.zeros((x.shape[0], self.NCoeff))
        for nq[0] in range(self.order + 1):
            if (self.dimension < 2):
                nq2_s = 0
            else:
                nq2_s = self.order - nq[0]
            for nq[1] in range(nq2_s + 1):
                if (self.dimension < 3):
                    nq3_s = 0
                else:
                    nq3_s = self.order - nq[0] - nq[1]
                for nq[2] in range(nq3_s + 1):
                    if (self.dimension < 4):
                        nq4_s = 0
                    else:
                        nq4_s = self.order - nq[0] - nq[1] - nq[2]
                    for nq[3] in range(nq4_s + 1):
                        if (derivComponent == 0):
                            nqNonNegative = nq[0] - 1
                            if (nqNonNegative < 0):
                                nqNonNegative = 0
                            valueP = nq[0] * pow(x[:,0], nqNonNegative);
                            for d in range(self.dimension):
                                if (d == derivComponent):
                                    continue
                                valueP *= pow(x[:,d], nq[d])
                            value[:,coeff_nr ] = valueP
                        elif (derivComponent == 1):
                            nqNonNegative = nq[1] - 1
                            if (nqNonNegative < 0):
                                nqNonNegative = 0
                            valueP = nq[1] * pow(x[:,1], nqNonNegative);
                            for d in range(self.dimension):
                                if (d == derivComponent):
                                    continue
                                valueP *= pow(x[:,d], nq[d])
                            value[:,coeff_nr ] = valueP
                        elif (derivComponent == 2):
                            nqNonNegative = nq[2] - 1
                            if (nqNonNegative < 0):
                                nqNonNegative = 0
                            valueP = nq[2] * pow(x[:,2], nqNonNegative);
                            for d in range(self.dimension):
                                if (d == derivComponent):
                                    continue
                                valueP *= pow(x[:,d], nq[d])
                            value[:,coeff_nr ] = valueP
                        elif (derivComponent == 3):
                            nqNonNegative = nq[3] - 1
                            if (nqNonNegative < 0):
                                nqNonNegative = 0
                            valueP = nq[3] * pow(x[:,3], nqNonNegative);
                            for d in range(self.dimension):
                                if (d == derivComponent):
                                    continue
                                valueP *= pow(x[:,d], nq[d])
                            value[:,coeff_nr ] = valueP
                        coeff_nr += 1                 
                                                
        return value
        
# %% This functions fits the polynomial coefficients.
def getPolynomialCoefficients(pathCoordinates, pathMuscleAnalysis, joints,
                              muscles, order_min=3, order_max=9, 
                              threshold=0.003):
    
    # Get joint coordinates.
    from utilities import getIK
    jointCoordinates = (getIK(pathCoordinates, joints)[0]).to_numpy()[:,1::]
    
    # Get muscle-tendon lengths.
    from utilities import getFromStorage
    pathMuscleTendonLengths = pathMuscleAnalysis + 'Length.sto'
    muscleTendonLengths = getFromStorage(
        pathMuscleTendonLengths, muscles).to_numpy()[:,1::] 
    
    # Get moment arms.
    momentArms = np.zeros((jointCoordinates.shape[0], len(muscles),
                           len(joints)))
    for i, joint in enumerate(joints):
        pathMomentArm = pathMuscleAnalysis + 'MomentArm_' + joint + '.sto'
        # getFromStorage outputs time vector as well, so [:,1::].
        momentArms[:, :, i] = getFromStorage(
            pathMomentArm, muscles).to_numpy()[:,1::] 
    # Detect which muscles actuate which joints.  
    spanningInfo = np.sum(momentArms, axis=0)
    spanningInfo = np.where(np.logical_and(
        spanningInfo<=0.0001, spanningInfo>=-0.0001), 0, 1)
        
    polynomialData = {}    
    for i, muscle in enumerate(muscles):
        muscle_momentArms = momentArms[:, i, spanningInfo[i, :]==1]
        muscle_dimension = muscle_momentArms.shape[1]
        muscle_muscleTendonLengths = muscleTendonLengths[:, i]
        
        is_fullfilled = False
        order = order_min
        while not is_fullfilled:
            
            polynomial = polynomial_estimation(muscle_dimension, order)
            mat = polynomial.calcValue(
                jointCoordinates[:, spanningInfo[i, :]==1])            
            diff_mat = np.zeros(
                (jointCoordinates.shape[0], mat.shape[1], muscle_dimension))    
            diff_mat_sq = np.zeros(
                (jointCoordinates.shape[0]*(muscle_dimension), mat.shape[1]))  
            for j in range(muscle_dimension):
                diff_mat[:,:,j] = polynomial.calcDerivative(
                    jointCoordinates[:, spanningInfo[i, :]==1], j)
                diff_mat_sq[
                    jointCoordinates.shape[0]*j:
                        jointCoordinates.shape[0]*(j+1),:] = -(
                            diff_mat[:,:,j]).reshape(-1, diff_mat.shape[1])
            
            A = np.concatenate((mat,diff_mat_sq),axis=0)            
            B = np.concatenate((muscle_muscleTendonLengths,
                                (muscle_momentArms.T).flatten()))
            
            # Solve least-square problem    .
            coefficients = np.linalg.lstsq(A,B,rcond=None)[0]
            
            # Compute difference with model data.
            # Muscle-tendon lengths.
            muscle_muscleTendonLengths_poly = np.matmul(mat,coefficients)
            muscleTendonLengths_diff_rms = np.sqrt(np.mean(
                    muscle_muscleTendonLengths -
                    muscle_muscleTendonLengths_poly)**2)
            # Moment-arms.
            muscle_momentArms_poly = np.zeros((jointCoordinates.shape[0],
                                               muscle_dimension))    
            for j in range(muscle_dimension):        
                muscle_momentArms_poly[:,j] = np.matmul(
                        -(diff_mat[:,:,j]).reshape(-1, diff_mat.shape[1]),
                        coefficients)                
            momentArms_diff_rms = np.sqrt(np.mean((
                    muscle_momentArms - muscle_momentArms_poly)**2, axis=0))
            
            # Check if criterion is satisfied.
            if (muscleTendonLengths_diff_rms <= threshold and 
                np.max(momentArms_diff_rms) <= threshold):
                is_fullfilled = True
            elif order == order_max:
                is_fullfilled = True
                print("Max order (" + str(order_max) + ") for " + muscle)            
            else:
                order += 1
                
        polynomialData[muscle] = {
            'dimension': muscle_dimension, 'order': order,
            'coefficients': coefficients, 'spanning': spanningInfo[i, :]}
        
    return polynomialData   

# %% This function plots muscle-tendon lengths and moment arms. Note that this
# is limited to 3D, so muscles actuating more than 2 DOFs are not displayed.
def testPolynomials(pathCoordinates, pathMuscleAnalysis, joints, muscles,
                    f_polynomial, polynomialData, momentArmIndices,
                    trunkMomentArmPolynomialIndices=[]):
    
    # Get joint coordinates.
    from utilities import getIK
    jointCoordinates = (getIK(pathCoordinates, joints)[0]).to_numpy()[:,1::]
    
    # Get muscle-tendon lengths.
    from utilities import getFromStorage
    pathMuscleTendonLengths = pathMuscleAnalysis + 'Length.sto'
    muscleTendonLengths = getFromStorage(
        pathMuscleTendonLengths, muscles).to_numpy()[:,1::] 
    
    # Get moment arms.
    momentArms = np.zeros((jointCoordinates.shape[0], len(muscles), 
                           len(joints)))
    for i, joint in enumerate(joints):
        pathMomentArm = pathMuscleAnalysis + 'MomentArm_' + joint + '.sto'
        # getFromStorage outputs time vector as well, so [:,1::].
        momentArms[:, :, i] = getFromStorage(
            pathMomentArm, muscles).to_numpy()[:,1::]
    
    # Approximate muscle-tendon lengths and moment-arms.
    lMT = np.zeros((len(muscles),muscleTendonLengths.shape[0]))
    dM = np.zeros((len(muscles),len(joints),muscleTendonLengths.shape[0]))
    dM_all = {}
    for k in range(muscleTendonLengths.shape[0]):
        Qsin = jointCoordinates[k, :].T
        Qdotsin = np.zeros((1,Qsin.shape[0]))
        lMT[:,k] = f_polynomial(Qsin, Qdotsin)[0].full().flatten()
        dM[:,:,k] = f_polynomial(Qsin, Qdotsin)[2].full()
        
    for j, joint in enumerate(joints):
        if joint[-1] == 'r' or joint[-1] == 'l':
            dM_all[joint] = dM[momentArmIndices[joint[:-1] + 'l'], j, :]
        else:
            dM_all[joint] = dM[trunkMomentArmPolynomialIndices, j, :]
        
    ny_0 = (np.sqrt(len(muscles))) 
    ny = np.floor(np.sqrt(len(muscles))) 
    ny_a = int(ny)
    ny_b = int(ny)
    if not ny == ny_0:
        ny_b = int(ny+1)
    fig = plt.figure()
    fig.suptitle('Muscle-tendon lengths')
    for i in range(len(muscles)):      
        muscle_obj = muscles[i][:-1] + 'r'
        if polynomialData[muscle_obj]['dimension'] == 1:
            temp = polynomialData[muscle_obj]['spanning']==1
            y = (i for i,v in enumerate(temp) if v == True)
            x1 = next(y)
            ax = fig.add_subplot(ny_a, ny_b, i+1)
            ax.scatter(jointCoordinates[:,x1],lMT[i,:])
            ax.scatter(jointCoordinates[:,x1],muscleTendonLengths[:,i],c='r')
            ax.set_title(muscles[i])
            ax.set_xlabel(joints[x1])
        elif polynomialData[muscle_obj]['dimension'] == 2:
            ax = fig.add_subplot(ny_a, ny_b, i+1, projection='3d')
            temp = polynomialData[muscle_obj]['spanning']==1
            y = (i for i,v in enumerate(temp) if v == True)
            x1 = next(y)
            x2 = next(y)
            ax.scatter(jointCoordinates[:,x1],jointCoordinates[:,x2],lMT[i,:])
            ax.scatter(jointCoordinates[:,x1],jointCoordinates[:,x2],
                       muscleTendonLengths[:,i],c='r')
            ax.set_title(muscles[i])
            ax.set_xlabel(joints[x1])
            ax.set_ylabel(joints[x2])   
            
    for i, joint in enumerate(joints):
        fig = plt.figure()
        fig.suptitle('Moment arms: ' + joint)
        NMomentarms = len(momentArmIndices[joint])
        ny_0 = (np.sqrt(NMomentarms)) 
        ny = np.round(ny_0) 
        ny_a = int(ny)
        ny_b = int(ny)
        if (ny == ny_0) == False:
            if ny_a == 1:
                ny_b = NMomentarms
            if ny < ny_0:
                ny_b = int(ny+1)
        for j in range(NMomentarms):
            if joint[-1] == 'r' or joint[-1] == 'l':
                muscle_obj_r = (
                    muscles[momentArmIndices[joint[:-1] + 'l'][j]][:-1] + 'r')
                muscle_obj = muscles[momentArmIndices[joint[:-1] + 'l'][j]]
            else:
                muscle_obj_r = muscles[trunkMomentArmPolynomialIndices[j]]
                muscle_obj = muscles[trunkMomentArmPolynomialIndices[j]]
            if polynomialData[muscle_obj_r]['dimension'] == 1:
                temp = polynomialData[muscle_obj_r]['spanning']==1
                y = (i for i,v in enumerate(temp) if v == True)
                x1 = next(y)
                ax = fig.add_subplot(ny_a, ny_b, j+1)
                ax.scatter(jointCoordinates[:,x1],dM_all[joint][j,:])
                if joint[-1] == 'r' or joint[-1] == 'l':
                    ax.scatter(
                        jointCoordinates[:,x1],
                        momentArms[:,momentArmIndices[joint[:-1] + 'l'][j],i],
                        c='r')
                else:
                    ax.scatter(
                        jointCoordinates[:,x1],
                        momentArms[:,trunkMomentArmPolynomialIndices[j],i],
                        c='r')
                ax.set_title(muscle_obj)
                ax.set_xlabel(joints[x1])
            if polynomialData[muscle_obj_r]['dimension'] == 2:
                temp = polynomialData[muscle_obj_r]['spanning']==1
                y = (i for i,v in enumerate(temp) if v == True)
                x1 = next(y)
                x2 = next(y)
                ax = fig.add_subplot(ny_a, ny_b, j+1, projection='3d')
                ax.scatter(jointCoordinates[:,x1],jointCoordinates[:,x2],
                           dM_all[joint][j,:])
                if joint[-1] == 'r' or joint[-1] == 'l':
                    ax.scatter(
                        jointCoordinates[:,x1],jointCoordinates[:,x2],
                        momentArms[:,momentArmIndices[joint[:-1] + 'l'][j],i],
                        c='r')
                else:
                    ax.scatter(
                        jointCoordinates[:,x1],jointCoordinates[:,x2],
                        momentArms[:,trunkMomentArmPolynomialIndices[j],i],
                        c='r')
                ax.set_title(muscle_obj)
                ax.set_xlabel(joints[x1])
                ax.set_ylabel(joints[x2])                  
