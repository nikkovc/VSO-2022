"""Attitude estimator functions for the attitude EKF

Attributes:
    eye3 (TYPE): 3x3 identity matrix
    eye6 (TYPE): 6x6 identity matrix
"""
import numpy as np
from time import time

eye3 = np.eye(3)
eye6 = np.eye(6)

def expMap(x):
    """Takes the analytical matrix exponential of a three vector
    
    Args:
        x (TYPE): the three vector input
    
    Returns:
        TYPE: the matrix exponential
    """
    return sp.linalg.expm(hatMap(x))

def logMap(x):
    """Takes the analytical matrix logarithm
    
    Args:
        x (TYPE): a 3x3 matrix 
    
    Returns:
        TYPE: the three vector that's the logarithm of matrix x
    """
    S = sp.linalg.logm(x)
    return invHatMap(S)

def returnAxisAngle(r):
    """returns the axis angle representation of a three-vector
    
    Args:
        r (TYPE): a three vector
    
    Returns:
        TYPE: a tuple of the angle and axis 
    """
    angle_estimate_AE = np.linalg.norm(r)
    # print(angle_estimate_AE)
    if angle_estimate_AE == 0:
        axis_estimate_AE = r
    else:
        axis_estimate_AE = r/angle_estimate_AE

    return (angle_estimate_AE, axis_estimate_AE)

def applyRodrigues(w_hat,theta):
    """Applies Rodrigues' formula to a rotation axis and an angle
    
    Args:
        w_hat (TYPE): a 3-(unit)vector encoding the rotation axis 
        theta (TYPE): the rotation magnitude
    
    Returns:
        TYPE: the matrix exponential encoding the rotation
    """
    matrixExp = eye3 + np.sin(theta) * w_hat + (1 - np.cos(theta)) * (w_hat @ w_hat)

    return matrixExp

def rotationMapRodrigues(r):
    """Applies Rodrigues' formula to an arbitrary 3-vector
    
    Args:
        r (TYPE): a 3-vector encoding the rotation axis 
    
    Returns:
        TYPE: the matrix exponential encoding the rotation
    """
    (theta, w) = returnAxisAngle(r)
    matrixExp = applyRodrigues(hatMap(w),theta)
    return matrixExp

def logMapManual(R_AE):
    """Takes the logarithm of a rotation matrix
    
    Args:
        R_AE (TYPE): A rotation matrix
    
    Returns:
        TYPE: the axis and angle of the rotation matrix
    """
    r11 = R_AE[0,0]
    r12 = R_AE[0,1]
    r13 = R_AE[0,2]

    r21 = R_AE[1,0]
    r22 = R_AE[1,1]
    r23 = R_AE[1,2]

    r31 = R_AE[2,0]
    r32 = R_AE[2,1]
    r33 = R_AE[2,2]

    # print('R_AE')
    # print(R_AE)

    # print(np.trace(R_AE))
    # if (R_AE == np.eye(3)).all():
    if np.trace(R_AE) == 3:
        # print('1')
        theta = 0;
        w = np.zeros((3,1))

    elif np.trace(R_AE) == -1:
        # print('2')
        theta = np.pi

        if r11 == -1:

            if r22 == -1:
                w = (1/np.sqrt(2 * (1 + r33))) * np.array([[r13],[r23],[1 + r33]])

            else:
                w = (1/np.sqrt(2 * (1 + r22))) * np.array([[r12],[1 + r22],[r32]])

        else:
            w = (1/np.sqrt(2 * (1 + r11))) * np.array([[1 + r11],[r21],[r31]])

    else:
        # print('3')
        # print(1/2 * (np.trace(R_AE) - 1  ))
        theta = np.arccos( 1/2 * (np.trace(R_AE) - 1  )  )

        w_hat = (1/(2 * np.sin(theta))) * (R_AE - R_AE.transpose())
        w = invHatMap(w_hat)

    return (theta,w)

def hatMap(x):
    """encodes the cross product with a vector x in 3x3 matrix form
    
    Args:
        x (TYPE): a 3-vector
    
    Returns:
        TYPE: the 3x3 matrix representing the cross product with x
    """
    x=x.reshape((-1,))
    return np.array([
        [ 0.0000,  -x[2],   x[1]],
        [   x[2], 0.0000,  -x[0]],
        [  -x[1],   x[0], 0.0000],
    ])

def invHatMap(S):
    """Turns a skew-symmetric matrix and extracts the 3-vector it encodes
    
    Args:
        S (TYPE): a skew-symmetric matrix 
    
    Returns:
        TYPE: a 3-vector
    """
    # S should be skew symmetric
    assert(np.linalg.norm(S+S.T)<1e-7)
    return np.array([[S[2,1], S[0,2], S[1,0]]]).T


def f(x, dt):
    """discrete-time steps through the process model of the Attitude EKF
    Assumes constant rotational speed and integrates to a new angle over dt
    
    Args:
        x (TYPE): the three-vector encoding the current rotational state of the A-EKF
        dt (TYPE): the time-step
    
    Returns:
        TYPE: the predicted next rotational three-vector
    """
    # print('f')
    # print('x')
    # print(x)
    time0 = time()
    r_old = x[:3,0]
    omega_old = x[3:, [0]]
    # print(omega_old.shape)s
    timeExp0 = time()
    # (angle_old, axis_old) = returnAxisAngle(r_old)
    # (angle_omega, axis_omega) = returnAxisAngle(omega_old*dt)
    
    R_old = rotationMapRodrigues(r_old)
    R_omega = rotationMapRodrigues(omega_old*dt)

    R_AE =  R_omega @ R_old 
    # print(R_AE)
    timeExp1 = time()

    # print('time exp: ' + str(timeExp1 - timeExp0))

    timeLog0 = time()

    (theta_next,w_next) = logMapManual(R_AE)
    # print(theta_next)
    # print(w_next)
    r_next = theta_next * w_next

    # print(r_next)
    # r_next = logMap( R_AE )  


    timeLog1 = time()

    # print('time log: ' + str(timeLog1 - timeLog0))
    # print('r_next')
    # print(r_next)
    omega_next = omega_old
    x_next = np.array(x)
    x_next[:3,[0]]=r_next
    x_next[3:,[0]]=omega_next
    # print('x_next')
    # print(x_next)
    time1 = time()
    # print('f time: '+str(time1 - time0))
    return x_next



def h(x):
    """discrete-time, encodes the measurement function of the A-EKF
     returns gyro and accel readings in IMU frame
    
    Args:
        x (TYPE): the three-vector encoding the current rotational state of the A-EKF
    
    Returns:
        TYPE: the predicted gyro and accel readings in IMU frame
    """
    # print(x)
    r = x[:3,0]
    # print(r)
    omega = x[3:, [0]]
    # print(omega)
    # print(omega.shape)

    zmodel = np.zeros((6,1))
    # print(zmodel.shape)

    # print(zmodel[3:,0])

    g_vec_global = np.array([[0],[0],[1]])
    # print(g_vec_global.shape)
    R_AE = rotationMapRodrigues(r)
    
    #first row second column

    # print(R_AE.transpose() @ g_vec_global)
    zmodel[:3,[0]] = R_AE.transpose() @ omega
    # print(zmodel)
    # zmodel[:3,[0]] = omega


    zmodel[3:6,[0]] = R_AE.transpose() @ g_vec_global
    # print(zmodel)
    # print(zmodel)
    return zmodel


def h_gyroOnly(x):
    """discrete-time, encodes the measurement function of the A-EKF
     returns gyro readings in IMU frame
    
    Args:
        x (TYPE): the three-vector encoding the current rotational state of the A-EKF
    
    Returns:
        TYPE: the predicted gyro readings in IMU frame
    """
    r = x[:3,0]
    omega = x[3:, [0]]
    # print(omega)
    # print(omega.shape)

    zmodel = np.zeros((3,1))
    # print(zmodel.shape)

    # print(zmodel[3:,0])

    # g_vec_global = np.array([[0],[0],[1]])
    # print(g_vec_global.shape)
    R_AE = rotationMapRodrigues(r)

    
    # print(R_AE.transpose() @ g_vec_global)
    zmodel[:3,[0]] = R_AE.transpose() @ omega
    # print(zmodel[:3,[0]])
    # zmodel[3:,0] = np.array([[0],[0],[0]])
    


    return zmodel



def e_vec(n, q):
    """returns a unit basis vector
    
    Args:
        n (TYPE): the index that contains 1
        q (TYPE): the dimension of the unit vector
    
    Returns:
        TYPE: the unit basis vector
    """
    ret = np.zeros((q,1))
    ret[n,0]=1
    return ret

def calculateF_N(x, delta=0.000001):
    """Numerically calculates the gradient matrix of the states
    Approximates the effect of the changing states on the covariance matrix
    
    Args:
        x (TYPE): The current state
        delta (float, optional): the numerical delta of the state
    
    Returns:
        TYPE: the 6x6 state transition gradient matrix
    """
    # F_AE = np.zeros((6,6))
    F_AE = eye6

    block = np.zeros((6,3))

    # print('block')
    # print(block)
    fx0_AE = f(x,delta)
    for i in range(3,6):

        block[:,[i - 3]] = (f(x+e_vec(i,6)*delta,delta)-fx0_AE  )/(delta)

    # for i in range(6):
    #   F_AE[:,[i]] = (f(x+e_vec(i,6)*delta)-f(x-1*e_vec(i,6)*delta))/(2*delta)
    # print(block)
    F_AE[0:3,3:6] = block[0:3,:]
    # print(F_AE)
    return F_AE

def calculateH_N(x, delta=0.000001):
    """Numerically calculates the gradient matrix of the measurements wrt the states
    
    Args:
        x (TYPE): The current state
        delta (float, optional): the numerical delta of the state
    
    Returns:
        TYPE: the 6x6 measurement gradient matrix
    """
    H_AE = np.zeros((6,6))
    hx0_AE = h(x)
    for i in range(6):
        H_AE[:,[i]] = (h(x+e_vec(i,6)*delta)-hx0_AE)/(delta)

    # print(H_AE)
    return H_AE

def calculateH_N_gyroOnly(x, delta=0.000001):
    """Numerically calculates the gradient matrix of only the gyro measurements wrt the states
    
    Args:
        x (TYPE): The current state
        delta (float, optional): the numerical delta of the state
    
    Returns:
        TYPE: the 3x6 measurement gradient matrix
    """
    H_AE = np.zeros((3,6))
    hx0_AE = h_gyroOnly(x)
    for i in range(6):
        H_AE[:,[i]] = (h_gyroOnly(x+e_vec(i,6)*delta)-hx0_AE)/(delta)

    # print(H_AE[0:3,:])
    # return H_AE[0:3,:]
    # print(H_AE[0:3,:])
    return H_AE





def estimateStep_AE(x_prev_state_estimate_AE, P_prev_covar_estimate_AE, dt, Q_AE, isUpdateTime, i, F_AE_prev):
    """Performs the estimation/prediction step of the AE
    
    Args:
        x_prev_state_estimate_AE (TYPE): The previous state of the AE
        P_prev_covar_estimate_AE (TYPE): The previous covariance of the AE
        dt (TYPE): The time step
        Q_AE (TYPE): The process noise matrix
        isUpdateTime (bool): If we update the state transition matrix
        i (TYPE): The current iteration
        F_AE_prev (TYPE): The previous state transition matrix
    
    Returns:
        TYPE: the state estimate, the state transition matrix, and the covariance estimate
    """
    x_state_estimate_AE = f(x_prev_state_estimate_AE,dt)

    if isUpdateTime or (i == 1):
        F_AE = calculateF_N(x_prev_state_estimate_AE)
    else:
        F_AE = F_AE_prev

    P_covar_estimate_AE = (F_AE @ P_prev_covar_estimate_AE @ F_AE.transpose()) + Q_AE

    return (x_state_estimate_AE, F_AE, P_covar_estimate_AE)




def updateStep_AE_gyroOnly(x_state_estimate_AE, P_covar_estimate_AE,gyroVec_corrected,accelVec_corrected, R_gyroOnly_AE, isUpdateTime):
    """The update step of the AE, using only the gyro measurements
    
    Args:
        x_state_estimate_AE (TYPE): The state estimate of the AE
        P_covar_estimate_AE (TYPE): The covariance estimate of the AE
        gyroVec_corrected (TYPE): The gyro measurements
        accelVec_corrected (TYPE): The accelerometer measurements
        R_gyroOnly_AE (TYPE): The measurement noise matrix
        isUpdateTime (boolean): If we update the measurement gradient matrix
    

    """
    # R_measured_AE = rotationMapRodrigues(x_state_estimate_AE[:3,0].flatten())

    z_measured_AE = gyroVec_corrected
    z_measured_AE = z_measured_AE.reshape(-1,1)

    z_model_AE = h_gyroOnly(x_state_estimate_AE)

    y_residual_AE = z_measured_AE - z_model_AE;
    HTime0 = time()

    # if isUpdateTime or i == 1:
    # print('update H_AE')
    # input()
    H_AE = calculateH_N_gyroOnly(x_state_estimate_AE)

    HTime1 = time()

    R_eff_AE = R_gyroOnly_AE
    isUpdateR = True
    


    HTime = HTime1 - HTime0

    S_covariance_AE = H_AE @ P_covar_estimate_AE @ H_AE.transpose() + R_eff_AE
    K_gain_AE = P_covar_estimate_AE @ H_AE.transpose() @ np.linalg.inv(S_covariance_AE)
    x_state_update_AE = x_state_estimate_AE + K_gain_AE @ y_residual_AE

    z_measured_AE = np.concatenate((z_measured_AE ,np.zeros((3,1))  ))
    z_measured_AE = z_measured_AE.reshape(-1,1)

    z_model_AE = np.concatenate((z_model_AE,np.zeros((3,1))   ))
    z_model_AE = z_model_AE.reshape(-1,1)

    y_residual_AE = np.concatenate((y_residual_AE,np.zeros((3,1))   ))
    y_residual_AE = y_residual_AE.reshape(-1,1)
    P_covar_update_AE = (eye6 - K_gain_AE @ H_AE) @ P_covar_estimate_AE
    return (z_measured_AE, z_model_AE, y_residual_AE, x_state_update_AE, P_covar_update_AE, isUpdateR)





def updateStep_AE(x_state_estimate_AE, P_covar_estimate_AE,gyroVec_corrected,accelVec_corrected, R_AE, isUpdateTime, i, isUpdateR, H_AE_prev):
    """Performs the update step of the AE
    
    Args:
        x_state_estimate_AE (TYPE): The state estimate of the AE
        P_covar_estimate_AE (TYPE): The covariance estimate of the AE
        gyroVec_corrected (TYPE): The gyro measurements
        accelVec_corrected (TYPE): The accelerometer measurements
        R_gyroOnly_AE (TYPE): The measurement noise matrix
        isUpdateTime (boolean): If we update the measurement gradient matrix
        i (TYPE): The current iteration
        isUpdateR (TYPE): If we update the measurement noise matrix
        H_AE_prev (TYPE): The previous measurement gradient matrix
    

    """
    # R_measured_AE = rotationMapRodrigues(x_state_estimate_AE[:3,0].flatten())

    z_measured_AE = np.concatenate((gyroVec_corrected,accelVec_corrected))
    z_measured_AE = z_measured_AE.reshape(-1,1)

    z_model_AE = h(x_state_estimate_AE)
    y_residual_AE = z_measured_AE - z_model_AE;
    HTime0 = time()

    if isUpdateTime or i == 1 or isUpdateR:
        # print('update H_AE')
        # input()
        H_AE = calculateH_N(x_state_estimate_AE)
        isUpdateR = False
    else:
        H_AE = H_AE_prev

    HTime1 = time()

    R_eff_AE = R_AE

    HTime = HTime1 - HTime0
    # print('HTime: '+ str(HTime))

    # print('H_AE')
    # print(H_AE)

    
    S_covariance_AE = H_AE @ P_covar_estimate_AE @ H_AE.transpose() + R_eff_AE
    # print('S_covariance_AE')
    # print(S_covariance_AE)

    K_gain_AE = P_covar_estimate_AE @ H_AE.transpose() @ np.linalg.inv(S_covariance_AE)

    # print('K_gain_AE')
    # print(K_gain_AE)

    x_state_update_AE = x_state_estimate_AE + K_gain_AE @ y_residual_AE
    P_covar_update_AE = (eye6 - K_gain_AE @ H_AE) @ P_covar_estimate_AE

    return (z_measured_AE, z_model_AE, y_residual_AE, x_state_update_AE, P_covar_update_AE, isUpdateR, H_AE)


def extractEulerAngles(R_AE):
    """DOES NOT WORK
    
    Args:
        R_AE (TYPE): Description
    """

# ZYX

# phi == z
# theta == y
# psi == x


    if R_AE[2,0] != 1:
        theta1 = -np.arcsin(R_AE[2,0]);
        theta2 = np.pi - theta1 ;
        
        psi1 = np.arctan2(R_AE[2,1]/np.cos(theta1), R_AE[2,2]/np.cos(theta1));
        psi2 = np.arctan2(R_AE[2,1]/np.cos(theta2), R_AE[2,2]/np.cos(theta2));
        
        phi1 = np.arctan2(R_AE[1,0]/np.cos(theta1), R_AE[0,0]/np.cos(theta1));
        phi2 = np.arctan2(R_AE[1,0]/np.cos(theta2), R_AE[0,0]/np.cos(theta2));
        
        eulerAngles = np.array( [psi1,theta1,phi1] )
        
    else:
        
        phi = 0;
        
        if (R_AE[2,0] == -1):
            theta = np.pi/2;
            psi = phi + np.arctan2(R_AE[0,1],R_AE[0,2]);
            
        else:
            
            theta = -pi/2;
            psi = -phi + np.arctan2(-R_AE[0,1],-R_AE[0,2]);
            
        
        eulerAngles = np.array([psi,theta,phi]);
        


    return eulerAngles



def extractEulerAngles_new(R):
    """Extracts the roll-pitch-yaw Euler angles of a rotation matrix R
    In the order roll, then pitch, then yaw, intrinsic Euler angles
    (Analgously: Rx * Ry * Rz)
    
    Args:
        R (TYPE): a 3x3 rotation matrix
    
    Returns:
        TYPE: Euler angles in order: roll, pitch, yaw
    """
    theta = np.arctan2(-R[0,2],np.sign(R[2,2]) * np.sqrt(R[1,2]**2 + R[2,2]**2))
    psi = np.arctan2(R[1,2]/np.cos(theta),R[2,2]/np.cos(theta));
    phi = np.arctan2(R[0,1]/np.cos(theta),R[0,0]/np.cos(theta));

    eulerAngles = np.array([psi,theta,phi]);
        

    return eulerAngles

def extractEulerAngles_new_ZYX(R):
    """Extracts the roll-pitch-yaw Euler angles of a rotation matrix R
    In the order yaw, then pitch, then roll, intrinsic Euler angles
    (Analgously: Rz * Ry * Rx)
    
    Args:
        R (TYPE): a 3x3 rotation matrix
    
    Returns:
        TYPE: Euler angles in order: yaw, pitch, roll
    """
    neg_roll,neg_pitch,neg_yaw = extractEulerAngles_new(R.T)


    eulerAngles = np.array([-neg_yaw ,-neg_pitch,-neg_roll]);
    return eulerAngles







