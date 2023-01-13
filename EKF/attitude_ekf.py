""" Made by Gray. Contains the full EKF for attitute"""
from attitudeEstimatorEKFFuncs import *
from time import time


class AttitudeEKF():
    def __init__(self,sigma_gyro=0.0023, sigma_accel=0.0032, sigma_q_AE=1e2, Q_pos_scale=1e-10):

        self.x0 = np.zeros((6,1))
        self.P0 = eye6*1e-3
        # print('AttitudeEKF.x0')
        # print(self.x0)
        self.F0 = calculateF_N(self.x0)
        # print('AttitudeEKF.F0')
        # print(self.F0)
        self.H = calculateH_N(self.x0)

        self.Q0 = np.diag([sigma_q_AE**2,sigma_q_AE**2,sigma_q_AE**2])
        self.Q = np.diag([0,0,0,sigma_q_AE**2,sigma_q_AE**2,sigma_q_AE**2])

        self.R = np.diag([sigma_gyro**2,sigma_gyro**2,sigma_gyro**2,sigma_accel**2,sigma_accel**2,sigma_accel**2])
        self.R_gyroOnly = np.diag([sigma_gyro**2,sigma_gyro**2,sigma_gyro**2])
        self.Q_pos_scale = Q_pos_scale


        self.x_state_estimate = None
        self.P_covar_estimate = None
        self.F = None
        self.isUpdateTime = None
        self.isUpdateR = False

        self.accelNormCutoff = 1.15
        # self.accelNormCutoff = 35

        self.isUsingAccel = True

        #timing internal variables
        self.timing_step = 0
        self.timing_measure = 0
        self.timing_get_euler_angles = 0
        self.timing_get_useful_angles = 0

        # VICON Correction
        vicon_angle_exo_imu_z = -30
        co_vicon_exo_imu_z = np.cos(np.pi/180 * (vicon_angle_exo_imu_z))
        so_vicon_exo_imu_z = np.sin(np.pi/180 * (vicon_angle_exo_imu_z)) 

        vicon_angle_exo_imu_y = 0
        co_vicon_exo_imu_y = np.cos(np.pi/180 * (vicon_angle_exo_imu_y))
        so_vicon_exo_imu_y = np.sin(np.pi/180 * (vicon_angle_exo_imu_y)) 

        vicon_angle_exo_imu_x = 10
        co_vicon_exo_imu_x = np.cos(np.pi/180 * (vicon_angle_exo_imu_x))
        so_vicon_exo_imu_x = np.sin(np.pi/180 * (vicon_angle_exo_imu_x)) 

        R_vicon_correct_exo_imu_z = np.array([[co_vicon_exo_imu_z, -so_vicon_exo_imu_z, 0],[so_vicon_exo_imu_z, co_vicon_exo_imu_z, 0],[0, 0, 1]])
        R_vicon_correct_exo_imu_x = np.array([[1, 0, 0], [0, co_vicon_exo_imu_x, -so_vicon_exo_imu_x],[0, so_vicon_exo_imu_x, co_vicon_exo_imu_x]])
        R_vicon_correct_exo_imu_y = np.array([[co_vicon_exo_imu_y, 0, so_vicon_exo_imu_y], [0, 1, 0], [-so_vicon_exo_imu_y, 0, co_vicon_exo_imu_y]])

        self.R_vicon_correct_exo_imu = R_vicon_correct_exo_imu_z @ R_vicon_correct_exo_imu_x


    def step(self, i, dt, isUpdateTime=True):
        time0 = time()
        first=(i==0)

        if first:
            # print('1st step')
            (self.x_state_estimate, self.F, self.P_covar_estimate) = estimateStep_AE(
                self.x0, self.P0, 1/180.0,
                self.Q, isUpdateTime, i, self.F0)
        else:

            dt = np.max((dt,1e-3))
            self.Q = np.zeros((6,6))
            self.Q[:3,:3] = self.Q0*(dt**2)/(2 * (0.01)) * self.Q_pos_scale
            self.Q[3:,3:] = self.Q0*(dt)/((0.01))


            # print('%d%s step'%(i,["th","st","nd","rd","th", "th", "th", "th", "th", "th"][i%10] if not (i%100>10 and i%100<20) else "th"))
            (self.x_state_estimate, self.F, self.P_covar_estimate) = estimateStep_AE(
                self.x_prev_state_estimate, self.P_prev_covar_estimate, dt, 
                self.Q, isUpdateTime, i, self.F)

        time1 = time()
        self.timing_step = time1 - time0

    def measure(self, i, gyroVec_corrected, accelVec_corrected, isUpdateTime=True, CORRECT_VICON=True):
        time0 = time()

        if CORRECT_VICON:
            accelVec_corrected = self.R_vicon_correct_exo_imu @ accelVec_corrected
            gyroVec_corrected = self.R_vicon_correct_exo_imu @ gyroVec_corrected


        accelNorm = np.linalg.norm(accelVec_corrected)
        if accelNorm > self.accelNormCutoff:
            # print('gyro only')
            self.isUsingAccel = False
            (self.z_measured, self.z_model, self.y_residual, self.x_state_update,
                self.P_covar_update, self.isUpdateR) = updateStep_AE_gyroOnly(
                self.x_state_estimate, self.P_covar_estimate,
                gyroVec_corrected, accelVec_corrected, self.R_gyroOnly, isUpdateTime)

        #update with both the gyro and accel measurements
        else:
            # print('accel and gyro')
            self.isUsingAccel = True
            (self.z_measured, self.z_model, self.y_residual, self.x_state_update,
                self.P_covar_update, self.isUpdateR, self.H) = updateStep_AE(
                self.x_state_estimate, self.P_covar_estimate,
                gyroVec_corrected, accelVec_corrected, self.R, isUpdateTime,
                i, self.isUpdateR, self.H)

        self.x_prev_state_estimate = self.x_state_update
        self.P_prev_covar_estimate = self.P_covar_update

        time1 = time()
        self.timing_measure = time1 - time0

    def get_euler_angles(self):
        time0 = time()
        r_g_update = self.x_state_update[0:3]
        R_update = rotationMapRodrigues(r_g_update)

        psi = -np.arccos(np.dot(  np.array([0,1,0]) , R_update.transpose() @ np.array([0,0,1])  )) + np.pi/2
        theta = np.arccos(np.dot(  np.array([1,0,0]) , R_update.transpose() @ np.array([0,0,1])  )) - np.pi/2

        eulerAngles = (psi,theta,0)
        # eulerAngles = extractEulerAngles(R_update)

        time1 = time()
        self.timing_get_euler_angles = time1 - time0
        return eulerAngles


        
    def get_useful_angles(self, sideMultiplier=1):

        time0 = time()
        psi,theta,phi = self.get_euler_angles()

        r_g_update = self.x_state_update[0:3]
        R_update = rotationMapRodrigues(r_g_update)
       
        shank_angle = sideMultiplier * -1 * theta *180/np.pi

        # foot_axis = np.array([np.cos(ankleAngle * np.pi/180),0,np.sin(ankleAngle * np.pi/180)])

        # foot_angle = np.arcsin(np.dot(  foot_axis , R_update.transpose() @ np.array([0,0,1])  )) * 180/np.pi

        time1 = time()
        self.timing_get_useful_angles = time1 - time0
        # eulerAngles = extractEulerAngles(R_update)
        return shank_angle
