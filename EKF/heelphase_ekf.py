"""Contains the Heelphase backup plan class
"""
# from phaseEstimatorEKFFuncs import *
import time
import traceback
import csv
import numpy as np
from arctanMapFuncs import *
from time import time

from timing_based_estimator import TimingPhaseEstimator
from StatProfiler import StatProfiler
import matplotlib.pyplot as plt

stepprof = StatProfiler(name="heelphase.step")
gols_prof = StatProfiler(name="update_grays_OLS")
prof_opee = StatProfiler(name="override_phase_ekf_estimate")
cz_prof = StatProfiler(name="compute_z")
DECIMATION_FACTOR = 1
DEBUG_PLOTS=True

class HeelPhaseEKF():

    """A class that estimates "heel-phase" which fits a gait-state estimate to the past stride using least squares (LS)
    This acts as a backup in case the EKF gets lost, as the Heelphase (HP) Estimator can then reset the EKF estimates
    This class also contains a method used to detect heel-strike (HS) events
    
    Attributes:
        avg_phase_error_N (int): The number of data points used to compute the average phase error during the LS fitting
        avg_phase_error_N_TBE (int): The number of data points used to compute the average phase error in TBE
        avg_phase_error_sum (int): The cumulative average phase error for the EKF
        avg_phase_error_sum_TBE (int): The cumulative average phase error for the TBE
        gait_model (class): The kinematic gait model
        HP_prevStepStartTime (int): The start time of the previous step
        HP_stepStartTime (int): The start time of the current step
        HSDetected (bool): If a HS event was detected on the current iteration
        incline_estimate_HP (int): The heelphase estimate of the incline
        isOverriding (bool): If the heelphase will override the EKF
        list_of_lists_of_time (list): Convenience list used to perform LS
        list_of_lists_of_x_hp (list): Convenience list used to perform LS
        list_of_lists_of_x_tbe (list): Convenience list used to perform LS
        list_of_lists_of_z_model (list): Convenience list used to perform LS
        num_meas (int): the number of measurements used in the EKF and HP
        numStepsTaken (int): The number of steps detected during the trail
        phase_ekf (class): The EKF that estimates gait state
        prevHSDetected (bool): If HS was detected during the previous iteration
        prevStepDuration (int): The time duration of the previous step
        pseudoStrideLength_estimate_HP (int): The heelphase estimate of the pseudo stride length (before the arctan function)
        save_plotting_data (bool): Whether to save data for plotting
        SSE (float): The sum of squared errors of the state, usign the HP estimates of the state, norm'd using the measurement noise matrix R
        SSE_bias (int): An empirically derived number that acts as the threshold for HP to override EKF
        SSE_diffs (list): A list of the differences between the HP SSEs and the EKF SSEs
        stepDuration (float): The duration (in seconds) of the previous step
        strideLength_estimate_HP (float): The heelphase estimate of the stride length
        TBE (class): The timing-based estimator that estimates phase using the timings of the previous few steps
        time_enum (float): A timing for debugging and run-time characterization
        time_linalg (float): A timing for debugging and run-time characterization
        time_loop0 (float): A timing for debugging and run-time characterization
        time_loop1 (float): A timing for debugging and run-time characterization
        time_loop2 (float): A timing for debugging and run-time characterization
        time_sqrt (float): A timing for debugging and run-time characterization
        timing_compute_z (float): A timing for debugging and run-time characterization
        timing_computeSSE (float): A timing for debugging and run-time characterization
        timing_detectHS (float): A timing for debugging and run-time characterization
        timing_override_phase_ekf_estimate (float): A timing for debugging and run-time characterization
        timing_step (float): A timing for debugging and run-time characterization
        timing_update_grays_OLS (float): A timing for debugging and run-time characterization
        x_ekf_list (list): Convenience list used to perform LS
        x_tbe_list (list): Convenience list used to perform LS
        y_residual_HP (np matrix): The residuals between the measured data and the modeled measurements predicted from the HP gait state
        z_measured_list (list): A list of measured data for use in LS
        z_model_HP (TYPE): The modeled measurements predicted using the HP gait state
    
   
    """
    
    # And Error Quantifier
    def __init__(self, phase_ekf, Q, R, timing_based_estimator=None):
        """Initialize 
        
        Args:
            phase_ekf (TYPE): Description
            save_plotting_data (bool, optional): Description
            timing_based_estimator (None, optional): Description
        """
        self.phase_ekf = phase_ekf
        self.meas_config = self.phase_ekf.meas_config
        self.x0 = self.phase_ekf.x0[2:]
        self.phase_HP = self.phase_ekf.x0[0].item(0)
        self.phase_rate_HP = self.phase_ekf.x0[1].item(0)
        self.P0 = self.phase_ekf.P0[2:,2:]
        self.F0 = self.phase_ekf.F0[2:,2:]
        self.Q_rate = Q
        self.gait_model = self.phase_ekf.gait_model
        self.R = R


        self.stepDuration = 1/self.phase_rate_HP
        self.prevStepDuration = 1
        self.HSDetected = False
        self.prevHSDetected = False
        
        self.gait_model = self.phase_ekf.gait_model
        
        self.TBE = timing_based_estimator

        self.num_meas = 4
        if self.meas_config == 'full':
            self.num_meas = 6

        elif self.meas_config == 'heelForward' or self.meas_config == 'heelUp':
            self.num_meas = 5

        # print(self.num_meas)
        self.numStepsTaken = 0


        self.HP_stepStartTime = 0
        self.HP_prevStepStartTime = 0
        self.isOverriding = False
        self.SSE = 0.0
        self.SSE_bias = 1.2e3
        self.SSE_diffs = []

       
        self.avg_phase_error_sum = 0
        self.avg_phase_error_N = 0

        self.avg_phase_error_sum_TBE = 0
        self.avg_phase_error_N_TBE = 0


        #internal timing variables
        self.timing_step = 0
        self.timing_update_grays_OLS = 0
        self.timing_compute_z = 0
        self.timing_computeSSE = 0
        self.timing_override_phase_ekf_estimate = 0
        self.timing_detectHS = 0

        #MORE internal timing variables
        self.time_loop0 = 0
        self.time_loop1 = 0
        self.time_loop2 = 0
        self.time_linalg = 0
        self.time_enum = 0
        self.time_sqrt = 0

    def update_phase_rate(self, phase_rate_new):
        self.x_state[1] = phase_rate_new



    def step(self, i, t, dt, data, HSDetected, DO_OVERRIDES=True):
        """Summary
        
        Args:
            i (int): current iteration count
            dt: float, the time step
            data (np vector (N,)): the measured kinematics
        """
        time0 = time()
        # HSDetected is a boolean that is true if our HS sensor says we HS'd, and false otherwise
        self.HSDetected = HSDetected
        self.isOverriding = False

        #UPDATE TBE RELATED VARIABLES IF TBE EXISTS
        if self.TBE:
            self.TBE.computeTBEPhase(t)

        #detect a rising edge on the HS sensor, i.e. we HS'd
        if self.HSDetected and not self.prevHSDetected:
            self.numStepsTaken += 1
            self.HP_stepStartTime = t

            self.stepDuration = self.HP_stepStartTime - self.HP_prevStepStartTime
            if self.stepDuration == 0:
                self.stepDuration = 1
            #Reset phase and phase dot
            self.phase_HP = 0
            self.phase_rate_HP = 1/self.stepDuration

            if self.TBE:
                self.TBE.stepDuration = self.stepDuration
                self.TBE.prevStepDuration = self.prevStepDuration
                self.TBE.numStepsTaken = self.numStepsTaken
                self.TBE.stepDurations.append(self.TBE.stepDuration)
                self.TBE.stepStartTime = self.HP_stepStartTime
                self.TBE.prevStepStartTime = self.HP_prevStepStartTime
                self.TBE.computeAvgStrideTime()
        


        #PREDICT STEP

        first=(i==0)

        if first:
            # print('1st step')
            # F = np.array([[1, 1/100.0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
            Q = self.Q_rate * 1/100.0
            # self.x_state_estimate = F @ self.x0
            self.phase_HP = self.phase_rate_HP * 1/100 + self.phase_HP
            self.x_state = self.x0

            # self.P_covar = (F @ self.P0 @ F.transpose()) + Q
            self.P_covar = (self.P0) + Q

           
        else:
            # F = np.array([[1, dt,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
            Q = self.Q_rate * dt
            # self.x_state_estimate = F @ self.x_prev_state_estimate
            self.phase_HP = self.phase_rate_HP * dt + self.phase_HP
            # self.P_covar = (F @ self.P_prev_covar_estimate @ F.transpose()) + Q
            self.P_covar = (self.P_covar) + Q

        self.phase_HP = self.phase_HP % 1

        self.z_measured = data.reshape(-1,1)

        # Extract state vector elements
        strideLength_estimate = arctanMap(self.x_state[0].item(0))
        incline_estimate = self.x_state[1].item(0)

        # Compute the modeled foot and shank angles based on the regressed gait_model
        footAngle_estimate = self.gait_model.returnFootAngle(self.phase_HP,strideLength_estimate,incline_estimate)
        shankAngle_estimate = self.gait_model.returnShankAngle(self.phase_HP,strideLength_estimate,incline_estimate)

        #estimate modeled velocity by multiplying the phase rate by the partials wrt phase
        evalfootAngleDeriv_dphase = self.gait_model.returnFootAngleDeriv_dphase(self.phase_HP,strideLength_estimate,incline_estimate)
        evalshankAngleDeriv_dphase = self.gait_model.returnShankAngleDeriv_dphase(self.phase_HP,strideLength_estimate,incline_estimate)

        footAngleVel_estimate = self.phase_rate_HP * evalfootAngleDeriv_dphase
        shankAngleVel_estimate = self.phase_rate_HP * evalshankAngleDeriv_dphase
        self.z_model = [footAngle_estimate, footAngleVel_estimate, shankAngle_estimate, shankAngleVel_estimate]

        if self.meas_config == 'full' or self.meas_config == 'heelForward':
            heelForwardPos_estimate = self.gait_model.returnHeelPosForward(self.phase_HP,strideLength_estimate,incline_estimate)
            self.z_model.append(heelForwardPos_estimate)

        if self.meas_config == 'full' or self.meas_config == 'heelUp':
            heelUpPos_estimate = self.gait_model.returnHeelPosUp(self.phase_HP,strideLength_estimate,incline_estimate)
            self.z_model.append(heelUpPos_estimate)

        self.z_model = np.array(self.z_model).reshape(-1,1)
        self.y_residual = self.z_measured - self.z_model

        self.SSE = (self.y_residual.T @ np.linalg.solve(self.R, self.y_residual) ).item(0) + self.SSE



        dsLdPsL = dArctanMap(self.x_state[0].item(0))

        # calculate the H matrix for the phase estimator, just for stride length and ramp
        # H is the Jacobian of the measurements wrt the state vector

        dh13 = dsLdPsL * self.gait_model.returnFootAngleDeriv_dsL(self.phase_HP,strideLength_estimate,incline_estimate)
        dh14 = self.gait_model.returnFootAngleDeriv_dincline(self.phase_HP,strideLength_estimate,incline_estimate)

        dh23 = dsLdPsL * self.phase_rate_HP * self.gait_model.returnFootAngle2ndDeriv_dphasedsL(self.phase_HP,strideLength_estimate,incline_estimate)
        dh24 = self.phase_rate_HP * self.gait_model.returnFootAngle2ndDeriv_dphasedincline(self.phase_HP,strideLength_estimate,incline_estimate)

        dh33 = dsLdPsL * self.gait_model.returnShankAngleDeriv_dsL(self.phase_HP,strideLength_estimate,incline_estimate)
        dh34 = self.gait_model.returnShankAngleDeriv_dincline(self.phase_HP,strideLength_estimate,incline_estimate)

        dh43 = dsLdPsL * self.phase_rate_HP * self.gait_model.returnShankAngle2ndDeriv_dphasedsL(self.phase_HP,strideLength_estimate,incline_estimate)
        dh44 = self.phase_rate_HP * self.gait_model.returnShankAngle2ndDeriv_dphasedincline(self.phase_HP,strideLength_estimate,incline_estimate)

        H = np.array([[dh13,dh14],
            [dh23,dh24],
            [dh33,dh34],
            [dh43,dh44],
            ])

        if self.meas_config == 'full' or self.meas_config == 'heelForward':
            #heel accelrow
            dh53 = dsLdPsL * self.gait_model.returnHeelPosForwardDeriv_dsL(self.phase_HP,strideLength_estimate,incline_estimate)
            dh54 = self.gait_model.returnHeelPosForwardDeriv_dincline(self.phase_HP,strideLength_estimate,incline_estimate)

            H = np.vstack((H, np.array([[dh53,dh54]])))

        if self.meas_config == 'full' or self.meas_config == 'heelUp':
            #tibia accelrow
            dh63 = dsLdPsL * self.gait_model.returnHeelPosUpDeriv_dsL(self.phase_HP,strideLength_estimate,incline_estimate)
            dh64 = self.gait_model.returnHeelPosUpDeriv_dincline(self.phase_HP,strideLength_estimate,incline_estimate)
            H = np.vstack((H, np.array([[dh63,dh64]])))


        #update the measurement covariance
        S_covariance = H @ self.P_covar @ H.transpose() + self.R

        # Compute Kalman Gain
        K_gain = self.P_covar @ H.transpose() @ np.linalg.inv(S_covariance)

        self.x_state = self.x_state + K_gain @ self.y_residual

        # Modulo phase to be between 0 and 1
        self.phase_HP = self.phase_HP % 1
        
        # Update covariance
        self.P_covar = (np.eye(2) - K_gain @ H) @ self.P_covar


        

        #detect a rising edge on the HS sensor, i.e. we HS'd
        if self.HSDetected and not self.prevHSDetected:
            # self.numStepsTaken += 1
            # self.HP_stepStartTime = t

            # self.stepDuration = self.HP_stepStartTime - self.HP_prevStepStartTime
            # #Reset phase and phase dot
            # self.phase_HP = 0
            # self.phase_rate_HP = 1/self.stepDuration

            if self.phase_ekf.SSE - self.SSE > self.SSE_bias and DO_OVERRIDES:
                # print(self.stepDuration)
                
                self.override_phase_ekf_estimate(self.phase_HP, self.phase_rate_HP, strideLength_estimate, incline_estimate)
            
            self.SSE = 0.0
            self.phase_ekf.set_SSE(0)
            self.HP_prevStepStartTime = self.HP_stepStartTime
            self.prevStepDuration = self.stepDuration

        self.prevHSDetected = self.HSDetected


        time1 = time()
        self.timing_update = time1 - time0




    def override_phase_ekf_estimate(self, phase, phase_dot, stride_length, incline):
        """Overrides the EKF gait state estimate
        
        Args:
            phase (float): The phase to override with
            phase_dot (float): The phase rate to override with
            stride_length (float): The stride length to override with
            incline (float): The incline to override with
        """
        time0 = time()
        prof_opee.tic()
        print('override HPEKF')
        self.isOverriding = True
        # self.phase_ekf.set_y_residual(self.y_residual_HP)
        self.phase_ekf.set_x_state_update(np.array([phase,phase_dot,invArctanMap(stride_length),incline]))
        # self.phase_ekf.set_x_state_update(np.array([[self.self.phase_HP_HP],[self.self.phase_rate_HP_HP],[self.strideLength_estimate_HP],[self.incline_estimate_HP]]))

        self.phase_ekf.set_P_covar_update(1e-3 * np.eye(4))

        self.phase_ekf.set_prev_x_state_estimate(self.phase_ekf.x_state_update)
        self.phase_ekf.set_prev_P_covar_estimate(self.phase_ekf.P_covar_update)

        time1 = time()
        self.timing_override_phase_ekf_update = time1 - time0

        prof_opee.toc()



