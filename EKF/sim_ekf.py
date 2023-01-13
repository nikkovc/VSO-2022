""" Simulates the phase estimator ekf using loaded data. """
import numpy as np
from time import strftime
np.set_printoptions(precision=4)
import time
import gc

import matplotlib
# matplotlib.use('QtAgg')
import matplotlib.pyplot as plt

from attitude_ekf import AttitudeEKF
from phase_ekf import PhaseEKF
from arctanMapFuncs import *
from heelphase import HeelPhaseEstimator
from heelphase_ekf import HeelPhaseEKF
from gait_model import GaitModel_Bezier, GaitModel_Fourier
from ekf_torque_profile import TorqueProfile
from measurement_noise_model import MeasurementNoiseModel
from timing_based_estimator import TimingPhaseEstimator
# import pandas as pd
from filter_classes import FirstOrderLowPassLinearFilter, FirstOrderHighPassLinearFilter, GenericLinearFilter
# from pyEKFboost import GaitModel, TorqueProfile
from attitudeEstimatorEKFFuncs import extractEulerAngles_new
from heel_strike_detector import HeelStrikeDetector

DO_KIDNAPPING =False
DO_OVERRIDES = True
UPDATE_OLS=True
DO_TBE = False
DO_NEW_HS_DETECTOR = True
DO_SIM_ATTITUDE_EST = False
sideMultiplier = -1

PLOT_EXO_IMU = False
PLOT_TIMING_INFORMATION = False
PLOT_MEASURED = True
PLOT_AHRS = False
SHOW_FULL_STATE = True
PLOT_SIM_FOOT_POS = True
DO_FILT_ON_POS_FORWARD = not True
USE_C = not True
SPOOF_SENSORS = False

DO_FILT_ON_POS_UP = not True
DO_GUARDRAILS = not True


gc.disable()

if USE_C:
    from C_Wrapper import *

def get_std(cov):
    """Get square root of diagonals (standard deviations) from covariances 
    
    Args:
        cov (np matrix): a stack (in the third dimension) of positive definite covariances
    
    Returns:
        stdevs (np matrix): an array of standard deviations
    """
    
    d, d, N = cov.shape
    std_devs = np.zeros((N, d), dtype=float)
    for ii in range(N):
        std_devs[ii, :] = np.sqrt(np.diag(cov[:, :, ii]))
    return std_devs


def main():
    filename = 'AB01/20220323-22_AE0629Standalone_PB_EKF_Test_AB01_Forward_02.csv' #gets lost, pauses for a long time, use to tune HSDetector
    # filename = 'AB01/20220323-22_AE2215Standalone_PB_EKF_Test_AB01_Backward_03.csv' #pauses, use to tune HSDetector
    # filename = 'AB03/20220328-21_AE1509Standalone_PB_EKF_Test_AB03_Forward_02.csv'
    # filename = 'AB03/20220328-21_AE3436Standalone_PB_EKF_Test_AB03_Forward03.csv'
    # filename = "AB03/20220328-21_AE5105Standalone_PB_EKF_Test_AB03_Reverse_04.csv" #use this to tune gains, most unstable one
    # filename = "AB04/20220328-22_AE3220Standalone_PB_EKF_Test_AB04_Reverse_Tuning.csv"#long pauses
    # filename = 'AB04/20220328-23_AE0413Standalone_PB_EKF_Test_AB04ReverseTuning_02.csv'
    # filename = 'AB04/20220328-22_AE5753Standalone_PB_EKF_Test_AB04ForwardTuning_02.csv'
    # filename = 'AB05/20220404-20_AE0058Standalone_PB_EKF_Test_AB05_TPVB_04.csv'
    filename = 'wavefield_2.csv'


    data = np.loadtxt(filename, delimiter=',') 
    
    N_data = data.shape[0]
    print(N_data)

    # Read in AHRS R data
    # df_R_mat = pd.read_csv('20210713-10_AE0754_R_matrix_log.csv')
    # r00 = df_R_mat['r00'].to_numpy()
    # r01 = df_R_mat['r01'].to_numpy()
    # r02 = df_R_mat['r02'].to_numpy()
    # r10 = df_R_mat['r10'].to_numpy()
    # r11 = df_R_mat['r11'].to_numpy()
    # r12 = df_R_mat['r12'].to_numpy()
    # r20 = df_R_mat['r20'].to_numpy()
    # r21 = df_R_mat['r21'].to_numpy()
    # r22 = df_R_mat['r22'].to_numpy()

    attitude_ekf_args = {'sigma_gyro':0.0023,
                        'sigma_accel': 0.0032,
                        'sigma_q_AE':1e2,
                        'Q_pos_scale':1e-10}

    gait_model_covar_path = f'GaitModel/covar_fourier_normalizedsL_linearsL.csv'

    # attitude_ekf=AttitudeEKF(**attitude_ekf_args)

    # torque_profile = TorqueProfile('TorqueProfile/torqueProfileCoeffs_dataport3P.csv')
    # gait_model = GaitModel_Fourier('GaitModel/gaitModel_fourier_normalizedsL_linearsL.csv',phase_order=20, stride_length_order=1, incline_order=1)



    #HETEROSCEDASTIC VELOCITY ON
    #stable
    # sigma_foot = 1
    # sigma_shank = 7

    # sigma_foot_vel = 10
    # sigma_shank_vel = 20

    # sigma_heel_pos_forward = 0.01 #m
    # sigma_heel_pos_up = 0.08 #m

    sigma_foot = 1
    sigma_shank = 7

    sigma_foot_vel = 10
    sigma_shank_vel = 20

    sigma_heel_pos_forward = 0.01 #m
    sigma_heel_pos_up = 0.08 #m

    meas_config = 'full'

    # #FULL
    R_meas = np.diag([sigma_foot**2,
        sigma_foot_vel**2,\
        sigma_shank**2,
        sigma_shank_vel**2,\
        sigma_heel_pos_forward**2, 
        sigma_heel_pos_up**2,
        ])

    #STABLE
    # sigma_q_phase=0
    # sigma_q_phase_dot=6e-4
    # sigma_q_sL=9e-4
    # sigma_q_incline=6e-3

    sigma_q_phase=0
    sigma_q_phase_dot=1e-3
    sigma_q_sL=2e-3
    sigma_q_incline=5e-2




    # phase_ekf = PhaseEKF(**phase_ekf_args)

    if (USE_C):

        attitude_ekf=AttitudeEKF_C(**attitude_ekf_args)

        torque_profile = TorqueProfile_C('TorqueProfile/torqueProfileCoeffs_dataport3P.csv')
        gait_model = GaitModel_C('GaitModel/gaitModel_fourier_normalizedsL_linearsL.csv',phase_order=20, stride_length_order=1, incline_order=1, isBezier=False)


        measurement_noise_model = MeasurementNoiseModel_C(R_meas, gait_model_covar_path, meas_config=meas_config,DO_XSUB_R=True)
        phase_ekf_args = {'gait_model':gait_model,
                'torque_profile':torque_profile,
                'measurement_noise_model':measurement_noise_model,
                'CANCEL_RAMP':False,
                'BOOST_BANDWIDTH':False,
                'sigma_q_phase':sigma_q_phase,
                'sigma_q_phase_dot':sigma_q_phase_dot,
                'sigma_q_sL':sigma_q_sL,
                'sigma_q_incline':sigma_q_incline
                }

        phase_ekf = PhaseEKF_C(**phase_ekf_args)
        
    else:

        attitude_ekf=AttitudeEKF(**attitude_ekf_args)

        torque_profile = TorqueProfile('TorqueProfile/torqueProfileCoeffs_dataport3P.csv')
        gait_model = GaitModel_Fourier('GaitModel/gaitModel_fourier_normalizedsL_linearsL.csv',phase_order=20, stride_length_order=1, incline_order=1)


        measurement_noise_model = MeasurementNoiseModel(R_meas, gait_model_covar_path, meas_config=meas_config,DO_XSUB_R=True)
        phase_ekf_args = {'gait_model':gait_model,
                'torque_profile':torque_profile,
                'measurement_noise_model':measurement_noise_model,
                'CANCEL_RAMP':False,
                'BOOST_BANDWIDTH':False,
                'sigma_q_phase':sigma_q_phase,
                'sigma_q_phase_dot':sigma_q_phase_dot,
                'sigma_q_sL':sigma_q_sL,
                'sigma_q_incline':sigma_q_incline,
                'DO_GUARDRAILS':DO_GUARDRAILS
                }

        phase_ekf = PhaseEKF(**phase_ekf_args)



    if DO_TBE:
        timing_based_estimator = TimingPhaseEstimator()
    else:
        timing_based_estimator = None


    #INITIALIZE BACKUP EKF
    #velocity het off
    # Q_HP = np.diag([5e-5,1e-3])
    # R_HP = phase_ekf.R_mean

    #Velocity het on
    Q_HP = np.diag([1e-4,5e-3])
    R_HP = phase_ekf.R_mean
    
    heelphase_ekf=HeelPhaseEKF(phase_ekf, Q_HP, R_HP, timing_based_estimator=timing_based_estimator)

    #set up the filters for the gyro and accel signals
    fc_gyro = 5
    gyroYLowPassFilter = FirstOrderLowPassLinearFilter(fc=fc_gyro,dt=1/100)

    fc_accel = 20
    accelZHighPassFilter = FirstOrderHighPassLinearFilter(fc=fc_accel,dt=1/100)

    
    #SET UP Filters
    ω = 0.5 * np.pi*2
    ζ= 0.9

    A = np.array([
        [0,             1        ]      ,
        [-ω**2,         -2*ω*ζ   ]])

    # C = np.array([[-ω**2,         -2*ω*ζ   ]])
    C = np.array([[1,         0  ]])
    B = np.array([[0, 1]]).T
    # D = np.array([[1.0]])
    D = np.array([[0]])
    HPF_X0 = 0.0*np.ones((A.shape[0],1))

    heelPosForwardFilter = GenericLinearFilter(A, B, C, D, HPF_X0)


    ω = 0.5 * np.pi*2
    ζ= 0.9

    A = np.array([
        [0,             1        ]      ,
        [-ω**2,         -2*ω*ζ   ]])

    # C = np.array([[-ω**2,         -2*ω*ζ   ]])
    C = np.array([[1,         0  ]])
    B = np.array([[0, 1]]).T
    # D = np.array([[1.0]])
    D = np.array([[0]])

    HPF_X0 = 0.0*np.ones((A.shape[0],1))
    heelPosUpFilter = GenericLinearFilter(A, B, C, D, HPF_X0)
    MAX_TIME_STEP_INTEGRATE = 0.06

    #set up HSDetector
    HS_analysis_window = 10
    HSDetector = HeelStrikeDetector(HS_analysis_window)

    if 'AB04' or 'AB01' in filename:
        SUBJECT_LEG_LENGTH = 1.778/2
    elif 'AB03' in filename:
        SUBJECT_LEG_LENGTH = 0.965
    elif 'AB05' in filename:
        SUBJECT_LEG_LENGTH = 1.7272/2 #verify this
    # SUBJECT_LEG_LENGTH = 1.854/2
    

    plot_data = []
    plot_data_heelphase = []
    plot_data_measured = []
    plot_data_timing_attitude_ekf = []
    plot_data_timing_phase_ekf = []
    plot_data_timing_heelphase = []
    plot_data_imu = []
    plot_data_TBE = []
    plot_data_HPEKF = []

    plot_states = []

    state_std_devs = []

    prev=0


    #INITIALIZE STORAGE FOR COVARS
    P_covars = np.zeros((4, 4, N_data))

    tic = time.time()


    #EXTRACT ACT VARIABLES
    timeSec_vec_hardware=data[:,0]
    dt_vec_hardware = np.diff(timeSec_vec_hardware)
    accelVec_corrected_vec_hardware=data[:,1:4]
    gyroVec_corrected_vec_hardware=data[:,4:7]

    ankleAngle_vec_hardware = data[:,44]
    isOverriding_vec_hardware = data[:,73]
    roll_vec_hardware = data[:,58]
    pitch_vec_hardware = data[:,59]
    yaw_vec_hardware = data[:,60]

    x_state_PE_vec_hardware = data[:,25:29]
    z_measured_vec_hardware = data[:,29:35]
    z_model_vec_hardware = data[:,35:41]

    phase_vec_hardware = x_state_PE_vec_hardware[:,0]
    phase_rate_vec_hardware = x_state_PE_vec_hardware[:,1]
    incline_vec_hardware = x_state_PE_vec_hardware[:,3]



    HSDetected_vec_hardware = data[:,24]
    strideLength_vec_hardware = data[:,45]

    heelPosForward_meas_filt_vec_hardware = data[:,62] #93
    heelPosUp_meas_filt_vec_hardware = data[:,63] #93

    actTorque_vec_hardware = data[:,49]
    desTorque_vec_hardware = data[:,50]

    heelAccForward_meas_fromDeltaVelocity_vec_hardware = data[:,70] #92
    heelAccSide_meas_fromDeltaVelocity_vec_hardware = data[:,71] #70
    heelAccUp_meas_fromDeltaVelocity_vec_hardware = data[:,72]#71

    heelAccForward_meas_fromDeltaVelocity_norm_vec_hardware = np.sqrt(heelAccForward_meas_fromDeltaVelocity_vec_hardware**2 +
                                                            heelAccSide_meas_fromDeltaVelocity_vec_hardware**2 +
                                                            (heelAccUp_meas_fromDeltaVelocity_vec_hardware)**2)


    for i,x in enumerate(data[:]):

        timeSec=x[0]
        dt = timeSec-prev

        prev=timeSec
        accelVec_corrected=x[1:4]
        gyroVec_corrected=x[4:7]
        psi_hardware, theta_hardware, phi_hardware=x[7:10]
        shankAngle_meas = x[22]
        footAngle_meas = x[23]

        # R = np.array([[r00[i],r01[i],r02[i]],[r10[i],r11[i],r12[i]],[r20[i],r21[i],r22[i]]])

        # R_foot = R_sensor_to_foot @ R @ R_y_correct
        # print('R_foot')
        # print(R_foot)

        # print('R_foot normal')
        # roll, pitch, yaw = extractEulerAngles_new(R_foot)
        roll_ahrs_hardware = x[58]
        pitch_ahrs_hardware = x[59]
        yaw_ahrs_hardware = x[60]
        x_state = x[25:29]
        # if i == 0:
        #   print('setting ICs')
        #   phase_ekf.x0 = x_state.reshape(-1,1)
        #   print(phase_ekf.x0)

        z_measured_act = x[29:35]
        HSDetected_hardware = x[24]
        isOverriding_hardware = x[73]
        accelZ = accelVec_corrected[2]
        gyroY = gyroVec_corrected[1]
        shankAngleVel_meas = z_measured_act[3]
        footAngleVel_meas = z_measured_act[1]
        strideLength_update_descaled_hardware = x[45]

        heelPosForward_meas_filt_hardware = x[62] #93
        heelPosUp_meas_filt_hardware = x[63] #93

        heelAccForward_meas_fromDeltaVelocity = x[70] #92
        heelAccSide_meas_fromDeltaVelocity = x[71] #70
        heelAccUp_meas_fromDeltaVelocity = x[72]#71

        heelAccForward_meas_norm = np.sqrt(heelAccForward_meas_fromDeltaVelocity**2 +
                                                            heelAccSide_meas_fromDeltaVelocity**2 +
                                                            (heelAccUp_meas_fromDeltaVelocity)**2)



        if DO_NEW_HS_DETECTOR:

            gyroY_filter = gyroYLowPassFilter.step(i, gyroY)
            accelZ_filter = accelZHighPassFilter.step(i, accelZ)
            HSDetected_sim = HSDetector.detectHS(timeSec, footAngle_meas, footAngleVel_meas,  heelAccForward_meas_norm)

        else:
            HSDetected_sim = HSDetected_hardware

        if DO_SIM_ATTITUDE_EST:
            updateFHfreq = 20
            isUpdateTime = (timeSec % 1/updateFHfreq  < 1e-2)
            attitude_ekf.step(i, dt, isUpdateTime=isUpdateTime)
            attitude_ekf.measure(i, gyroVec_corrected, accelVec_corrected, isUpdateTime=isUpdateTime, CORRECT_VICON=False)
            psi_sim, theta_sim, phi_sim = attitude_ekf.get_euler_angles()
            shankAngle_sim = attitude_ekf.get_useful_angles(ankleAngle)
            shankAngle_meas = shankAngle_sim

            psi_imu = psi_sim
            theta_imu = theta_sim
            phi_imu = phi_sim
        else:
            psi_imu = psi_hardware
            theta_imu = theta_hardware
            phi_imu = phi_hardware

        phase_ekf.step(i,dt)

        if DO_KIDNAPPING and i % 1000 == 0 and i != 0:
            print(timeSec)
            temp_estimate = np.zeros(4)
            temp_estimate[0] = np.random.uniform(0,1)
            temp_estimate[1] = np.random.uniform(-1,1)
            temp_estimate[2] = np.random.uniform(-5,5)
            temp_estimate[3] = np.random.uniform(-10,10)
            phase_ekf.set_x_state_estimate(temp_estimate)

        
       
        heelPosForward_meas_filt = heelPosForward_meas_filt_hardware
        heelPosUp_meas_filt = heelPosUp_meas_filt_hardware

        if DO_FILT_ON_POS_FORWARD:
            dt_int = np.min([dt, MAX_TIME_STEP_INTEGRATE])
            states_heelPosForward, heelPosForward_meas_filt = heelPosForwardFilter.step(i, dt_int, heelAccForward_meas_fromDeltaVelocity)
            heelPosForward_meas_filt = heelPosForward_meas_filt[0,0]

            # heelPosForward_meas_filt = np.max([np.min([heelPosForward_meas_filt, 0.4]), -0.25])

        if DO_FILT_ON_POS_UP:
            states_heelPosUp, heelPosUp_meas_filt = heelPosUpFilter.step(i, dt_int, heelAccUp_meas_fromDeltaVelocity)
            heelPosUp_meas_filt = heelPosUp_meas_filt[0,0]
            # heelPosUp_meas_filt = np.max([np.min([heelPosUp_meas_filt, 0.2]), -0.16])


        #Fake ideal accel signals
        if SPOOF_SENSORS:
            # phase_estimate = phase_ekf.x_state_estimate[0,0]
            # phase_dot_estimate = phase_ekf.x_state_estimate[1,0]
            phase_estimate = x_state[0]
            phase_dot_estimate = x_state[1]
            pseudoStrideLength_estimate = phase_ekf.x_state_estimate[2,0]
            # strideLength_estimate = arctanMap(pseudoStrideLength_estimate)
            strideLength_estimate = 1.3/SUBJECT_LEG_LENGTH
            incline_estimate = phase_ekf.x_state_estimate[3,0]

            footAngle_meas = gait_model.returnFootAngle(phase_estimate,strideLength_estimate,incline_estimate)
            shankAngle_meas = gait_model.returnShankAngle(phase_estimate,strideLength_estimate,incline_estimate)
            heelPosForward_meas_filt = gait_model.returnHeelPosForward(phase_estimate,strideLength_estimate,incline_estimate)
            heelPosUp_meas_filt = gait_model.returnHeelPosUp(phase_estimate,strideLength_estimate,incline_estimate)

            #estimate modeled velocity by multiplying the phase rate by the partials wrt phase
            evalfootAngleDeriv_dphase = gait_model.returnFootAngleDeriv_dphase(phase_estimate,strideLength_estimate,incline_estimate)
            evalshankAngleDeriv_dphase = gait_model.returnShankAngleDeriv_dphase(phase_estimate,strideLength_estimate,incline_estimate)

            footAngleVel_meas = phase_dot_estimate * evalfootAngleDeriv_dphase
            shankAngleVel_meas = phase_dot_estimate * evalshankAngleDeriv_dphase


        z_measured_sim = np.array([footAngle_meas, footAngleVel_meas, shankAngle_meas, shankAngleVel_meas, heelPosForward_meas_filt, heelPosUp_meas_filt])
        phase_ekf.update(i, dt, z_measured_sim)

        heelphase_ekf.step(i, timeSec, dt, z_measured_sim, HSDetected_sim, DO_OVERRIDES=DO_OVERRIDES)
        
        strideLength_update_descaled_sim = arctanMap(phase_ekf.x_state_update[2].item(0))

        #scale strideLength by subject height
        strideLength_update_descaled_sim = SUBJECT_LEG_LENGTH * strideLength_update_descaled_sim

        P_covars[:, :, i] = phase_ekf.P_covar_update
        arctanFromPos = np.arctan2(heelPosUp_meas_filt, heelPosForward_meas_filt) * 180/np.pi

        plot_data.append([
            timeSec, #0
            phase_ekf.x_state_update[0].item(0), #1
            phase_ekf.x_state_update[1].item(0), #2
            strideLength_update_descaled_sim, #3
            phase_ekf.x_state_update[3].item(0), #4
            phase_ekf.SSE, #5
            phase_ekf.get_torque(), #6
            HSDetected_sim, #7
            ])

        plot_data_timing_attitude_ekf.append([
            timeSec,
            attitude_ekf.timing_step, 
            attitude_ekf.timing_measure,
            attitude_ekf.timing_get_euler_angles,
            attitude_ekf.timing_get_useful_angles])

        plot_data_timing_phase_ekf.append([
            timeSec,
            phase_ekf.timing_step,
            phase_ekf.timing_measure,
            phase_ekf.timing_update,
            phase_ekf.timing_gain_schedule_R])

        plot_data_measured.append([
            timeSec, #0
            phase_ekf.z_model[0].item(0),#1
            phase_ekf.z_model[1].item(0),#2
            phase_ekf.z_model[2].item(0),#3
            phase_ekf.z_model[3].item(0),#4
            phase_ekf.z_model[4].item(0),#5
            phase_ekf.z_model[5].item(0),#6
            z_measured_sim[0], #7
            z_measured_sim[1], #8
            z_measured_sim[2], #9
            z_measured_sim[3],#10
            z_measured_sim[4],#11
            z_measured_sim[5],#12
            phase_ekf.z_model[0].item(0) - 2*np.sqrt(phase_ekf.R[0,0]),#13
            phase_ekf.z_model[0].item(0) + 2*np.sqrt(phase_ekf.R[0,0]), #14
            phase_ekf.z_model[1].item(0) - 2*np.sqrt(phase_ekf.R[1,1]),#15
            phase_ekf.z_model[1].item(0) + 2*np.sqrt(phase_ekf.R[1,1]), #16
            phase_ekf.z_model[2].item(0) - 2*np.sqrt(phase_ekf.R[2,2]),#17
            phase_ekf.z_model[2].item(0) + 2*np.sqrt(phase_ekf.R[2,2]), #18
            phase_ekf.z_model[3].item(0) - 2*np.sqrt(phase_ekf.R[3,3]),#19
            phase_ekf.z_model[3].item(0) + 2*np.sqrt(phase_ekf.R[3,3]), #20
            phase_ekf.z_model[4].item(0) - 2*np.sqrt(phase_ekf.R[4,4]), #21
            phase_ekf.z_model[4].item(0) + 2*np.sqrt(phase_ekf.R[4,4]), #22
            phase_ekf.z_model[5].item(0) - 2*np.sqrt(phase_ekf.R[5,5]), #23
            phase_ekf.z_model[5].item(0) + 2*np.sqrt(phase_ekf.R[5,5]), #24
            arctanFromPos])

        # plot_states.append([
        #     states_heelPosForward[0,0],
        #     states_heelPosForward[1,0],
        #     states_heelPosUp[0,0],
        #     states_heelPosUp[1,0],
        #     ])

        plot_data_HPEKF.append([
            timeSec, 
            heelphase_ekf.isOverriding,\
            heelphase_ekf.phase_HP,
            heelphase_ekf.phase_rate_HP,
            SUBJECT_LEG_LENGTH * arctanMap(heelphase_ekf.x_state[0,0]),
            heelphase_ekf.x_state[1,0],
            heelphase_ekf.SSE
            ])

        state_std_devs.append([
            phase_ekf.x_state_update[0].item(0) - 2*np.sqrt(phase_ekf.P_covar_update[0,0]),#0
            phase_ekf.x_state_update[0].item(0) + 2*np.sqrt(phase_ekf.P_covar_update[0,0]),#1
            phase_ekf.x_state_update[1].item(0) - 2*np.sqrt(phase_ekf.P_covar_update[1,1]),#2
            phase_ekf.x_state_update[1].item(0) + 2*np.sqrt(phase_ekf.P_covar_update[1,1]),#3
            strideLength_update_descaled_sim - 2*np.sqrt(phase_ekf.P_covar_update[2,2]),#4
            strideLength_update_descaled_sim + 2*np.sqrt(phase_ekf.P_covar_update[2,2]),#5
            phase_ekf.x_state_update[3].item(0) - 2*np.sqrt(phase_ekf.P_covar_update[3,3]),#6
            phase_ekf.x_state_update[3].item(0) + 2*np.sqrt(phase_ekf.P_covar_update[3,3]),#7
            ])


        if DO_TBE:
            phase_estimate_TBE = timing_based_estimator.phase_estimate_TBE
            plot_data_TBE.append([
                timeSec, 
                phase_estimate_TBE,
                timing_based_estimator.stepDuration,
                timing_based_estimator.timeStrideMean])

    toc = time.time()
    print(f"Ran simulation loop in {toc - tic:0.4f} seconds")

    plot_data = np.array(plot_data)

    plot_data_timing_attitude_ekf = np.array(plot_data_timing_attitude_ekf)
    plot_data_timing_phase_ekf = np.array(plot_data_timing_phase_ekf)
    plot_data_measured = np.array(plot_data_measured)
    plot_data_TBE = np.array(plot_data_TBE)
    plot_states = np.array(plot_states)
    plot_data_HPEKF = np.array(plot_data_HPEKF)

    state_std_devs = np.array(state_std_devs)

    # state_std_devs = get_std(P_covars)

    # print(plot_data_timing_heelphase)

    # print sampling rate

    print('Sampling Rate')
    sample_rate = 1/np.mean(np.diff(data[:,0]))
    print(sample_rate)


    
    if SHOW_FULL_STATE:
        fig, axs = plt.subplots(5,1,sharex=True,figsize=(10,6))

        axs[0].plot(timeSec_vec_hardware, phase_vec_hardware, label=r"$phase_{hardware}$")
        axs[0].plot(plot_data[:,0], plot_data[:,1],'b', label=r"$phase_{sim}$")
        # axs[0].fill_between(plot_data[:,0], state_std_devs[:,0],  state_std_devs[:,1], color='blue', alpha=0.3)
        axs[0].plot(plot_data[:,0], state_std_devs[:,0],'b--')
        axs[0].plot(plot_data[:,0], state_std_devs[:,1],'b--')
        axs[0].plot(plot_data_HPEKF[:,0], plot_data_HPEKF[:,2], label=r"$phase_{hpekf}$")

        if DO_TBE:
            axs[0].plot(plot_data[:,0], plot_data_TBE[:,1], label=r"$phase_{TBE}$") 

        # axs[0].plot(timeSec_vec_hardware, HSDetected_vec_hardware, label=r"$HSDetected, hardware$")
        axs[0].plot(plot_data[:,0], plot_data[:,7], label=r"$HSDetected Sim$")
        axs[0].plot(plot_data[:,0], plot_data_HPEKF[:,1],'k', label=r"$isOverriding sim$")
        # axs[0].plot(plot_data[:,0], plot_data[:,19], label=r"$isOverriding old$")
        axs[0].legend()
        

        axs[1].plot(timeSec_vec_hardware, phase_rate_vec_hardware, label=r"$phasedot_{hardware}$")
        axs[1].plot(plot_data[:,0], plot_data[:,2],'b', label=r"$phasedot_{sim}$")
        # axs[0].fill_between(plot_data[:,0], state_std_devs[:,2],  state_std_devs[:,3], color='blue', alpha=0.3)
        axs[1].plot(plot_data[:,0], state_std_devs[:,2],'b--')
        axs[1].plot(plot_data[:,0], state_std_devs[:,3],'b--')
        # axs[1].fill_between(plot_data[:,0], plot_data[:,6] - 1.96 * state_std_devs[:, 1],  plot_data[:,6] + 1.96*state_std_devs[:, 1], color='blue', alpha=0.3)
        axs[1].plot(plot_data_HPEKF[:,0], plot_data_HPEKF[:,3], label=r"$phase rate_{hpekf}$")
        axs[1].legend()
        axs[1].set_ylim([0,1.3])

        axs[2].plot(timeSec_vec_hardware, strideLength_vec_hardware, label=r"$Stride Length_{hardware}$")
        axs[2].plot(plot_data[:,0], plot_data[:,3],'b', label=r"$Stride Length_{sim}$")
        # axs[0].fill_between(plot_data[:,0], state_std_devs[:,4],  state_std_devs[:,5], color='blue', alpha=0.3)
        # axs[2].plot(plot_data[:,0], state_std_devs[:,4],'b--')
        # axs[2].plot(plot_data[:,0], state_std_devs[:,5],'b--')
        # axs[2].fill_between(plot_data[:,0], plot_data[:,7] - 1.96 * state_std_devs[:, 2],  plot_data[:,7] + 1.96*state_std_devs[:, 2], color='blue', alpha=0.3)
        axs[2].plot(plot_data_HPEKF[:,0], plot_data_HPEKF[:,4], label=r"$stride length_{hpekf}$")

        # axs[2].plot(plot_data[:,0], HSDetected_vec_hardware, label=r"$HSDetected, hardware$"))
        # axs[2].plot(plot_data[:,0], plot_data[:,7],'k', label=r"$isOverriding sim$")
        # axs[2].plot(plot_data[:,0], plot_data[:,19], label=r"$isOverriding old$")
        axs[2].legend()

        axs[3].plot(timeSec_vec_hardware, incline_vec_hardware, label=r"$Ramp_{hardware}$")
        axs[3].plot(plot_data[:,0], plot_data[:,4],'b', label=r"$Ramp_{sim}$")
        # axs[0].fill_between(plot_data[:,0], state_std_devs[:,6],  state_std_devs[:,7], color='blue', alpha=0.3)
        axs[3].plot(plot_data[:,0], state_std_devs[:,6],'b--')
        axs[3].plot(plot_data[:,0], state_std_devs[:,7],'b--')
        # axs[3].fill_between(plot_data[:,0], plot_data[:,8] - 1.96 * state_std_devs[:, 3],  plot_data[:,8] + 1.96*state_std_devs[:, 3], color='blue', alpha=0.3)

        axs[3].plot(plot_data_HPEKF[:,0], plot_data_HPEKF[:,5], label=r"$ramp_{hpekf}$")

        # axs[3].plot(plot_data[:,0], HSDetected_vec_hardware*10, label=r"$HSDetected$")
        # axs[3].plot(plot_data[:,0], plot_data[:,7]*10,'k', label=r"$isOverriding sim$")
        # axs[3].plot(plot_data[:,0], plot_data[:,19], label=r"$isOverriding old$")
        axs[3].legend()
        axs[3].set_ylim([-14,14])

        axs[4].plot(plot_data_HPEKF[:,0], plot_data_HPEKF[:,6], label=r"$SSE_{hpekf}$")
        axs[4].plot(plot_data[:,0], plot_data[:,5], label=r"$SSE_{sim}$")
        # axs[4].plot(plot_data[:,0], HSDetected_vec_hardware*1e3, label=r"$HSDetected$")
        # axs[4].plot(plot_data[:,0], plot_data[:,7]*1e3,'k', label=r"$isOverriding sim$")
        # axs[4].plot(plot_data[:,0], plot_data[:,19], label=r"$isOverriding old$")

        axs[4].legend()

        axs[-1].set_xlabel("time (sec)")
        print("this is done (show state)")
        # plt.show()

    if PLOT_AHRS:
    
        fig, axs = plt.subplots(3,1,sharex=True,figsize=(10,6))

        axs[0].plot(timeSec_vec_hardware, plot_data_imu[:,6], label=r"$roll AHRS$")
        axs[0].plot(timeSec_vec_hardware, HSDetected_vec_hardware*10, label=r"$HSDetected Hardware$")
        axs[0].plot(plot_data[:,0], plot_data[:,7]*10,'r', label=r"$HSDetected Sim$")
        axs[0].legend()

        axs[1].plot(timeSec_vec_hardware, plot_data_imu[:,7], label=r"$pitch AHRS$")
        axs[1].plot(timeSec_vec_hardware, HSDetected_vec_hardware*10, label=r"$HSDetected Hardware$")
        axs[1].plot(plot_data[:,0], plot_data[:,7]*10,'r', label=r"$HSDetected Sim$")
        # axs[1].plot(plot_data[:,0], plot_data[:,21], label=r"$HSD State$")
        axs[1].legend()

        axs[2].plot(timeSec_vec_hardware, plot_data_imu[:,8], label=r"$yaw AHRS$")
        axs[2].plot(timeSec_vec_hardware, HSDetected_vec_hardware*10 , label=r"$HSDetected Hardware$")
        axs[2].plot(plot_data[:,0], plot_data[:,7]*10,'r', label=r"$HSDetected Sim$")
        # axs[2].plot(plot_data[:,0], plot_data[:,21], label=r"$HSD State$")
        axs[2].legend()
        axs[-1].set_xlabel("time (sec)")
        print("this is done")

    if PLOT_EXO_IMU:
        fig, axs = plt.subplots(3,2,sharex=True,figsize=(10,6))

        axs[0,0].plot(timeSec_vec_hardware, accelVec_corrected_vec_hardware[:,0], label=r"$accelX IMU$")
        # axs[0,0].plot(timeSec_vec_hardware, HSDetected_vec_hardware*1, label=r"$HSDetected Hardware$")
        # axs[0,0].plot(plot_data[:,0], plot_data[:,7]*1, label=r"$HSDetected Sim$")

        axs[0,0].legend()

        axs[1,0].plot(plot_data[:,0], accelVec_corrected_vec_hardware[:,1], label=r"$accelY IMU$")
        # axs[1,0].plot(plot_data[:,0], HSDetected_vec_hardware*1, label=r"$HSDetected Hardware$")
        # axs[1,0].plot(plot_data[:,0], plot_data[:,7]*1, label=r"$HSDetected Sim$")
        axs[1,0].legend()

        axs[2,0].plot(plot_data[:,0], accelVec_corrected_vec_hardware[:,2], label=r"$accelZ IMU$")
        # axs[2,0].plot(plot_data[:,0], HSDetected_vec_hardware*1, label=r"$HSDetected Hardware$")
        # axs[2,0].plot(plot_data[:,0], plot_data[:,7]*1, label=r"$HSDetected Sim$")
        axs[2,0].legend()

        axs[2,0].legend()


        axs[-1,0].set_xlabel("time (sec)")
        print("this is done")

        # plt.show()


        axs[0,1].plot(timeSec_vec_hardware, gyroVec_corrected_vec_hardware[:,0], label=r"$gyroX IMU$")
        axs[0,1].plot(timeSec_vec_hardware, HSDetected_vec_hardware*1, label=r"$HSDetected Hardware$")
        axs[0,1].plot(plot_data[:,0], plot_data[:,7]*1, label=r"$HSDetected Sim$")
        
        axs[0,1].legend()

        axs[1,1].plot(timeSec_vec_hardware, gyroVec_corrected_vec_hardware[:,1], label=r"$gyroY IMU$")
        axs[1,1].plot(timeSec_vec_hardware, HSDetected_vec_hardware*1, label=r"$HSDetected Hardware$")
        axs[1,1].plot(plot_data[:,0], plot_data[:,7]*1, label=r"$HSDetected Sim$")
        axs[1,1].legend()

        axs[2,1].plot(timeSec_vec_hardware, gyroVec_corrected_vec_hardware[:,2], label=r"$gyroZ IMU$")
        axs[2,1].plot(timeSec_vec_hardware, HSDetected_vec_hardware*1, label=r"$HSDetected Hardware$")
        axs[2,1].plot(plot_data[:,0], plot_data[:,7]*1, label=r"$HSDetected Sim$")

        # axs[2].plot(plot_data[:,0], plot_data[:,21], label=r"$HSD State$")
        axs[2,1].legend()
        axs[-1,1].set_xlabel("time (sec)")
        print("this is done")

        fig, axs = plt.subplots(3,1,sharex=True,figsize=(10,6))

        axs[0].plot(timeSec_vec_hardware, roll_vec_hardware*180/(2*np.pi), label=r"$roll exo IMU$")
        axs[0].plot(timeSec_vec_hardware, HSDetected_vec_hardware*10, label=r"$HSDetected Hardware$")
        axs[0].plot(plot_data[:,0], plot_data[:,7]*10,'r', label=r"$HSDetected Sim$")
        
        axs[0].legend()

        axs[1].plot(timeSec_vec_hardware, pitch_vec_hardware*180/(2*np.pi), label=r"$pitch exo IMU$")
        axs[1].plot(timeSec_vec_hardware, HSDetected_vec_hardware*10, label=r"$HSDetected Hardware$")
        axs[1].plot(plot_data[:,0], plot_data[:,7]*10,'r', label=r"$HSDetected Sim$")
        axs[1].legend()

        axs[2].plot(timeSec_vec_hardware, yaw_vec_hardware*180/(2*np.pi), label=r"$yaw exo IMU$")
        axs[2].plot(timeSec_vec_hardware, HSDetected_vec_hardware*10 , label=r"$HSDetected Hardware$")
        axs[2].plot(plot_data[:,0], plot_data[:,7]*10,'r', label=r"$HSDetected Sim$")
        axs[2].legend()
        axs[-1].set_xlabel("time (sec)")
        print("this is done")

    # plt.show()

    if PLOT_MEASURED:

        fig, axs = plt.subplots(5,1,sharex=True,figsize=(10,6))

        # axs[0].plot(plot_data[:,0], plot_data[:,1], label=r"$phase_{hardware}$")
        axs[0].plot(timeSec_vec_hardware, z_measured_vec_hardware[:,0], label=r"$foot angle, measured_{act}$")
        axs[0].plot(plot_data_measured[:,0], plot_data_measured[:,1], label=r"$foot angle, model_{sim}$")
        axs[0].plot(plot_data_measured[:,0], plot_data_measured[:,7], label=r"$foot angle, meas_{sim}$")
        axs[0].fill_between(plot_data_measured[:,0], plot_data_measured[:,13],plot_data_measured[:,14], alpha=.5)
        axs[0].plot(plot_data[:,0], plot_data[:,7]*1e1, 'r', label=r"$HSDetected Sim$")
        # axs[0].plot(plot_data[:,0], HSDetected_vec_hardware*1e1, label=r"$HSDetected hardware$")
        axs[0].plot(plot_data[:,0], plot_data_HPEKF[:,1]*1e1,'k', label=r"$isOverriding sim$")
        # axs[0].fill_between(plot_data_measured[:,0], plot_data_measured[:,16],plot_data_measured[:,17], alpha=.5)
        
        axs[0].legend()
        axs[0].set_ylim([-70,50])

        # axs[0].plot(plot_data[:,0], plot_data[:,1], label=r"$phase_{hardware}$")
        axs[1].plot(timeSec_vec_hardware, z_measured_vec_hardware[:,1], label=r"$foot angle vel, measured_{act}$")
        axs[1].plot(plot_data_measured[:,0], plot_data_measured[:,2], label=r"$foot angle vel, model_{sim}$")
        axs[1].plot(plot_data_measured[:,0], plot_data_measured[:,8], label=r"$foot angle vel, meas_{sim}$")
        axs[1].fill_between(plot_data_measured[:,0], plot_data_measured[:,15],plot_data_measured[:,16], alpha=.5)
        axs[1].plot(plot_data[:,0], plot_data[:,7]*1e1, 'r', label=r"$HSDetected Sim$")
        # (?# axs[1].plot(plot_data[:,0], HSDetected_vec_hardware*1e1, label=r"$HSDetected Hardware$"))
        axs[1].plot(plot_data[:,0], plot_data_HPEKF[:,1]*1e1,'k', label=r"$isOverriding sim$")
        axs[1].legend()
        # axs[1].set_ylim([-500,500])

        # axs[1].plot(plot_data[:,0], plot_data[:,2], label=r"$phasedot_{hardware}$")
        axs[2].plot(timeSec_vec_hardware, z_measured_vec_hardware[:,2], label=r"$shank angle, measured_{act}$")
        axs[2].plot(plot_data_measured[:,0], plot_data_measured[:,3], label=r"$shank angle, model_{sim}$")
        axs[2].plot(plot_data_measured[:,0], plot_data_measured[:,9], label=r"$shank angle, meas_{sim}$")
        axs[2].fill_between(plot_data_measured[:,0], plot_data_measured[:,17],plot_data_measured[:,18], alpha=.5)
        axs[2].plot(plot_data[:,0], plot_data[:,7]*1e1, 'r', label=r"$HSDetected Sim$")
        # axs[2].plot(plot_data[:,0], HSDetected_vec_hardware*1e1, label=r"$HSDetected Hardware$")
        axs[2].plot(plot_data[:,0], plot_data_HPEKF[:,1]*1e1,'k', label=r"$isOverriding sim$")
        # axs[2].fill_between(plot_data_measured[:,0], plot_data_measured[:,18],plot_data_measured[:,19], alpha=.5)
        axs[2].legend()
        axs[2].set_ylim([-70,50])

        axs[3].plot(timeSec_vec_hardware, z_measured_vec_hardware[:,3], label=r"$shank angle vel, measured_{act}$")
        axs[3].plot(plot_data_measured[:,0], plot_data_measured[:,4], label=r"$shank angle vel, model_{sim}$")
        axs[3].plot(plot_data_measured[:,0], plot_data_measured[:,10], label=r"$shank angle vel, meas_{sim}$")
        axs[3].fill_between(plot_data_measured[:,0], plot_data_measured[:,19],plot_data_measured[:,20], alpha=.5)
        axs[3].plot(plot_data[:,0], plot_data[:,7]*1e1, 'r', label=r"$HSDetected Sim$")
        # axs[3].plot(plot_data[:,0], HSDetected_vec_hardware*1e1, label=r"$HSDetected Hardware$")
        axs[3].plot(plot_data[:,0], plot_data_HPEKF[:,1]*1e1,'k', label=r"$isOverriding sim$")
        axs[3].legend()
        # axs[3].set_ylim([-500,500])

        axs[4].plot(timeSec_vec_hardware, z_measured_vec_hardware[:,4], label=r"$foot position, measured_{act}$")
        axs[4].plot(plot_data_measured[:,0], plot_data_measured[:,5], label=r"$foot position, model_{sim}$")
        axs[4].plot(plot_data_measured[:,0], plot_data_measured[:,11], label=r"$foot position, meas_{sim}$")
        axs[4].fill_between(plot_data_measured[:,0], plot_data_measured[:,21],plot_data_measured[:,22], alpha=.5)
        axs[4].plot(plot_data[:,0], plot_data[:,7]*1e-1, 'r', label=r"$HSDetected Sim$")
        # axs[4].plot(plot_data[:,0], HSDetected_vec_hardware*1e-1, label=r"$HSDetected Hardware$")
        axs[4].plot(plot_data[:,0], plot_data_HPEKF[:,1]*1e-1,'k', label=r"$isOverriding sim$")
        axs[4].legend()
        # axs[4].set_ylim([-0.5,0.5])

        print("this is done (plot measured)")

    if PLOT_SIM_FOOT_POS:
        fig, axs = plt.subplots(2,1,sharex=True,figsize=(10,6))
        axs[0].plot(timeSec_vec_hardware, z_measured_vec_hardware[:,4], '-o', label=r"$foot position forward, measured_{act}$")
        axs[0].plot(plot_data_measured[:,0], plot_data_measured[:,5], label=r"$foot position forward, model_{sim}$")
        axs[0].fill_between(plot_data_measured[:,0], plot_data_measured[:,21],plot_data_measured[:,22], alpha=.5)
        if DO_FILT_ON_POS_FORWARD or SPOOF_SENSORS:
            axs[0].plot(plot_data_measured[:,0], plot_data_measured[:,11], '-o', label=r"$foot position forward, meas_{sim}$")
        # axs[0].plot(plot_data[:,0], HSDetected_vec_hardware*1e-1, label=r"$HSDetected Hardware$")
        axs[0].plot(plot_data[:,0], plot_data[:,7]*1e-1, 'r', label=r"$HSDetected Sim$")
        axs[0].plot(plot_data[:,0], plot_data_HPEKF[:,1]*1e-1,'k', label=r"$isOverriding sim$")

        axs[0].legend()

        axs[1].plot(timeSec_vec_hardware, z_measured_vec_hardware[:,5], label=r"$foot position up, measured_{act}$")
        axs[1].plot(plot_data_measured[:,0], plot_data_measured[:,6], label=r"$foot position up, model_{sim}$")
        axs[1].fill_between(plot_data_measured[:,0], plot_data_measured[:,23],plot_data_measured[:,24], alpha=.5)
        if DO_FILT_ON_POS_UP or SPOOF_SENSORS:
            axs[1].plot(plot_data_measured[:,0], plot_data_measured[:,12], '-o', label=r"$foot position up, meas_{sim}$")
        # axs[1].plot(plot_data[:,0], HSDetected_vec_hardware*1e-1, label=r"$HSDetected hardware$")
        axs[1].plot(plot_data[:,0], plot_data[:,7]*1e-1, 'r', label=r"$HSDetected Sim$")
        axs[1].plot(plot_data[:,0], plot_data_HPEKF[:,1]*1e-1,'k', label=r"$isOverriding sim$")
        axs[1].legend()

        print("this is done (plot foot pos)")

    if not True:
        fig, axs = plt.subplots(2,1,sharex=True,figsize=(10,6))
        axs[0].plot(plot_data_measured[:,0], plot_states[:,0],'-o', label=r"$state 1$")
        axs[0].plot(plot_data_measured[:,0], plot_states[:,1],'-o', label=r"$state 2$")
        axs[0].set_ylabel('Foot Forward')
        axs[0].legend()


        axs[1].plot(plot_data_measured[:,0], plot_states[:,2],'-o', label=r"$state 1$")
        axs[1].plot(plot_data_measured[:,0], plot_states[:,3],'-o', label=r"$state 2$")
        axs[1].set_ylabel('Foot Up')
        axs[1].legend()



    if not True:

        fig, axs = plt.subplots(sharex=True,figsize=(10,6))
        axs.plot(plot_data_measured[:,15], plot_data_measured[:,27])
        axs.set_xlabel('Foot pos forward')
        axs.set_ylabel('Foot pos Up')

        fig, axs = plt.subplots(sharex=True,figsize=(10,6))
        axs.plot(plot_data_measured[:,0], plot_data_measured[:,28],label='arctan angle')
        axs.set_xlabel('Time')


    if PLOT_TIMING_INFORMATION: # timing information

        # attitude

        fig, axs = plt.subplots(3,1,sharex=True,figsize=(10,6))

        axs[0].plot(plot_data_timing_attitude_ekf[:,0], plot_data_timing_attitude_ekf[:,1], label="timing_step")
        axs[0].plot(plot_data_timing_attitude_ekf[:,0], plot_data_timing_attitude_ekf[:,2], label="timing_measure")
        axs[0].plot(plot_data_timing_attitude_ekf[:,0], plot_data_timing_attitude_ekf[:,3], label="timing_get_euler_angles")
        axs[0].plot(plot_data_timing_attitude_ekf[:,0], plot_data_timing_attitude_ekf[:,4], label="timing_get_useful_angles")
        axs[0].legend()

        axs[1].plot(plot_data_timing_phase_ekf[:,0], plot_data_timing_phase_ekf[:,1], label="timing_step")
        axs[1].plot(plot_data_timing_phase_ekf[:,0], plot_data_timing_phase_ekf[:,2], label="timing_measure")
        axs[1].plot(plot_data_timing_phase_ekf[:,0], plot_data_timing_phase_ekf[:,3], label="timing_update")
        axs[1].plot(plot_data_timing_phase_ekf[:,0], plot_data_timing_phase_ekf[:,4], label="timing_gain_schedule_R")
        axs[1].legend()

        axs[2].plot(plot_data_timing_heelphase[:,0], plot_data_timing_heelphase[:,1], label="timing_step")
        axs[2].plot(plot_data_timing_heelphase[:,0], plot_data_timing_heelphase[:,2], label="timing_update_grays_OLS")
        axs[2].plot(plot_data_timing_heelphase[:,0], plot_data_timing_heelphase[:,3], label="timing_compute_z")
        axs[2].plot(plot_data_timing_heelphase[:,0], plot_data_timing_heelphase[:,4], label="timing_computeSSE")
        axs[2].plot(plot_data_timing_heelphase[:,0], plot_data_timing_heelphase[:,5], label="timing_override_phase_ekf_estimate")
        axs[2].plot(plot_data_timing_heelphase[:,0], plot_data_timing_heelphase[:,6], label="timing_detectHS")

        axs[2].plot(plot_data_timing_heelphase[:,0], plot_data_timing_heelphase[:,7], label = "time_loop0")
        axs[2].plot(plot_data_timing_heelphase[:,0], plot_data_timing_heelphase[:,8], label = "time_loop1")
        axs[2].plot(plot_data_timing_heelphase[:,0], plot_data_timing_heelphase[:,9], label = "time_loop2")
        axs[2].plot(plot_data_timing_heelphase[:,0], plot_data_timing_heelphase[:,10], label = "time_linalg")
        axs[2].plot(plot_data_timing_heelphase[:,0], plot_data_timing_heelphase[:,11], label = "time_enum")
        axs[2].plot(plot_data_timing_heelphase[:,0], plot_data_timing_heelphase[:,12], label = "time_sqrt")

        axs[-1].set_xlabel("time (sec)")

        axs[2].legend()
        # axs[0].set_title("heelphase")

        # plt.show()

    #PLOT FOOT IMU ACCEL
    if True:
        fig, axs = plt.subplots(4,1,sharex=True,figsize=(10,6))

        axs[0].plot(timeSec_vec_hardware, heelAccForward_meas_fromDeltaVelocity_vec_hardware, label=r"$heel acc from delta vel$")

        axs[0].plot(plot_data[:,0], plot_data[:,7]*1e1, 'r', label=r"$HSDetected Sim$")
        axs[0].plot(plot_data[:,0], plot_data_HPEKF[:,1]*1e1,'k', label=r"$isOverriding sim$")

        axs[1].plot(timeSec_vec_hardware, heelAccSide_meas_fromDeltaVelocity_vec_hardware, label=r"$heel side from delta vel$")

        axs[1].plot(plot_data[:,0], plot_data[:,7]*1e1, 'r', label=r"$HSDetected Sim$")
        axs[1].plot(plot_data[:,0], plot_data_HPEKF[:,1]*1e-1,'k', label=r"$isOverriding sim$")

        axs[2].plot(timeSec_vec_hardware, heelAccUp_meas_fromDeltaVelocity_vec_hardware, label=r"$heel up from delta vel$")

        axs[2].plot(plot_data[:,0], plot_data[:,7]*1e1, 'r', label=r"$HSDetected Sim$")
        axs[2].plot(plot_data[:,0], plot_data_HPEKF[:,1]*1e-1,'k', label=r"$isOverriding sim$")

        axs[3].plot(timeSec_vec_hardware, heelAccForward_meas_fromDeltaVelocity_norm_vec_hardware, label=r"$heel accel norm$")
        axs[3].plot(plot_data[:,0], plot_data[:,7]*1e1, 'r', label=r"$HSDetected Sim$")
        axs[3].plot(plot_data[:,0], plot_data_HPEKF[:,1]*1e-1,'k', label=r"$isOverriding sim$")


    if True:
        fig, axs = plt.subplots(sharex=True,figsize=(10,6))
        axs.plot(timeSec_vec_hardware[1:], dt_vec_hardware, label=r"$dt$")




    plt.show()
    


if __name__ == '__main__':
    main()