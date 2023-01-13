import os, sys
from time import sleep, time, strftime, perf_counter
import traceback
import csv
import numpy as np
import scipy as sp
from scipy import linalg
import scipy.linalg
from StatProfiler import StatProfiler

from evalBezierFuncs_3P import *
from arctanMapFuncs import *
from attitudeEstimatorEKFFuncs import * 

from attitude_ekf import AttitudeEKF

from phase_ekf import PhaseEKF
from heelphase import HeelPhaseEstimator
from heelphase_ekf import HeelPhaseEKF
import calibrateAngles as ca
from gait_model import GaitModel_Bezier, GaitModel_Fourier
from ekf_torque_profile import TorqueProfile
from measurement_noise_model import MeasurementNoiseModel
# from pyEKFboost import GaitModel, TorqueProfile
from C_Wrapper import *


# from imu_raw_wrapper import IMU_Wrapper
from AhrsManager import AhrsManager

from filter_classes import FirstOrderLowPassLinearFilter, FirstOrderHighPassLinearFilter, GenericLinearFilter
from heel_strike_detector import HeelStrikeDetector
np.set_printoptions(precision=4)

thisdir = os.path.dirname(os.path.abspath(__file__))
print(thisdir)
sys.path.append(thisdir)

sys.path.append(thisdir + '/BestFitParams')

from flexsea import flexsea as flex
from flexsea import fxUtils as fxu  # pylint: disable=no-name-in-module
from flexsea import fxEnums as fxe  # pylint: disable=no-name-in-module

USE_C = True

def clear():
    if os.name == 'nt':
        os.system('cls')
    else:
        os.system('clear')



def printActPack(devId):
    exoState = fxReadDevice(devId)
    print('accelx: ', exoState.accelx, ', accely: ', exoState.accely, ' accelz: ', exoState.accelz)
    print('gyrox: ', exoState.gyrox, ', gyroy: ', exoState.gyroy, ' gyroz: ', exoState.gyroz)
    print('motor position: ', exoState.encoderAngle, ', motor velocity: ', exoState.encoderVelocity)
    print('battery current: ', exoState.batteryCurrent, ' , battery voltage: ', exoState.batteryVoltage, ' , battery temperature: ', exoState.batteryTemp)
    print('motor position: ', exoState.ankleAngle, ', motor velocity: ', exoState.ankleVelocity)


accelScaleFactor = 8192 #LSB/g
gyroScaleFactor = 32.8 #LSB/ deg/s
degToCount = 45.5111 
countToDeg = 1/degToCount


accelNormCutoff = 1.15


#account for small biases in the gyro measurment, in the IMU frame in rad/s
gyroX_IMU_bias = 0.021065806164927196
gyroY_IMU_bias = 0.01037782833021424
gyroZ_IMU_bias = 0.007913779656035359


##define ankle angle zero when the ankle is perpendicular to shank
#this value is only good for the right leg
# 

eye3 = np.eye(3)
eye6 = np.eye(6)
eye4= np.eye(4)


# correct exo IMU

#also only good for the right leg
theta_correction = 39.1090 * np.pi/180


#correct to non tilted axes
Rot_unskew = np.array(  [[np.cos(theta_correction), -np.sin(theta_correction),0],[np.sin(theta_correction), np.cos(theta_correction),0],[0, 0, 1]])

# correct to z up, x forward, y left

Rot1 = np.array( [[1, 0, 0],[0,np.cos(-np.pi/2), -np.sin(-np.pi/2)],[0,np.sin(-np.pi/2), np.cos(-np.pi/2)]] )
Rot2 = np.array( [[np.cos(-np.pi/2), 0 ,np.sin(-np.pi/2)],[0,1, 0],[-np.sin(-np.pi/2),0, np.cos(-np.pi/2)]]  )

Rot_correct = Rot2 @ Rot1 @ Rot_unskew




# correct foot IMU

Rot_correct_imu =  np.array( [[0, 0, 1],[0,-1,0],[1,0,0]] )

#Phase estimator
Kt = 0.14 * 0.537/np.sqrt(2)
N_avg = 15



side = 'right'

sideMultiplier = 1
if (side == "left" or side == "l"):
    sideMultiplier = -1
elif (side == "right" or side == "r"):
    sideMultiplier = 1


def runPB_EKF(fxs, devId, writer, fd_l, am, run_time = 60*10):

    attitude_ekf_args = {'sigma_gyro':0.0023,
                            'sigma_accel': 0.0032*5*1/5,
                            'sigma_q_AE':1e2,
                            'Q_pos_scale':1e-10}
    


    gait_model_covar_path = f'GaitModel/covar_fourier_normalizedsL_linearsL.csv'

    sigma_foot = 1
    sigma_shank = 5

    sigma_foot_vel = 10
    sigma_shank_vel = 10

    sigma_heel_pos_forward = 0.01 #m
    sigma_heel_pos_up = 0.02 #m

    meas_config = 'full'

    if meas_config == 'angles':
        num_meas = 4
    elif meas_config == 'full':
        num_meas = 6
    else:  
        num_meas = 5

    # #FULL
    R_meas = np.diag([sigma_foot**2,
        sigma_foot_vel**2,\
        sigma_shank**2,
        sigma_shank_vel**2,\
        sigma_heel_pos_forward**2, 
        sigma_heel_pos_up**2,
        ])


    sigma_q_phase=0
    sigma_q_phase_dot=3e-4
    sigma_q_sL=1e-3
    sigma_q_incline=9e-3


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
                'sigma_q_incline':sigma_q_incline
                }

        phase_ekf = PhaseEKF(**phase_ekf_args)

    # phase_ekf = PhaseEKF(shankAngleOffset, ankleAngleOffset)
    # heel_phase_estimator=HeelPhaseEstimator(phase_ekf, save_plotting_data=False)
    
    #INITIALIZE BACKUP EKF
    Q_HP = np.diag([1e-4,5e-4])
    R_HP = phase_ekf.R_mean
    heelphase_ekf=HeelPhaseEKF(phase_ekf, Q_HP, R_HP, timing_based_estimator=None)


    #set up the filters for the gyro and accel signals
    fc_gyro = 5
    gyroYLowPassFilter = FirstOrderLowPassLinearFilter(fc=fc_gyro,dt=1/100)

    fc_accel = 20
    accelZHighPassFilter = FirstOrderHighPassLinearFilter(fc=fc_accel,dt=1/100)

    #set up HSDetector
    HS_analysis_window = 20*2
    HSDetector = HeelStrikeDetector(HS_analysis_window)

    profilers={}
    profilers["measure_update"] = StatProfiler(name="measure_update")
    profilers["heel_phase"] = StatProfiler(name="heel_phase")


    #SET UP SUBJECT LEG LENGTH
    # SUBJECT_LEG_LENGTH = 1.854/2

    SUBJECT_LEG_LENGTH = 1.778/2

    #SET UP FILTERS

    #SET UP Filters
    ω = 0.5 * np.pi*2
    ζ= 0.9

    A = np.array([
        [0,             1        ]      ,
        [-ω**2,         -2*ω*ζ   ]])

    C = np.array([[-ω**2,         -2*ω*ζ   ]])
    B = np.array([[0, 1]]).T
    D = np.array([[1.0]])
    HPF_X0 = 0.0*np.ones((A.shape[0],1))

    heelPosForwardFilter = GenericLinearFilter(A, B, C, D, HPF_X0)


    ω = 0.5 * np.pi*2
    ζ= 0.9

    A = np.array([
        [0,             1        ]      ,
        [-ω**2,         -2*ω*ζ   ]])

    C = np.array([[-ω**2,         -2*ω*ζ   ]])
    B = np.array([[0, 1]]).T
    D = np.array([[1.0]])
    HPF_X0 = 0.0*np.ones((A.shape[0],1))
    heelPosUpFilter = GenericLinearFilter(A, B, C, D, HPF_X0)


    startTime = time()
    prevTime = 0
    inProcedure = True
    i = 0
    dt = 1/180
    DO_OVERRIDES= True
    UPDATE_OLS= True

    DRAW_CURRENT = not False
    CORRECT_VICON = True
    
    accelZVec_buffer = []
    gyroYVec_buffer = []
    timeVec_buffer = []
    
    HSDetected = False


    updateFHfreq = 20
    isUpdateTime = True
    isUpdateR = False

    SHANK_ANGLE_OFFSET_VICON = -10

    heelVelForward_numInt = 0
    heelPosForward_numInt = 0

    heelVelUp_numInt = 0
    heelPosUp_numInt = 0
    heelPos_filt = 0.0



    fxs.set_gains(devId, 40, 400, 0, 0, 0, 128)


    try:
        while inProcedure:
            mainprof.tic()
            firsthalf.tic()

            unskewTime0 = time()
            currentTime = time()
            timeSec = currentTime - startTime

            isUpdateTime = (timeSec % 1/updateFHfreq  < 1e-2)
            # print(isUpdateTime)
            dt = timeSec - prevTime
            # print(dt)

            read_exo_prof.tic()
            exoState = fxs.read_device(devId)
            read_exo_prof.toc()
        
            # clearTerminal()
            # print(i) 

            prof_accel_gyro_math.tic()
        
            accelX = exoState.accelx/accelScaleFactor # in units of g
            accelY = exoState.accely/accelScaleFactor
            accelZ = exoState.accelz/accelScaleFactor

            gyroX = exoState.gyrox/gyroScaleFactor * np.pi/180 #in units of rad/s
            gyroY = exoState.gyroy/gyroScaleFactor * np.pi/180
            gyroZ = exoState.gyroz/gyroScaleFactor * np.pi/180

            accelVec = np.array([accelX,accelY,accelZ])
            accelVec_corrected = Rot_correct @ (accelVec)

            accelNorm = np.linalg.norm(accelVec_corrected)

        
            gyroVec = np.array([gyroX,gyroY,gyroZ])
            gyroVec_corrected = Rot_correct @ (gyroVec)
            gyroVec_corrected = gyroVec_corrected - Rot_correct @ np.array([gyroX_IMU_bias,gyroY_IMU_bias,gyroZ_IMU_bias])
        
            ankleAngle_buffer = exoState.ank_ang

            ankleAngleVel_buffer = exoState.ank_vel
            motorCurrent_buffer = exoState.mot_cur
        
            accelZ = accelVec_corrected[2]
            gyroY = gyroVec_corrected[1]

            prof_accel_gyro_math.toc()

            
            # read IMU data
            ahrs_prof.tic()
            am.update()
            ahrs_prof.toc()

            filters_prof.tic()

            unskewTime1 = time()
            unskewTime = unskewTime1 - unskewTime0
        
            #filter the signals for the HSD
            gyroY_filter = gyroYLowPassFilter.step(i, gyroY)
            accelZ_filter = accelZHighPassFilter.step(i, accelZ)

            #READ IN AHRS ACCEL AND FILTER TO GET POS
            acc_vec_ahrs = am.get_linear_acc(CORRECT_VICON=CORRECT_VICON)
            heelAccForward_meas = acc_vec_ahrs[0,0]
            heelAccSide_meas = acc_vec_ahrs[1,0]
            heelAccUp_meas = acc_vec_ahrs[2,0] - 9.81

            acc_vec_ahrs_fromDeltaVelocity = am.get_linear_acc_fromDeltaVelocity()
            heelAccForward_meas_fromDeltaVelocity = acc_vec_ahrs_fromDeltaVelocity[0,0]
            heelAccSide_meas_fromDeltaVelocity = acc_vec_ahrs_fromDeltaVelocity[1,0]
            heelAccUp_meas_fromDeltaVelocity = acc_vec_ahrs_fromDeltaVelocity[2,0] - 9.81

            #NUMERICALLY INTEGRATE FOOT POS Forward and Up
            heelVelForward_numInt += dt * heelAccForward_meas_fromDeltaVelocity
            heelPosForward_numInt += dt * heelVelForward_numInt + dt**2*0.5*heelAccForward_meas_fromDeltaVelocity
            states_heelPosForward, heelPosForward_meas_filt = heelPosForwardFilter.step(i, dt, heelPosForward_numInt)
            heelPosForward_meas_filt = heelPosForward_meas_filt[0,0]
            heelPosForward_meas_filt = np.max([np.min([heelPosForward_meas_filt, 0.4]), -0.25])


            heelVelUp_numInt += dt * heelAccUp_meas_fromDeltaVelocity
            heelPosUp_numInt += dt * heelVelUp_numInt + dt**2*0.5*heelAccUp_meas_fromDeltaVelocity
            states_heelPosUp, heelPosUp_meas_filt = heelPosUpFilter.step(i, dt, heelPosUp_numInt)
            heelPosUp_meas_filt = heelPosUp_meas_filt[0,0]
            heelPosUp_meas_filt = np.max([np.min([heelPosUp_meas_filt, 0.2]), -0.16])



            
            # print(f'heelPosForward_meas_filt: {heelPosForward_meas_filt}')
            filters_prof.toc()
            firsthalf.toc()

            #SOFT LIMIT
            if abs(motorCurrent_buffer) > 50000: #if the motor current magnitude exceeds 40 Amps
            
                print("HIT CURRENT SOFT LIMIT\n")
                raise Exception("HIT CURRENT SOFT LIMIT\n")
                beforeExiting()


            HSdetect_prof.tic()
            HSDetected = HSDetector.detectHS(timeSec, gyroY_filter, accelZ_filter)
            HSdetect_prof.toc()

            attStep_prof.tic()
            attitude_ekf.step(i, dt, isUpdateTime)
            attStep_prof.toc()

            phaseStep_prof.tic()
            phase_ekf.step(i,dt)
            phaseStep_prof.toc()
    

            attUpdate_prof.tic()
            attitude_ekf.measure(i, gyroVec_corrected, accelVec_corrected, isUpdateTime, CORRECT_VICON)
            psi, theta, phi = attitude_ekf.get_euler_angles()
            attUpdate_prof.toc()

            prevTime = timeSec

            math_prof.tic()

            # print('Psi (about +x): ' + str(  round(psi ,4) *180/np.pi))
            # print('Theta (about +y): ' + str(  round(theta ,4) *180/np.pi))
            # print('Phi (about +z): ' + str(  round(phi ,4)*180/np.pi ))

            ankleAngle = sideMultiplier * ((ankleAngle_buffer * countToDeg ) - ankleAngleOffset)
            # print('ankleAngle')
            # print(ankleAngle)
            # print(ankleAngle_buffer/10)


            shankAngle_meas = attitude_ekf.get_useful_angles(ankleAngle, sideMultiplier)

            shankAngle_meas = shankAngle_meas - shankAngleOffset + SHANK_ANGLE_OFFSET_VICON

            ## dump new IMU code here
            R_foot = am.get_R_foot(CORRECT_VICON=CORRECT_VICON)


            roll, pitch, yaw = attitude_ekf.get_euler_angles_new(R_foot)

            footAngle_meas = -pitch*180/np.pi

            footAngle_meas = footAngle_meas - footAngleOffset

            if footAngle_meas < -180:
                footAngle_meas += 360

            if footAngle_meas > 180:
                footAngle_meas -= 360

            # if i%10 == 0:
            #   print('shankAngle: ' + str(shankAngle))
            #   print('footAngle: ' + str(footAngle))

            ankleAngleVel_meas = sideMultiplier * (ankleAngleVel_buffer * countToDeg) * 1000

            shankAngleVel_meas = sideMultiplier * -1 * gyroVec_corrected[1] * 180/np.pi

            ang_vel_vec_ahrs = -1 * am.get_rotational_vel(CORRECT_VICON=CORRECT_VICON)
            footAngleVel_meas = ang_vel_vec_ahrs[1,0]

            # firsthalf.toc() 75

            # Scale kinematics

            # footAngle = (footAngle - footBias) * footScale
            # shankAngle = (shankAngle - shankBias) * shankScale

            # shankAngleVel_meas *= footScale
            # footAngleVel_meas *= shankScale


            # update the phase estimator
            # Extract relevant variables from the PE portion of the EKF
            # print('x_state_estimate')
            # print(x_state_estimate)

            # print('P_covar_estimate')
            # print(P_covar_estimate)

            math_prof.toc()
            getters_prof.tic()

            phase_estimate = phase_ekf.x_state_estimate[0]
            phase_dot_estimate = phase_ekf.x_state_estimate[1]
            pseudoStepLength_estimate = phase_ekf.x_state_estimate[2]
            incline_estimate = phase_ekf.x_state_estimate[3]

            z_measured = np.array([footAngle_meas, footAngleVel_meas, shankAngle_meas, shankAngleVel_meas, heelPosForward_meas_filt,heelPosUp_meas_filt])
            getters_prof.toc()
    
            profilers["measure_update"].profile( 
                lambda: phase_ekf.update(i, z_measured)
                )
            # print(HSDetected)
    

            # profilers["heel_phase"].profile(
            #     lambda: heel_phase_estimator.step(i, timeSec, HSDetected, DO_OVERRIDES=DO_OVERRIDES, UPDATE_OLS=UPDATE_OLS)
            #     )

            profilers["heel_phase"].profile(
                lambda: heelphase_ekf.step(i, timeSec, dt, z_measured, HSDetected, DO_OVERRIDES)
                )

            # print('P_covar_update')
            # print(P_covar_update)
            temp_x_state_update = phase_ekf.x_state_update

            strideLength_update_descaled = arctanMap(temp_x_state_update[2])

            #DENORMALIZE BY SUBJECT LEG LENGTH 
            strideLength_update_descaled = SUBJECT_LEG_LENGTH * strideLength_update_descaled

            # control current
            
            desCurrent = phase_ekf.get_torque()

            if desCurrent < 0.2:
                desCurrent = 0.2
            if desCurrent > 20:
                desCurrent = 20

            # print(desCurrent) 

            desTorque = desCurrent * Kt * N_avg * sideMultiplier
            actTorque = motorCurrent_buffer/1000 * Kt * N_avg

            if DRAW_CURRENT:
                fxs.send_motor_command(devId, fxe.FX_CURRENT, desCurrent * 1000 * sideMultiplier)

            log_prof.tic()



            times = phase_ekf.get_times()
            

            data_frame_vec = [
                round(timeSec,4), #0
                accelVec_corrected[0], #1
                accelVec_corrected[1], #2
                accelVec_corrected[2], #3
                gyroVec_corrected[0], #4
                gyroVec_corrected[1], #5
                gyroVec_corrected[2], #6
                psi, #7 
                theta, #8
                phi, # 9
                attitude_ekf.get_z_measured()[0], #10
                attitude_ekf.get_z_measured()[1], #11
                attitude_ekf.get_z_measured()[2], #12
                attitude_ekf.get_z_measured()[3], #13
                attitude_ekf.get_z_measured()[4], #14
                attitude_ekf.get_z_measured()[5], #15
                attitude_ekf.get_z_model()[0], #16
                attitude_ekf.get_z_model()[1], #17
                attitude_ekf.get_z_model()[2], #18
                attitude_ekf.get_z_model()[3], #19
                attitude_ekf.get_z_model()[4], #20
                attitude_ekf.get_z_model()[5], #21
                shankAngle_meas, #22
                footAngle_meas, #23
                int(HSDetected), #24
                temp_x_state_update[0], #25
                temp_x_state_update[1], #26
                temp_x_state_update[2], #27
                temp_x_state_update[3], #28
                phase_ekf.z_measured[0], #29
                phase_ekf.z_measured[1], #30
                phase_ekf.z_measured[2], #31
                phase_ekf.z_measured[3], #32
                phase_ekf.z_measured[4], #33
                phase_ekf.z_measured[5], #34
                phase_ekf.z_model[0], #35
                phase_ekf.z_model[1], #36
                phase_ekf.z_model[2], #37
                phase_ekf.z_model[3], #38
                phase_ekf.z_model[4], #39
                phase_ekf.z_model[5], #40
                phase_ekf.x_state_estimate[0], #41
                phase_ekf.x_state_estimate[1], #42
                phase_ekf.x_state_estimate[2], #43
                phase_ekf.x_state_estimate[3], #44
                strideLength_update_descaled, #45
                ankleAngle, #46
                ankleAngleVel_meas, #47
                gyroY, #48
                actTorque, #49
                desTorque, #50
                attitude_ekf.timing_step, #51
                attitude_ekf.timing_measure, #52
                attitude_ekf.timing_get_euler_angles, #53
                attitude_ekf.timing_get_useful_angles, #54
                phase_ekf.timing_measure, #55
                phase_ekf.timing_update, #56
                phase_ekf.timing_gain_schedule_R, #57
                roll*180/3.1415, #58
                pitch*180/3.1415, #59
                yaw*180/3.1415, #60
                heelAccForward_meas, #61
                heelPosForward_meas_filt, #62
                heelPosUp_meas_filt, #63
                states_heelPosForward[0,0],#64
                states_heelPosForward[1,0],#65
                states_heelPosUp[0,0],#66
                states_heelPosUp[1,0],#67
                heelAccSide_meas, #68
                heelAccUp_meas, #69
                heelAccForward_meas_fromDeltaVelocity, #70
                heelAccSide_meas_fromDeltaVelocity, #71
                heelAccUp_meas_fromDeltaVelocity, #72
                int(heelphase_ekf.isOverriding), #73
                heelphase_ekf.phase_HP, #74
                heelphase_ekf.phase_rate_HP, #75
                SUBJECT_LEG_LENGTH * arctanMap(heelphase_ekf.x_state[0,0]), #76
                heelphase_ekf.x_state[1,0], #77
                heelphase_ekf.SSE #78
                ]
            # print(data_frame_vec)
    
            writer.writerow(data_frame_vec)
        
            i += 1
            log_prof.toc()

            if timeSec >= run_time:
                inProcedure = False
                fxClose(devId)

            mainprof.toc()

    except Exception as e:
        print("broke: " + str(e))
        print(traceback.format_exc())
        fxs.send_motor_command(devId, fxe.FX_NONE, 0)
        sleep(0.5)
        fxs.close(devId)
        pass

        
    return True

if __name__ == '__main__':

    port_cfg_path = '/home/pi/code/Actuator-Package/Python/flexsea_demo/ports.yaml'
    ports, baud_rate = fxu.load_ports_from_file(port_cfg_path)

    read_exo_prof = StatProfiler(name="reading_exo")
    prof_accel_gyro_math = StatProfiler(name="accel and gyro math")
    ahrs_prof = StatProfiler(name="Ahrs.update")
    filters_prof = StatProfiler(name="filters")
    HSdetect_prof = StatProfiler(name="detecting HS")
    attStep_prof = StatProfiler(name="attitude step")
    phaseStep_prof = StatProfiler(name="phase step")
    math_prof = StatProfiler(name="Math and getting important euler angles")
    getters_prof = StatProfiler(name="Getters")
    attUpdate_prof = StatProfiler(name="attituse update and get euler")
    log_prof = StatProfiler(name="Logging")

    
    
    
    mainprof = StatProfiler(name="Main Loop")
    firsthalf = StatProfiler(name="first_quarter_loop")

    print('Loaded ports: ' + str(ports))
    print('Using baud rate: ' + str(int(baud_rate)))
    port_left = ports[1]
    port_right = ports[1]

    print(port_right)
    
    fxs = flex.FlexSEA()
    streamFreq = 300 # Hz
    data_log = False  # False means no logs will be saved
    shouldAutostream = 1; # This makes the loop run slightly faster
    debug_logging_level = 6  # 6 is least verbose, 0 is most verbose
    
    devId_right = fxs.open(port_right, baud_rate, debug_logging_level)
    print(devId_right)
    fxs.start_streaming(devId_right, freq=streamFreq, log_en=data_log)
    app_type = fxs.get_app_type(devId_right)


    filename_offsets = 'angleOffsets_right.csv'
    print(filename_offsets)

    attitude_ekf_calibrate=AttitudeEKF_C()#Attitude_EKF()
    # attitude_ekf_imu_calibrate=AttitudeEKF()
    imu_port = "/dev/ttyACM0"
    

    footAngleOffset, shankAngleOffset, ankleAngleOffset = ca.main(fxs, devId_right,filename_offsets, attitude_ekf_calibrate)

    filename = '{0}Standalone_PB_EKF_Test.csv'.format(strftime("%Y%m%d-%H_AE%M%S"))
    print(filename)

    

    input('HIT ENTER TO START')

    with open(filename, "w", newline="\n") as fd_l:
        writer_l = csv.writer(fd_l)
        print(writer_l)
        with AhrsManager(csv_file_name='{0}_R_matrix_log.csv'.format(strftime("%Y%m%d-%H_AE%M%S"))) as am:
            runPB_EKF(fxs, devId_right,  writer_l, fd_l, am)
    
