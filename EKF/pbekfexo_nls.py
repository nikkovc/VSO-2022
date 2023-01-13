import os, sys
from time import sleep, time, strftime, perf_counter
import traceback
import csv
import numpy as np
import scipy as sp
from scipy import linalg
import scipy.linalg
from StatProfiler import StatProfiler
from SoftRealtimeLoop import SoftRealtimeLoop
from ActPackMan import ActPackMan, FlexSEA
import gc
gc.disable()


from evalBezierFuncs_3P import *
import callibrate_angles_nls as ca

from filter_classes import FirstOrderLowPassLinearFilter, FirstOrderHighPassLinearFilter
from heel_strike_detector import HeelStrikeDetector
np.set_printoptions(precision=4)


thisdir = os.path.dirname(os.path.abspath(__file__))
print(thisdir)
sys.path.append(thisdir)
sys.path.append(thisdir + '/BestFitParams')
FADE_IN_TIME = 1.0

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

#correct new exoskeleton
Rot3 = np.array( [[np.cos(np.pi/4), 0 ,np.sin(np.pi/4)],[0,1, 0],[-np.sin(np.pi/4),0, np.cos(np.pi/4)]]  )
Rot4 = np.array( [[1, 0, 0],[0,np.cos(np.pi), -np.sin(np.pi)],[0,np.sin(np.pi), np.cos(np.pi)]] )
Rot5 = np.array( [[np.cos(np.pi), 0 ,np.sin(np.pi)],[0,1, 0],[-np.sin(np.pi),0, np.cos(np.pi)]]  )
Rot_correct = Rot5 @ Rot4 @ Rot3 @ Rot_correct




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


def runPB_EKF(exo_right, writer, fd_l, am, run_time = 60*10):

    attitude_ekf_args = {'sigma_gyro':0.0023,
                            'sigma_accel': 0.0032*5*1/5,
                            'sigma_q_AE':1e2,
                            'Q_pos_scale':1e-10}
    

    sigma_shank = 7

    sigma_shank_vel = 20


    # #FULL
    R_meas = np.diag([
        sigma_shank**2,
        sigma_shank_vel**2,
        ])


    sigma_q_phase=0
    sigma_q_phase_dot=6e-4

    attitude_ekf=AttitudeEKF(**attitude_ekf_args)

    # torque_profile = TorqueProfile('TorqueProfile/torqueProfileCoeffs_dataport3P.csv')
    gait_model = GaitModel_Fourier('GaitModel/VSPA_gait_model.csv',phase_order=20)


    phase_ekf_args = {'gait_model':gait_model,
            'torque_profile':None,
            'R_meas':R_meas,
            'sigma_q_phase':sigma_q_phase,
            'sigma_q_phase_dot':sigma_q_phase_dot,
            }

    phase_ekf = PhaseEKF(**phase_ekf_args)

    
    #set up the filters for the gyro and accel signals
    fc_gyro = 5
    gyroYLowPassFilter = FirstOrderLowPassLinearFilter(fc=fc_gyro,dt=1/100)

    fc_accel = 20
    accelZHighPassFilter = FirstOrderHighPassLinearFilter(fc=fc_accel,dt=1/100)

    #set up HSDetector
    HS_analysis_window = 10
    HSDetector = HeelStrikeDetector(HS_analysis_window)

    profilers={}
    profilers["measure_update"] = StatProfiler(name="measure_update")
    profilers["heel_phase"] = StatProfiler(name="heel_phase")




    startTime = time()
    prevTime = 0
    # inProcedure = True
    i = 0
    dt = 1/180
    DO_OVERRIDES = True
    UPDATE_OLS = True

    DRAW_CURRENT = False
    CORRECT_VICON = True
    
    accelZVec_buffer = []
    gyroYVec_buffer = []
    timeVec_buffer = []
    
    HSDetected = False


    updateFHfreq = 20
    isUpdateTime = True
    isUpdateR = False

    SHANK_ANGLE_OFFSET_VICON = -10


    exo_right.set_current_gains()
    loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.1)
    for t in loop:# inProcedure:
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
        exo_right.update()
        exoState=exo_right.act_pack
        # exoState = fxs.read_device(devId)
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

        #Correct for the IMU orientation to the standard from (z up, x forward)
        gyroVec_corrected = gyroVec_corrected - Rot_correct @ np.array([gyroX_IMU_bias,gyroY_IMU_bias,gyroZ_IMU_bias])
    
        ankleAngle_buffer = exoState.ank_ang

        ankleAngleVel_buffer = exoState.ank_vel
        motorCurrent_buffer = exoState.mot_cur
    
        accelZ = accelVec_corrected[2]
        gyroY = gyroVec_corrected[1]

        prof_accel_gyro_math.toc()

        
        unskewTime1 = time()
        unskewTime = unskewTime1 - unskewTime0
    
        
        #SOFT LIMIT
        if abs(motorCurrent_buffer) > 50000: #if the motor current magnitude exceeds 40 Amps
        
            print("HIT CURRENT SOFT LIMIT\n")
            raise Exception("HIT CURRENT SOFT LIMIT\n")
            beforeExiting()


        

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


        ankleAngleVel_meas = sideMultiplier * (ankleAngleVel_buffer * countToDeg) * 1000

        shankAngleVel_meas = sideMultiplier * -1 * gyroVec_corrected[1] * 180/np.pi


        HSdetect_prof.tic()
        HSDetected = HSDetector.detectHS(timeSec, footAngle_meas, footAngleVel_meas,  heelAccForward_meas_norm)
        HSdetect_prof.toc()


        getters_prof.tic()

        phase_estimate = phase_ekf.x_state_estimate[0]
        phase_dot_estimate = phase_ekf.x_state_estimate[1]

        z_measured = np.array([shankAngle_meas, shankAngleVel_meas])
        getters_prof.toc()

        profilers["measure_update"].profile( 
            lambda: phase_ekf.update(i, dt, z_measured)
            )
 
        temp_x_state_update = phase_ekf.x_state_update

        # control current
        
        # desCurrent = phase_ekf.get_torque()
        desCurrent = 1

        if desCurrent < 0.7:
            desCurrent = 0.7
        # print(desCurrent) 

        desTorque = desCurrent * Kt * N_avg * sideMultiplier
        actTorque = motorCurrent_buffer/1000 * Kt * N_avg

        if DRAW_CURRENT:
            # fxs.send_motor_command(devId, fxe.FX_CURRENT, desCurrent * 1000 * sideMultiplier)
            if t>FADE_IN_TIME:
                exo_right.i = desCurrent*sideMultiplier*loop.fade
            else:
                exo_right.i = (t/FADE_IN_TIME)*desCurrent*sideMultiplier*loop.fade

        log_prof.tic()

        temp_y_residual = phase_ekf.get_y_residual(num_meas)
        temp_x_state_estimate = phase_ekf.get_x_state_estimate()
        times = phase_ekf.get_times()
        

        # assert(isinstance(phase_ekf.SSE,float))

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
            int(HSDetected), #24
            temp_x_state_update[0], #25
            temp_x_state_update[1], #26
            phase_ekf.z_measured[0], #29
            phase_ekf.z_measured[1], #30
            phase_ekf.z_model[0], #35
            phase_ekf.z_model[1], #36
            temp_x_state_estimate[0], #41
            temp_x_state_estimate[1], #42
            ankleAngle, #46
            ankleAngleVel_meas, #47
            gyroY, #48
            actTorque, #49
            desTorque, #50
            attitude_ekf.get_times()[0], #51
            attitude_ekf.get_times()[1], #52
            attitude_ekf.get_times()[2], #53
            attitude_ekf.get_times()[3], #54
            times[1], #55
            times[2], #56
            times[3], #57
        ]
        # print(data_frame_vec)

        writer.writerow(data_frame_vec)
    
        i += 1
        log_prof.toc()

        if t > run_time:
            break
        # if timeSec >= run_time:
        #     inProcedure = False
        #     fxClose(devId)

        mainprof.toc()

        
    return True

if __name__ == '__main__':

    # port_cfg_path = '/home/pi/code/Actuator-Package/Python/flexsea_demo/ports.yaml'
    # ports, baud_rate = fxu.load_ports_from_file(port_cfg_path)

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


    filename_offsets = 'angleOffsets_right.csv'
    print(filename_offsets)

    attitude_ekf_calibrate=AttitudeEKF()
    imu_port = "/dev/ttyACM0"
    

    with ActPackMan("/dev/ttyACM1") as exo_right:
        print("in")
        footAngleOffset, shankAngleOffset, ankleAngleOffset = ca.main(exo_right, filename_offsets, attitude_ekf_calibrate)

        filename = '{0}Standalone_PB_EKF_Test.csv'.format(strftime("%Y%m%d-%H_AE%M%S"))
        print(filename)

        

        input('HIT ENTER TO START')
        with open(filename, "w", newline="\n") as fd_l:
            writer_l = csv.writer(fd_l)
            print(writer_l)
            with AhrsManager(csv_file_name='{0}_R_matrix_log.csv'.format(strftime("%Y%m%d-%H_AE%M%S"))) as am:
                runPB_EKF(exo_right,  writer_l, fd_l, am)
    
