""" Simulates the phase estimator ekf using loaded data. """
import numpy as np
from time import strftime
import time
import pyEKFboost
np.set_printoptions(precision=4)
# import matplotlib.pyplot as plt



from attitude_ekf import AttitudeEKF
from phase_ekf import PhaseEKF
from arctanMapFuncs import *
from evalBezierFuncs_3P import *
from heelphase import HeelPhaseEstimator
from gait_model import GaitModel
# from ekf_torque_profile import TorqueProfile


DO_KIDNAPPING=False
DO_OVERRIDES=False
UPDATE_OLS=True # 7.790 s or 5.425 s with trivilized-but-C++-linked gait model or 5.505 s with actual Beziers! Now 5.209
# UPDATE_OLS=False # 6.423 s or 5.068 s with trivilized-but-C++-linked gait model or 5.082 s with actual Beziers! Now 4.936
sideMultiplier = -1

class FakeProfile():
    def __init__(self):
        self.profileMesh=None

def profile(fmt):
    def decorator(fnc):
        def inner(*args, **kwargs):
            t0 = time.time()
            res=fnc(*args, **kwargs)
            print(fmt%(time.time()-t0))
            return res
        return inner
    return decorator

@profile("main: %.3f seconds")
def main(): 
    # Initial performance: Gray's VM: 18.404 s , Gray's Pi4: 57.697 s 
    # First 1000 points: Gray's VM: 2.56 s, Gray's Pi4: 7.820 s
    data = np.loadtxt("20200712-19_AE5647_PB_EKF_Test0deg.csv", delimiter=',')[:1000,:]


    

    attitude_ekf=AttitudeEKF()
    # torque_profile = TorqueProfile()
    # gait_model = GaitModel()
    gait_model = pyEKFboost.GaitModel() # 5.425 s
    phase_ekf = PhaseEKF(0, 0, gait_model, FakeProfile())
    heel_phase_estimator=HeelPhaseEstimator(phase_ekf, save_plotting_data=True)


    plot_data = []
    state_data = []

    plot_data1 = []

    plot_data_angles = []
    plot_data_measured = []

    timeData = []
    prev=0
    footAngleOffset = 3.816*0
    for i,x in enumerate(data[:-1]):

        timeSec=x[0]
        dt = timeSec-prev

        prev=timeSec
        accelVec_corrected=x[1:4]
        gyroVec_corrected=x[4:7]
        psi1, theta1, phi1=x[7:10]
        x_state_AE = x[10:16]

        ankleAngle = x[64]
        # print(ankleAngle)
        shankAngle1 = x[40] + footAngleOffset
        footAngle1 = x[41] - footAngleOffset

        x_state_PE = x[43:47]
        z_measured_act = x[47:51]
        z_measured_act[0] = z_measured_act[0] - footAngleOffset
        z_measured_act[2] = z_measured_act[2] + footAngleOffset

        HSDetected = x[42]

        updateFHfreq = 20
        isUpdateTime = (timeSec % 1/updateFHfreq  < 1e-2)
        attitude_ekf.step(i+1, dt, isUpdateTime=isUpdateTime)
        phase_ekf.step(i+1,dt)

        if i % 1000 == 0 and i != 0 and DO_KIDNAPPING:
            # print(timeSec)
            phase_ekf.x_state_estimate[0] = np.random.uniform(0,1)
            phase_ekf.x_state_estimate[1] = np.random.uniform(-1,1)
            phase_ekf.x_state_estimate[2] = np.random.uniform(-5,5)
            phase_ekf.x_state_estimate[3] = np.random.uniform(-10,10)

        attitude_ekf.measure(i+1, gyroVec_corrected, accelVec_corrected, isUpdateTime=isUpdateTime)
        psi2, theta2, phi2 = attitude_ekf.get_euler_angles()

        shankAngle2, footAngle2 = attitude_ekf.get_useful_angles(ankleAngle)

        footAngle2 = footAngle2 - footAngleOffset
        shankAngle2 = shankAngle2 + footAngleOffset


        ankleAngleVel_meas = x[65]
        shankAngleVel_meas = sideMultiplier * gyroVec_corrected[1] * 180/np.pi
        footAngleVel_meas = ankleAngleVel_meas + shankAngleVel_meas

        phase_estimate_PE = phase_ekf.x_state_estimate[0,0]


        # phase_ekf.gain_schedule_Q(phase_estimate_PE)

        phase_ekf.gain_schedule_R(phase_estimate_PE)
        # print(phase_ekf.R)

        z_measured_sim = [footAngle2, footAngleVel_meas, shankAngle2, shankAngleVel_meas]
        phase_ekf.measure(i+1, footAngle2, footAngleVel_meas, shankAngle2, shankAngleVel_meas)

        # print(HSDetected)
        heel_phase_estimator.step(i+1, timeSec, HSDetected, DO_OVERRIDES=DO_OVERRIDES,UPDATE_OLS=UPDATE_OLS) #VM:2.06 vs 2.533 ## PI 6.365 vs 7.761 segfault
        phase_ekf.update(i+1)


        stepLength_update_descaled_PE1 = x[63]
        stepLength_update_descaled_PE2 = arctanMap(phase_ekf.x_state_update[2,0])

        # print(x_state_PE)
        # print(np.array([x_state_PE[:], phase_ekf.x_state_estimate[:,0]]))

        # """ # 
        state_data.append(np.array([x_state_PE[:], phase_ekf.x_state_estimate[:,0]]))
        # plot_data.append([timeSec, psi1, theta1, phi1, psi2, theta2, phi2, shankAngle1, footAngle1, shankAngle2, footAngle2])

        plot_data_angles.append([timeSec,accelVec_corrected[0],accelVec_corrected[1],accelVec_corrected[2],gyroVec_corrected[0],gyroVec_corrected[1],gyroVec_corrected[2]])

        plot_data.append([timeSec, x_state_PE[0],x_state_PE[1],stepLength_update_descaled_PE1,x_state_PE[3],  \
            phase_ekf.x_state_update[0,0],phase_ekf.x_state_update[1,0],stepLength_update_descaled_PE2,phase_ekf.x_state_update[3,0],\
            heel_phase_estimator.SSE, phase_ekf.SSE, HSDetected, heel_phase_estimator.isOverriding,\
            heel_phase_estimator.phase_estimate_HP,heel_phase_estimator.phase_dot_estimate_HP,heel_phase_estimator.stepLength_estimate_HP,heel_phase_estimator.incline_estimate_HP])

        plot_data1.append([timeSec, footAngle1])

        # timeData.append(dt)

        # print([timeSec, z_measured_act[0],z_measured_act[1],z_measured_act[2],z_measured_act[3],\
        #     phase_ekf.z_model[0,0],phase_ekf.z_model[1,0],phase_ekf.z_model[2,0],phase_ekf.z_model[3,0],\
        #     heel_phase_estimator.z_model_HP[0,0],heel_phase_estimator.z_model_HP[1,0],heel_phase_estimator.z_model_HP[2,0],heel_phase_estimator.z_model_HP[3,0]])
        plot_data_measured.append([timeSec,
            z_measured_act[0],z_measured_act[1],z_measured_act[2],z_measured_act[3],\
            phase_ekf.z_model[0,0],phase_ekf.z_model[1,0],phase_ekf.z_model[2,0],phase_ekf.z_model[3,0],\
            z_measured_sim[0], z_measured_sim[1], z_measured_sim[2], z_measured_sim[3],\
            phase_ekf.z_model[0,0] + 2*np.sqrt(phase_ekf.R[0,0]), phase_ekf.z_model[2,0] + 2*np.sqrt(phase_ekf.R[2,2]),\
            phase_ekf.z_model[0,0] - 2*np.sqrt(phase_ekf.R[0,0]), phase_ekf.z_model[2,0] - 2*np.sqrt(phase_ekf.R[2,2])])
        """   """ # - 2.537 vs 2.679 VM

    plot_data = np.array(plot_data)
    plot_data_angles = np.array(plot_data_angles)
    plot_data_measured = np.array(plot_data_measured)
    plot_data1 = np.array(plot_data1)
    state_data = np.array(state_data)


    timeData = data[:,0]
    dt = timeData[1:] - timeData[:-1]
    freqData = 1/dt

    new_HS_data=[]
    rms_hist = []
    for i in range(len(heel_phase_estimator.list_of_lists_of_time)):

        ts = heel_phase_estimator.list_of_lists_of_time[i]
        zmods = heel_phase_estimator.list_of_lists_of_z_model[i]
        xhps = heel_phase_estimator.list_of_lists_of_x_hp[i]
        rms_hist.append(xhps[0][5])
        for q in range(len(ts)):
            new_HS_data.append([
                ts[q], zmods[q][0,0], zmods[q][1,0], zmods[q][2,0], zmods[q][3,0],
                xhps[q][0], xhps[q][1], xhps[q][2], xhps[q][3], xhps[q][4], xhps[q][5], xhps[q][6]])
    new_HS_data=np.array(new_HS_data)



if __name__ == '__main__':
    main()