""" Plots the results of an exoskeleton trial """
import numpy as np
from time import strftime
np.set_printoptions(precision=4)
# import matplotlib.pyplot as plt


import matplotlib
# matplotlib.use('Agg')

import matplotlib.pyplot as plt



def main():

    # SUBJECT_LEG_LENGTH = 0.935 #AB01
    # # SUBJECT_LEG_LENGTH = 0.975 # AB02
    # # SUBJECT_LEG_LENGTH = 0.965 #AB03
    # # SUBJECT_LEG_LENGTH = 0.96 #AB04
    # # SUBJECT_LEG_LENGTH = 0.96 #AB05
    # # SUBJECT_LEG_LENGTH = 0.845 #AB06

    
    # data = np.loadtxt("AB02/20220404-23_AE0107Standalone_PB_EKF_Test_AB02_TPVC_03.csv", delimiter=',')
    # data = np.loadtxt("AB03/20220328-21_AE5105Standalone_PB_EKF_Test_AB03_Reverse_04.csv", delimiter=',')
    # data = np.loadtxt("AB01/20220323-22_AE2215Standalone_PB_EKF_Test_AB01_Backward_03.csv", delimiter=',')
    # data = np.loadtxt("Wavefield/Run 2/Mars/mars_6.csv", delimiter=',')
    data = np.loadtxt("AB10/20220916-20_AE4802Standalone_PB_EKF_Test_TPVA_04.csv", delimiter=',')


    timeSec=data[:,0]
    accelVec_corrected=data[:,1:4]


    gyroVec_corrected=data[:,4:7]
    # x_state_AE = data[:,10:16]

    ankleAngle = data[:,44]
    isOverriding = data[:,73]
    roll = data[:,58]
    pitch = data[:,59]
    yaw = data[:,60]

    x_state_PE = data[:,25:29]
    z_measured_act = data[:,29:35]
    z_model_act = data[:,35:41]
    HSDetected = data[:,24]
    strideLength_update_descaled = data[:,45]

    heelAccForward_meas = data[:,61] #92
    heelPosForward_meas_filt = data[:,62] #93
    heelPosUp_meas_filt = data[:,63] #93

    actTorque = data[:,49]
    desTorque = data[:,50]


    heelAccSide_meas = data[:,68] #70
    heelAccUp_meas = data[:,69] #71

    heelAccForward_meas_fromDeltaVelocity = data[:,70] #92
    heelAccSide_meas_fromDeltaVelocity = data[:,71] #70
    heelAccUp_meas_fromDeltaVelocity = data[:,72]#71

    heelAccForward_meas_fromDeltaVelocity_norm = np.sqrt(heelAccForward_meas_fromDeltaVelocity**2 +
                                                            heelAccSide_meas_fromDeltaVelocity**2 +
                                                            (heelAccUp_meas_fromDeltaVelocity)**2)

    phase_HPEKF = data[:,74]
    phase_rate_HPEKF = data[:,75]
    sL_HPEKF = data[:,76]
    incline_HPEKF = data[:,77]

    HPEKF_SSE = data[:,78] #71
    EKF_SSE = data[:,79] #71

    # MEM_ALLOC = data[:,80] 
    # MEM_FREE = data[:,81]


    diff_SSEs = HPEKF_SSE - EKF_SSE

    HS_idxs = np.diff(HSDetected)
    HS_idxs = np.concatenate((HS_idxs,np.array([0])))

    HS_idxs = HS_idxs > 0

    print(HS_idxs)


    diff_SSEs = EKF_SSE[HS_idxs] - HPEKF_SSE[HS_idxs]

    dt = np.diff(timeSec)
    freqData = 1/dt


    plt.figure()
    plt.hist(freqData)
    plt.title('freqData')
    plt.savefig("time_")


    #PLOT STATES
    fig, axs = plt.subplots(6,1,sharex=True,figsize=(10,6))

    axs[0].plot(timeSec, x_state_PE[:,0], label=r"$phase_{hardware}$")
    axs[0].plot(timeSec, phase_HPEKF, label=r"$phase_{HPEKF,hardware}$")
    

    axs[0].plot(timeSec, HSDetected, label=r"$HSDetected$")
    axs[0].plot(timeSec, isOverriding,'k', label=r"$isOverriding_{hardware}$")
    axs[0].legend()

    axs[1].plot(timeSec, x_state_PE[:,1], label=r"$phasedot_{hardware}$")
    axs[1].plot(timeSec, phase_rate_HPEKF, label=r"$phase_rate_{HPEKF,hardware}$")
    axs[1].legend()

    axs[2].plot(timeSec, strideLength_update_descaled, label=r"$Stride Length_{hardware}$")
    axs[2].plot(timeSec, sL_HPEKF, label=r"$sL_{HPEKF,hardware}$")
    axs[2].plot(timeSec, isOverriding,'k', label=r"$isOverriding_{hardware}$")
    axs[2].legend()

    axs[3].plot(timeSec, x_state_PE[:,3], label=r"$Ramp_{hardware}$")
    axs[3].plot(timeSec, incline_HPEKF, label=r"$Ramp_{HPEKF,hardware}$")
    axs[3].plot(timeSec, isOverriding,'k', label=r"$isOverriding_{hardware}$")
    axs[3].legend()

    axs[4].plot(timeSec, actTorque, label=r"$Act Torque$")
    axs[4].plot(timeSec, desTorque, label=r"$Des Torque$")
    axs[4].plot(timeSec, isOverriding*10,'k', label=r"$isOverriding_{hardware}$")
    axs[4].legend()

    axs[5].plot(timeSec, HPEKF_SSE, label=r"$HPEKF SSE$")
    axs[5].plot(timeSec, EKF_SSE, label=r"$EKF SSE$")
    axs[5].plot(timeSec, HSDetected*20000, label=r"$HSDetected$")
    axs[5].plot(timeSec, isOverriding*20000,'k', label=r"$isOverriding_{hardware}$")
    axs[5].legend()


    


    axs[-1].set_xlabel("time (sec)")
    print("this is done")

    plt.savefig("states_")

    # plt.show()


    #PLOT EXO IMU DATA
    if True:

        fig, axs = plt.subplots(3,2,sharex=True,figsize=(10,6))

        axs[0,0].plot(timeSec, accelVec_corrected[:,0], label=r"$Accel X$")
        axs[0,0].legend()
        axs[1,0].plot(timeSec, accelVec_corrected[:,1], label=r"$Accel Y$")
        axs[1,0].legend()
        axs[2,0].plot(timeSec, accelVec_corrected[:,2], label=r"$Accel Z$")
        axs[2,0].legend()
        axs[0,1].plot(timeSec, gyroVec_corrected[:,0], label=r"$Gyro X$")
        axs[0,1].legend()
        axs[1,1].plot(timeSec, gyroVec_corrected[:,1], label=r"$Gyro Y$")
        axs[1,1].legend()
        axs[2,1].plot(timeSec, gyroVec_corrected[:,2], label=r"$Gyro Z$")
        axs[2,1].legend()
        axs[-1,0].set_xlabel("time (sec)")
        axs[-1,1].set_xlabel("time (sec)")
        print("this is done")
        # plt.show()

        # plt.savefig(filename)

    #PLOT KINEMATICS
    if True:

        fig, axs = plt.subplots(6,1,sharex=True,figsize=(10,6))
        axs[0].set_title("Kinematics")

        # axs[0].plot(timeSec, plot_data[:,1], label=r"$phase_{hardware}$")
        axs[0].plot(timeSec, z_measured_act[:,0], label=r"$foot angle, measured$")
        axs[0].plot(timeSec, z_model_act[:,0], label=r"$foot angle, model$")

        axs[0].plot(timeSec, HSDetected*1e1, label=r"$HSDetected$")
        axs[0].plot(timeSec, isOverriding*1e1,'k', label=r"$isOverriding$")
        axs[0].set_ylim([-70,50])


        axs[0].legend()


        axs[1].plot(timeSec, z_measured_act[:,1], label=r"$foot angle vel, measured$")
        axs[1].plot(timeSec, z_model_act[:,1], label=r"$foot angle vel, model$")

        axs[1].plot(timeSec, HSDetected*1e1, label=r"$HSDetected$")
        axs[1].plot(timeSec, isOverriding*1e1,'k', label=r"$isOverriding$")

        axs[1].legend()



        axs[2].plot(timeSec, z_measured_act[:,2], label=r"$shank angle, measured$")
        axs[2].plot(timeSec, z_model_act[:,2], label=r"$shank angle, model$")
        axs[2].plot(timeSec, HSDetected*1e1, label=r"$HSDetected$")
        axs[2].plot(timeSec, isOverriding*1e1,'k', label=r"$isOverriding$")
        axs[2].set_ylim([-70,50])

        axs[2].legend()


        axs[3].plot(timeSec, z_measured_act[:,3], label=r"$shank angle vel, measured$")
        axs[3].plot(timeSec, z_model_act[:,3], label=r"$shank angle vel, model$")
        axs[3].plot(timeSec, HSDetected*1e1, label=r"$HSDetected$")
        axs[3].plot(timeSec, isOverriding*1e1,'k', label=r"$isOverriding$")
        # axs[3].set_ylim([-70,50])

        axs[3].legend()



        axs[4].plot(timeSec, z_measured_act[:,4], label=r"$foot position forward measured$")
        axs[4].plot(timeSec, z_model_act[:,4], label=r"$foot position forward model$")
        axs[4].plot(timeSec, HSDetected*1, label=r"$HSDetected$")
        axs[4].plot(timeSec, isOverriding*1,'k', label=r"$isOverriding$")

        axs[4].legend()

        axs[5].plot(timeSec, z_measured_act[:,5], label=r"$foot position up measured$")
        axs[5].plot(timeSec, z_model_act[:,5], label=r"$foot position up model$")
        axs[5].plot(timeSec, HSDetected*1, label=r"$HSDetected$")
        axs[5].plot(timeSec, isOverriding*1,'k', label=r"$isOverriding$")

        axs[5].legend()


        plt.savefig("meas_")

    #PLOT FOOT IMU ACCEL
    if True:
        fig, axs = plt.subplots(4,1,sharex=True,figsize=(10,6))

        axs[0].plot(timeSec, heelAccForward_meas, label=r"$heel acc$")
        axs[0].plot(timeSec, heelAccForward_meas_fromDeltaVelocity, label=r"$heel acc from delta vel$")

        axs[0].plot(timeSec, HSDetected*1e1, label=r"$HSDetected$")
        axs[0].plot(timeSec, isOverriding*1e1,'k', label=r"$isOverriding$")

        axs[1].plot(timeSec, heelAccSide_meas, label=r"$heel side$")
        axs[1].plot(timeSec, heelAccSide_meas_fromDeltaVelocity, label=r"$heel side from delta vel$")

        axs[1].plot(timeSec, HSDetected*1e1, label=r"$HSDetected$")
        axs[1].plot(timeSec, isOverriding*1e1,'k', label=r"$isOverriding$")

        axs[2].plot(timeSec, heelAccUp_meas, label=r"$heel up$")
        axs[2].plot(timeSec, heelAccUp_meas_fromDeltaVelocity, label=r"$heel up from delta vel$")

        axs[2].plot(timeSec, HSDetected*1e1, label=r"$HSDetected$")
        axs[2].plot(timeSec, isOverriding*1e1,'k', label=r"$isOverriding$")

        axs[3].plot(timeSec, heelAccForward_meas_fromDeltaVelocity_norm, label=r"$heel accel norm$")
        axs[3].plot(timeSec, HSDetected*1e1, label=r"$HSDetected$")
        axs[3].plot(timeSec, isOverriding*1e1,'k', label=r"$isOverriding$")


        


        axs[0].legend()
        axs[1].legend()
        axs[2].legend()


       


        print("this is done")
        plt.savefig("heel_")

    if True:
        fig, axs = plt.subplots(sharex=True,figsize=(10,6))
        axs.plot(timeSec[1:], dt, label=r"$dt$")

    if True:
        fig, axs = plt.subplots(sharex=True,figsize=(10,6))
        axs.hist(diff_SSEs,bins=100)
        mean_diff_SSEs = np.mean(diff_SSEs)
        std_diff_SSEs = np.std(diff_SSEs)

        perc1 = np.percentile(diff_SSEs, 1)
        perc99 = np.percentile(diff_SSEs, 99)
        plt.axvline(x=mean_diff_SSEs, color="black", linestyle="-")
        # plt.axvline(x=mean_diff_SSEs - 2.576*std_diff_SSEs, color="red", linestyle="-")
        plt.axvline(x=perc99, color="red", linestyle="-")
        plt.axvline(x=perc1, color="red", linestyle="-")

    # if True:
    #     fig, axs = plt.subplots(sharex=True,figsize=(10,6))

    #     axs.plot(timeSec, MEM_ALLOC, color="red", linestyle="-")
    #     axs.plot(timeSec, MEM_FREE, color="blue", linestyle="-")

    plt.show()




if __name__ == '__main__':
    main()