"""Summary
"""
from time import time
import numpy as np


class MeasurementNoiseModel():

    """This class encodes the noise in the measurements of the kinematics for the KF
    This noise emerges from two sources: the inter-subject variability in gait (i.e. each subject walks differently)
    and the noise in the measurements themselves
    
    Attributes:
        dim (int): The number of measurements
        DO_VELOCITIES (bool): Whether to use the velocity heteroscedastic covariances (usually false)
        DO_XSUB_R (bool): Whether to use the heteroscedastic x-subject variability
        meas_config (TYPE): The config string that determines which measurements to use
        R (np matrix): The full measurement covariance
        R_mean (TYPE): An average measurement covariance
        R_meas (np matrix): The measurement covariance due to noise in the measurements
        R_xsub (np matrix): The heteroscedastic x-subject variability
    
    
    """
    
    def __init__(self, R_meas, covar_filepath='GaitModel/covar_fourier_normalizedsL.csv', meas_config='full',DO_XSUB_R=True):
        """Initialize
        
        Args:
            R_meas (np matrix): The measurement noise matrix pertaining to the noise in the measurements themselves
            covar_filepath (str, optional): the filepath to the inter-subject covariance model
            meas_config (str, optional): a string describing which measurements to use
            DO_XSUB_R (bool, optional): whether to use the inter-subject covariance model
        
        """
        self.dim = R_meas.shape[0]
        self.R_meas = R_meas
        self.meas_config = meas_config


        print(f'CONFIG: {self.meas_config}')

        self.DO_XSUB_R = DO_XSUB_R
        self.DO_VELOCITIES = not False

        (self.best_fit_params_11,
            self.best_fit_params_12,
            self.best_fit_params_13,
            self.best_fit_params_14,
            self.best_fit_params_15,
            self.best_fit_params_16,
            self.best_fit_params_22,
            self.best_fit_params_23,
            self.best_fit_params_24,
            self.best_fit_params_25,
            self.best_fit_params_26,
            self.best_fit_params_33,
            self.best_fit_params_34,
            self.best_fit_params_35,
            self.best_fit_params_36,
            self.best_fit_params_44,
            self.best_fit_params_45,
            self.best_fit_params_46,
            self.best_fit_params_55,
            self.best_fit_params_56,
            self.best_fit_params_66) = self.loadCovarCurves(covar_filepath)

        self.gain_schedule_R(0)

    def loadCovarCurves(self, filename):
        """Helper function that loads the heteroscedastic noise covariance model
        Loads in foot, foot vel, shank, shank vel, thigh, thigh vel, pelvis, pelvis vel
        
        Args:
            filename (TYPE): the filepath to load
        
        Returns:
            the model coefficients
        """
        data = np.loadtxt(filename,delimiter=',')

        best_fit_params_11 = data[0,:]
        best_fit_params_12 = data[1,:]
        best_fit_params_13 = data[2,:]
        best_fit_params_14 = data[3,:]
        best_fit_params_15 = data[4,:]
        best_fit_params_16 = data[5,:]

        best_fit_params_22 = data[6,:]
        best_fit_params_23 = data[7,:]
        best_fit_params_24 = data[8,:]
        best_fit_params_25 = data[9,:]
        best_fit_params_26 = data[10,:]

        best_fit_params_33 = data[11,:]
        best_fit_params_34 = data[12,:]
        best_fit_params_35 = data[13,:]
        best_fit_params_36 = data[14,:]

        best_fit_params_44 = data[15,:]
        best_fit_params_45 = data[16,:]
        best_fit_params_46 = data[17,:]

        best_fit_params_55 = data[18,:]
        best_fit_params_56 = data[19,:]

        best_fit_params_66 = data[20,:]

        return (best_fit_params_11,
            best_fit_params_12,
            best_fit_params_13,
            best_fit_params_14,
            best_fit_params_15,
            best_fit_params_16,
            best_fit_params_22,
            best_fit_params_23,
            best_fit_params_24,
            best_fit_params_25,
            best_fit_params_26,
            best_fit_params_33,
            best_fit_params_34,
            best_fit_params_35,
            best_fit_params_36,
            best_fit_params_44,
            best_fit_params_45,
            best_fit_params_46,
            best_fit_params_55,
            best_fit_params_56,
            best_fit_params_66)

    def compute_R_xsub(self,phase_estimate):
        """Update the inter-subject heteroscedastic covariance matrix given the current phase
        
        Args:
            phase_estimate (float): the phase at whih to evaluate the 
            heteroscedastic covariance matrix given the current phase
        
        """
        phase = np.linspace(0,1,150)
        # print(self.best_fit_params_11.size)
        # print('phase_estimate.shape')
        # print(phase_estimate.shape)
        R11 = np.interp(phase_estimate, phase, self.best_fit_params_11)
        R12 = np.interp(phase_estimate, phase, self.best_fit_params_12)
        R13 = np.interp(phase_estimate, phase, self.best_fit_params_13)
        R14 = np.interp(phase_estimate, phase, self.best_fit_params_14)
        R15 = np.interp(phase_estimate, phase, self.best_fit_params_15)
        R16 = np.interp(phase_estimate, phase, self.best_fit_params_16)

        R22 = np.interp(phase_estimate, phase, self.best_fit_params_22)
        R23 = np.interp(phase_estimate, phase, self.best_fit_params_23)
        R24 = np.interp(phase_estimate, phase, self.best_fit_params_24)
        R25 = np.interp(phase_estimate, phase, self.best_fit_params_25)
        R26 = np.interp(phase_estimate, phase, self.best_fit_params_26)

        R33 = np.interp(phase_estimate, phase, self.best_fit_params_33)
        R34 = np.interp(phase_estimate, phase, self.best_fit_params_34)
        R35 = np.interp(phase_estimate, phase, self.best_fit_params_35)
        R36 = np.interp(phase_estimate, phase, self.best_fit_params_36)

        R44 = np.interp(phase_estimate, phase, self.best_fit_params_44)
        R45 = np.interp(phase_estimate, phase, self.best_fit_params_45)
        R46 = np.interp(phase_estimate, phase, self.best_fit_params_46)

        R55 = np.interp(phase_estimate, phase, self.best_fit_params_55)
        R56 = np.interp(phase_estimate, phase, self.best_fit_params_56)

        R66 = np.interp(phase_estimate, phase, self.best_fit_params_66)

        if not self.DO_VELOCITIES:
            R12 = 0
            R14 = 0

            R22 = 0
            R23 = 0
            R24 = 0
            R25 = 0
            R26 = 0

            R34 = 0

            R44 = 0
            R45 = 0
            R46 = 0


        # R_con = np.zeros(len(self.meas_config)*2)

        if self.meas_config == 'full':

            self.R_xsub = np.array([[R11, R12, R13, R14, R15, R16],
                [R12, R22, R23, R24, R25, R26],
                [R13, R23, R33, R34, R35, R36],
                [R14, R24, R34, R44, R45, R46],
                [R15, R25, R35, R45, R55, R56],
                [R16, R26, R36, R46, R56, R66]
                ])

        elif self.meas_config == 'heelForward':
            self.R_xsub = np.array([
                [R11, R12, R13, R14, R15],
                [R12, R22, R23, R24, R25],
                [R13, R23, R33, R34, R35],
                [R14, R24, R34, R44, R45],
                [R15, R25, R35, R45, R55]
                ])

        elif self.meas_config == 'heelUp':
            self.R_xsub = np.array([
                [R11, R12, R13, R14, R16],
                [R12, R22, R23, R24, R26],
                [R13, R23, R33, R34, R36],
                [R14, R24, R34, R44, R46],
                [R16, R26, R36, R46, R66]
                ])

        elif self.meas_config == 'angles':
            self.R_xsub = np.array([[R11, R12, R13, R14],
                [R12, R22, R23, R24],
                [R13, R23, R33, R34],
                [R14, R24, R34, R44]
                ])

        return self.R_xsub

    def gain_schedule_R(self,phase):
        """Return the total measurement noise matrix:
            the sum of the measurement noise matrix and the xsubject covariance matrix
        
        Args:
            phase (float): The phase at which to obtain the total noise matrix
        
        Returns:
            np mat: the total noise matrix
        
        
        """
        self.R = self.R_meas.astype(float)
        # print(self.R_meas)
        # input()

        if self.DO_XSUB_R:
            # print(self.meas_config)
            self.compute_R_xsub(phase)
            # print(self.R_xsub)
            # input()
            self.R += self.R_xsub

        return self.R


    
    def calc_R_mean(self,):
        """Utility function to get an average R covariance
        
        Returns:
            np matrix: The average measurement covariance
        """

        self.R_mean = np.copy(self.R_meas)


        if self.DO_XSUB_R:

            R11 = np.mean(self.best_fit_params_11)
            R12 = np.mean(self.best_fit_params_12)
            R13 = np.mean(self.best_fit_params_13)
            R14 = np.mean(self.best_fit_params_14)
            R15 = np.mean(self.best_fit_params_15)
            R16 = np.mean(self.best_fit_params_16)

            R22 = np.mean(self.best_fit_params_22)
            R23 = np.mean(self.best_fit_params_23)
            R24 = np.mean(self.best_fit_params_24)
            R25 = np.mean(self.best_fit_params_25)
            R26 = np.mean(self.best_fit_params_26)

            R33 = np.mean(self.best_fit_params_33)
            R34 = np.mean(self.best_fit_params_34)
            R35 = np.mean(self.best_fit_params_35)
            R36 = np.mean(self.best_fit_params_36)

            R44 = np.mean(self.best_fit_params_44)
            R45 = np.mean(self.best_fit_params_45)
            R46 = np.mean(self.best_fit_params_46)

            R55 = np.mean(self.best_fit_params_55)
            R56 = np.mean(self.best_fit_params_56)

            R66 = np.mean(self.best_fit_params_66)

            if not self.DO_VELOCITIES:
                R12 = 0
                R14 = 0

                R22 = 0
                R23 = 0
                R24 = 0
                R25 = 0
                R26 = 0

                R34 = 0

                R44 = 0
                R45 = 0
                R46 = 0


            if self.meas_config == 'full':

                self.R_xsub = np.array([[R11, R12, R13, R14, R15, R16],
                    [R12, R22, R23, R24, R25, R26],
                    [R13, R23, R33, R34, R35, R36],
                    [R14, R24, R34, R44, R45, R46],
                    [R15, R25, R35, R45, R55, R56],
                    [R16, R26, R36, R46, R56, R66]
                    ])

            elif self.meas_config == 'heelForward':
                self.R_xsub = np.array([
                    [R11, R12, R13, R14, R15],
                    [R12, R22, R23, R24, R25],
                    [R13, R23, R33, R34, R35],
                    [R14, R24, R34, R44, R45],
                    [R15, R25, R35, R45, R55]
                    ])

            elif self.meas_config == 'heelUp':
                self.R_xsub = np.array([
                    [R11, R12, R13, R14, R16],
                    [R12, R22, R23, R24, R26],
                    [R13, R23, R33, R34, R36],
                    [R14, R24, R34, R44, R46],
                    [R16, R26, R36, R46, R66]
                    ])

            elif self.meas_config == 'angles':
                self.R_xsub = np.array([[R11, R12, R13, R14],
                    [R12, R22, R23, R24],
                    [R13, R23, R33, R34],
                    [R14, R24, R34, R44]
                    ])

            print(self.R_meas)
            print(np.linalg.norm(self.R_meas))
            print("R_meas b4 ^^")
            self.R_mean += self.R_xsub
            print(self.R_meas)
            print(np.linalg.norm(self.R_meas))
            print("R_meas after ^^")

        return self.R_mean

