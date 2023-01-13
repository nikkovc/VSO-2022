"""Made by Leo. Contains the full EKF for phase
"""
from time import time
import numpy as np
from arctanMapFuncs import *

class PhaseEKF():

    """Summary
    
    Attributes:
        F (TYPE): Description
        F0 (TYPE): Description
        gait_model (TYPE): Description
        meas_config (TYPE): Description
        measurement_noise_model (TYPE): Description
        P0 (TYPE): Description
        P_covar_estimate (TYPE): Description
        P_covar_update (TYPE): Description
        P_prev_covar_estimate (TYPE): Description
        Q_rate (TYPE): Description
        R (TYPE): Description
        R_mean (TYPE): Description
        SSE (int): Description
        timing_gain_schedule_R (int): Description
        timing_measure (int): Description
        timing_step (int): Description
        timing_update (int): Description
        torque_profile (TYPE): Description
        x0 (TYPE): Description
        x_prev_state_estimate (TYPE): Description
        x_state_estimate (TYPE): Description
        x_state_update (TYPE): Description
        y_residual (TYPE): Description
        z_measured (TYPE): Description
        z_model (TYPE): Description
    """
    
    def __init__(self, gait_model, torque_profile, R_meas,
                    sigma_q_phase=0,sigma_q_phase_dot=5.1e-4):
        """The Extended Kalman Filter, which linearizes about the current estimated state
        to approximate the nonlinear systems as linear
        
            Args:
                gait_model (class): the model that maps the gait state to measured kinematics
                torque_profile (class): the torque profile the exoskeleton will apply, as a function of gait state
                measurement_noise_model (class): Contains the measurement covariances 
                sigma_q_phase (float, optional): the process noise for phase (should be noiseless)
                sigma_q_phase_dot (float, optional): the process noise for phase dot
        
        Args:
            gait_model (TYPE): Description
            torque_profile (TYPE): Description
            measurement_noise_model (TYPE): Description
            sigma_q_phase (int, optional): Description
            sigma_q_phase_dot (float, optional): Description
        """
        # Initialize state vector and covariance
        # State vector contains, in order: phase, phase rate, stride length, incline
        self.x0 = np.array([[0],[1]])
        self.P0 = 1e-3 * np.eye(2) #empirically arrived

        
        # print(self.x0)
        # Initialize state transition matrix
        self.F0 = np.array([[1, 1/180],[0,1]])
        # print(self.F0)


        # Q is initialized as a covariance rate, which is scaled by the time step to maintain consistent bandwidth behavior 
        self.Q_rate = np.diag([0,sigma_q_phase_dot**2])/(1e-2)
        # lambda_psL = -0.031
        # epsilon = 1e-20
        # self.Q_rate = np.zeros((4,4))
        # self.Q_rate[1:3,1:3] = sigma_q_phase_dot**2 * np.array([[1, lambda_psL],[lambda_psL, lambda_psL**2]])
        # self.Q_rate[2,2] += epsilon
        # self.Q_rate[3,3] = sigma_q_incline**2
        print(self.Q_rate)


        self.R_meas = R_meas
        self.torque_profile = torque_profile
        self.gait_model = gait_model


        self.x_state_estimate = None
        self.P_covar_estimate = None

        self.F = None

        self.SSE = 0

        #timing internal variables
        self.timing_step = 0
        self.timing_measure = 0
        self.timing_update = 0
        self.timing_gain_schedule_R = 0

    def step(self, i, dt):
        """Step function that encodes the prediction step of the EKF
        Follows standard EKF formulae
        
        Args:
            i: int, current iteration count
            dt: float, the time step
            
        """
        time0 = time()

        first=(i==0)

        if first:
            # print('1st step')
            F = np.array([[1, 1/100.0],[0,1]])
            Q = self.Q_rate * 1/100.0
            self.x_state_estimate = F @ self.x0
            self.P_covar_estimate = (F @ self.P0 @ F.transpose()) + Q

           
        else:
            F = np.array([[1, dt],[0,1]])
            Q = self.Q_rate * dt
            self.x_state_estimate = F @ self.x_prev_state_estimate
            self.P_covar_estimate = (F @ self.P_prev_covar_estimate @ F.transpose()) + Q

        self.x_state_estimate[0] = self.x_state_estimate[0].item(0) % 1

        time1 = time()
        self.timing_step = time1 - time0

    # Measurement function that conducts the measurement step of the EKF
    def update(self, i, dt, data):
        """Summary
        
        Args:
            i (int): current iteration count
            data (np vector (N,)): the measured kinematics
        """
        time0 = time()
        self.z_measured = data.reshape(-1,1)

        # Extract state vector elements
        phase_estimate = self.x_state_estimate[0].item(0)
        phase_dot_estimate = self.x_state_estimate[1].item(0)


        # Compute the modeled shank angles based on the regressed gait_model
        shankAngle_estimate = self.gait_model.returnShankAngle(phase_estimate)

        #estimate modeled velocity by multiplying the phase rate by the partials wrt phase
        evalshankAngleDeriv_dphase = self.gait_model.returnShankAngleDeriv_dphase(phase_estimate)

        shankAngleVel_estimate = phase_dot_estimate * evalshankAngleDeriv_dphase
        self.z_model = [shankAngle_estimate, shankAngleVel_estimate]

        self.z_model = np.array(self.z_model).reshape(-1,1)
        self.y_residual = self.z_measured - self.z_model

  
        self.SSE = (self.y_residual.T @ np.linalg.solve(self.R_mean, self.y_residual) ) + self.SSE
    

        # calculate the H matrix for the phase estimator
        # H is the Jacobian of the measurements wrt the state vector

        dh11 = evalshankAngleDeriv_dphase
        dh12 = 0

        dh21 = phase_dot_estimate * self.gait_model.returnShankAngle2ndDeriv_dphase2(phase_estimate,strideLength_estimate,incline_estimate)
        dh22 = evalshankAngleDeriv_dphase

        H = np.array([[dh11, dh12],
            [dh21, dh22],
            ])

        S_covariance = H @ self.P_covar_estimate @ H.transpose() + self.R_meas

        # Compute Kalman Gain
        K_gain = self.P_covar_estimate @ H.transpose() @ np.linalg.inv(S_covariance)

        self.x_state_update = self.x_state_estimate + K_gain @ self.y_residual

        # Modulo phase to be between 0 and 1
        self.x_state_update[0] = self.x_state_update[0].item(0) % 1

        # Update covariance
        self.P_covar_update = (np.eye(4) - K_gain @ H) @ self.P_covar_estimate

        self.x_prev_state_estimate = self.x_state_update
        self.P_prev_covar_estimate = self.P_covar_update
        time1 = time()
        self.timing_update = time1 - time0


    def get_torque(self):
        """Returns exoskeleton torque given the current state vector
        
        Returns:
            float: the desired torque
        """
        phase_estimate = self.x_state_update[0].item(0)
        phase_dot_estimate = self.x_state_update[1].item(0)        
        return self.torque_profile.evalTorqueProfile(phase_estimate,strideLength_estimate,incline_estimate)




    def set_SSE(self,newSSE):
        """Setter method that sets SSE
        
        Args:
            newSSE (float): the new SSE to set
        """
        self.SSE = newSSE

    def getSSE(self,):
        """Getter method that returns SSE
        
        Returns:
            float: SSE
        """
        return self.SSE

    def get_z_measured(self,):
        """Getter method that returns z_measured
        
        Returns:
            np matrix: z_measured
        """
        return self.z_measured

    def set_y_residual(self,y_residual_new):
        """Setter method that sets y_residual
        
        Args:
            y_residual_new (np vector): the new y_residual
        """
        self.y_residual = y_residual_new

    def set_prev_x_state_estimate(self,x_prev_state_estimate_new):
        """Setter method that sets x_prev_state_estimate
        
        Args:
            x_prev_state_estimate_new (np vector): the new x_prev_state_estimate
        """
        self.x_prev_state_estimate[0, 0] = x_prev_state_estimate_new[0]
        self.x_prev_state_estimate[1, 0] = x_prev_state_estimate_new[1]
        self.x_prev_state_estimate[2, 0] = x_prev_state_estimate_new[2]
        self.x_prev_state_estimate[3, 0] = x_prev_state_estimate_new[3]

    def set_prev_P_covar_estimate(self,P_prev_covar_estimate_new):
        """Setter method that sets P_prev_covar_estimate
        
        Args:
            P_prev_covar_estimate_new (np matrix): The new P_prev_covar_estimate
        """
        self.P_prev_covar_estimate = P_prev_covar_estimate_new

    def set_x_state_estimate(self,x_state_estimate_new):
        """Setter method that sets x_state_estimate
        
        Args:
            x_state_estimate_new (np vector): the new x_state_estimate
        """
        self.x_state_estimate[0, 0] = x_state_estimate_new[0]
        self.x_state_estimate[1, 0] = x_state_estimate_new[1]
        self.x_state_estimate[2, 0] = x_state_estimate_new[2]
        self.x_state_estimate[3, 0] = x_state_estimate_new[3]

    def get_x_state_estimate(self,):
        """Getter method that returns x_state_estimate
        
        Returns:
            np matrix: x_state_estimate
        """
        return self.x_state_estimate

    def set_P_covar_estimate(self,P_covar_estimate_new):
        """Setter method that sets P_covar_estimate
        
        Args:
            P_covar_estimate_new (np matrix): The new P_covar_estimate
        """
        self.P_covar_estimate = P_covar_estimate_new
        


    def get_x_state_update(self,):
        """Getter method that returns x_state_update
        
        Returns:
            np matrix: x_state_update
        """
        return self.x_state_update

    def set_x_state_update(self,x_state_update_new):
        """Setter method that sets x_state_update
        
        Args:
            x_state_update_new (np vector): the new x_state_update
        """

        self.x_state_update[0, 0] = x_state_update_new[0]
        self.x_state_update[1, 0] = x_state_update_new[1]
        self.x_state_update[2, 0] = x_state_update_new[2]
        self.x_state_update[3, 0] = x_state_update_new[3]

    def set_P_covar_update(self,P_covar_update_new):
        """Setter method that sets P_covar_update
        
        Args:
            P_covar_update_new (np matrix): The new P_covar_update
        """
        self.P_covar_update = P_covar_update_new


