"""Contains a class that handles the detection of heelstrikes
"""
import numpy as np
import matplotlib.pyplot as plt
from time import time


class HeelStrikeDetector():

	"""This class contains a method and logic for the detection of heelstrike events
	
	Attributes:
		accelZThreshold (float): The threshold for detecting high-frequency accel signal
		accelZVec_buffer (list): The running buffer for filtered accel signals
		gyroYSlopeThreshold (int): The threshold for detecting gyro Y signals
		gyroYVec_buffer (list): The running buffer for filtered gyro signals
		HS_analysis_window (int): The number of samples to buffer and analyze
		HSDetected (bool): If a heel-strike (HS) was detected
		timeVec_buffer (list): The running buffer for time (in seconds)
		timing_detectHS (float): Runtime for HS detection
	"""
	
	def __init__(self, HS_analysis_window):
		"""Initialize
		
		Args:
			HS_analysis_window (int): The number of samples to buffer and analyze
		"""
		self.HS_analysis_window = HS_analysis_window

		self.accelNormBuffer = []
		self.timeVec_buffer = []
		self.HSDetected = False
		self.timeHSStart = 0
		self.accelThreshold = 10 #m/s

	def detectHS(self,timeSec, footAngle, footAngleVel, heelAccNorm):
		"""Detects if a heel-strike (HS) event has occurred given gyro and accelerometer signals
		
		Args:
			timeSec (TYPE): Description
			gyroY_filter (TYPE): Description
			accelZ_filter (TYPE): Description
		
		Returns:
			bool: If a HS was detected
		
		"""
		time0 = time()

		self.accelNormBuffer.append(heelAccNorm) 
		# self.gyroYVec_buffer.append(gyroY_filter) 
		# self.timeVec_buffer.append(timeSec)

		N = len(self.accelNormBuffer)

		if N > self.HS_analysis_window: #ensure the buffer stays a constant length
			self.accelNormBuffer.pop(0)
		# 	self.gyroYVec_buffer.pop(0)
		# 	self.timeVec_buffer.pop(0)
		# 	# print(self.timeVec_buffer)
		# 	# input()
		# 	# print(len(self.timeVec_buffer))

		
		# print(gyroYWindowed)
		# accelZWindowed = np.array(self.accelZVec_buffer)
		# accelZWindowed_abs = np.abs(accelZWindowed)

		# gyroYWindowed = np.array(self.gyroYVec_buffer)
		# timeWindowed = np.array(self.timeVec_buffer)

		# dt = (timeWindowed[-1] - timeWindowed[0])

		# if dt < 1e-6: #the first iteration will likely have dt == 0, prevent that from causing div by zero errors
		# 	dt = 1/100

		isFootAng_Positive = footAngle > 0 and footAngle <= 30

		isFootAngVel_Negative = footAngleVel < 0

		isAccelMagnitude = np.mean(self.accelNormBuffer) >= self.accelThreshold

		# avgGyroYSlope = (gyroYWindowed[-1] - gyroYWindowed[0] )
		# # print(f'Timediff: {(timeWindowed[-1] - timeWindowed[0])}')
		# # print(f'gyroYSlope: {avgGyroYSlope}')

		# # gyroYSlope_bool = (np.abs(avgGyroYSlope) >= 0.67*np.abs(self.gyroYSlopeThreshold)) and (np.abs(avgGyroYSlope) <= 1.5*np.abs(self.gyroYSlopeThreshold)) 

		# sign_slopes = np.diff(np.sign(gyroYWindowed))
		# zeroCrossingEvents_gyroY = len(sign_slopes[np.nonzero(sign_slopes)])
		# numZeroCrossingEvents_gyroY_bool = zeroCrossingEvents_gyroY == 1 # and zeroCrossingEvents_gyroY <= 2
		
		# gyroYSlopeSign = np.sign(avgGyroYSlope)

		# gyroYSlope_bool = avgGyroYSlope > 0
		# # print(type(gyroYWindowed))
		# isGyroMagnitude = len(gyroYWindowed[gyroYWindowed < -2]) >= 2
		# isAccelMagnitude = len(accelZWindowed[accelZWindowed_abs >= self.accelZThreshold]) >= 2
		# # print(len(accelZWindowed[accelZWindowed >= self.accelZThreshold]))
		isOutHSCooldown = (timeSec - self.timeHSStart) > 0.4
		# isGyroMagnitude = True
		# HSDetected = numAccelEvents_bool and isOutHSCooldown and gyroYSlope_bool
		# self.HSDetected = numZeroCrossingEvents_gyroY_bool and isGyroMagnitude and isAccelMagnitude and gyroYSlope_bool #uncommented for new HS
		# HSDetected = isAccelMagnitude
		self.HSDetected = isFootAng_Positive and isFootAngVel_Negative and isAccelMagnitude and isOutHSCooldown
		if self.HSDetected:
			self.timeHSStart = timeSec

		time1 = time()
		self.timing_detectHS = time1 - time0


		return self.HSDetected


if __name__ == '__main__':

	# filename = 'AB01/20220323-22_AE0629Standalone_PB_EKF_Test_AB01_Forward_02.csv' #gets lost, pauses, use to tune HSDetector
	# filename = 'AB01/20220323-22_AE2215Standalone_PB_EKF_Test_AB01_Backward_03.csv' #pauses, use to tune HSDetector
	# filename = 'AB03/20220328-21_AE1509Standalone_PB_EKF_Test_AB03_Forward_02.csv'
	# filename = 'AB03/20220328-21_AE3436Standalone_PB_EKF_Test_AB03_Forward03.csv'
	# filename = "AB03/20220328-21_AE5105Standalone_PB_EKF_Test_AB03_Reverse_04.csv" #use this to tune gains, most unstable one
	# filename = "AB04/20220328-22_AE3220Standalone_PB_EKF_Test_AB04_Reverse_Tuning.csv"#pauses
	# filename = 'AB04/20220328-23_AE0413Standalone_PB_EKF_Test_AB04ReverseTuning_02.csv'
	filename = 'AB04/20220328-22_AE5753Standalone_PB_EKF_Test_AB04ForwardTuning_02.csv'


	data = np.loadtxt(filename, delimiter=',') 
	#set up HSDetector
	HS_analysis_window = 10
	HSDetector = HeelStrikeDetector(HS_analysis_window)

	z_measured_vec_hardware = data[:,29:35]
	heelAccForward_meas_fromDeltaVelocity_vec_hardware = data[:,70] #92
	heelAccSide_meas_fromDeltaVelocity_vec_hardware = data[:,71] #70
	heelAccUp_meas_fromDeltaVelocity_vec_hardware = data[:,72]#71

	heelAccForward_meas_fromDeltaVelocity_norm_vec_hardware = np.sqrt(heelAccForward_meas_fromDeltaVelocity_vec_hardware**2 +
															heelAccSide_meas_fromDeltaVelocity_vec_hardware**2 +
															(heelAccUp_meas_fromDeltaVelocity_vec_hardware)**2)

	timeSec_vec_hardware=data[:,0]

	HSs = []
	for i,x in enumerate(data[:]):
		timeSec=x[0]
		shankAngle_meas = x[22]
		footAngle_meas = x[23]

		z_measured_act = x[29:35]
		HSDetected_hardware = x[24]
		isOverriding_hardware = x[73]
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

		HSDetected_sim = HSDetector.detectHS(timeSec, footAngle_meas, footAngleVel_meas,  heelAccForward_meas_norm)

		HSs.append(HSDetected_sim)

	HSs = np.array(HSs)
	fig, axs = plt.subplots(3,1,sharex=True,figsize=(10,6))

	axs[0].plot(timeSec_vec_hardware, z_measured_vec_hardware[:,0], label=r"$foot angle, measured_{act}$")
	axs[0].plot(timeSec_vec_hardware, HSs*1e1, 'r',label=r"$HSDetected, Sim$")
	axs[0].legend()

	axs[1].plot(timeSec_vec_hardware, z_measured_vec_hardware[:,1], label=r"$foot angle vel, measured_{act}$")
	axs[1].plot(timeSec_vec_hardware, HSs*1e2, 'r',label=r"$HSDetected, Sim$")
	axs[1].legend()

	axs[2].plot(timeSec_vec_hardware, heelAccForward_meas_fromDeltaVelocity_norm_vec_hardware, label=r"$heel accel norm$")
	axs[2].plot(timeSec_vec_hardware, HSs*1e1, 'r',label=r"$HSDetected, Sim$")
	axs[2].legend()

	plt.show()

















	