import os, sys
from time import sleep, time, strftime, perf_counter
import traceback
import csv
import numpy as np
import scipy as sp
from scipy import linalg
import scipy.linalg
from SoftRealtimeLoop import SoftRealtimeLoop

from evalBezierFuncs_3P import *
from arctanMapFuncs import *


np.set_printoptions(precision=4)

thisdir = os.path.dirname(os.path.abspath(__file__))
print(thisdir)
sys.path.append(thisdir)

sys.path.append(thisdir + '/BestFitParams')


def clear():
	if os.name == 'nt':
		os.system('cls')
	else:
		os.system('clear')


accelScaleFactor = 8192 #LSB/g
gyroScaleFactor = 32.8 #LSB/ deg/s
degToCount = 45.5111 
countToDeg = 1/degToCount


accelNormCutoff = 1.15


#account for small biases in the gyro measurment, in the IMU frame in rad/s
gyroX_IMU_bias = 0.0194
gyroY_IMU_bias = -0.0094
gyroZ_IMU_bias = -0.0321


##define ankle angle zero when the ankle is perpendicular to shank
#this value is only good for the right leg

eye3 = np.eye(3)
eye6 = np.eye(6)
eye4= np.eye(4)


#--------also only good for the right leg--------
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


side = 'right'

sideMultiplier = 1
if (side == "left" or side == "l"):
	sideMultiplier = -1
elif (side == "right" or side == "r"):
	sideMultiplier = 1


def calibrateAngles_write(attitude_ekf,exo, writer, run_time = 10):

	# print(port)
	
	# devId =	fxOpen(port, baudRate,6)
	# print(devId)
	# fxStartStreaming(devId, frequency = 500, shouldLog = False)


	CORRECT_VICON = True


	startTime = time()
	prevTime = 0
	inProcedure = True
	i = 0
	dt = 1/180

	updateFHfreq = 20
	isUpdateTime = True
	isUpdateR = False

	phaseDelins = [0.1,0.5,0.65,1]

	shankAngleVec = []
	footAngleVec = []
	ankleAngleVec = []

	gravX_imuVec = []
	gravZ_imuVec = []

	for t in SoftRealtimeLoop(dt = 0.01):
		unskewTime0 = time()
		currentTime = time()
		timeSec = currentTime - startTime

		isUpdateTime = (timeSec % 1/updateFHfreq  < 1e-2)
		# print(isUpdateTime)
		dt = timeSec - prevTime
		# print(dt)
		exo.update()
		exoState = exo.act_pack
		# clearTerminal()

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
		gyroVec_corrected = gyroVec_corrected - np.array([gyroX_IMU_bias,gyroY_IMU_bias,gyroZ_IMU_bias])

		ankleAngle_buffer = exoState.ank_ang

		# step through
		attitude_ekf.step(i,dt,isUpdateTime)

		attitude_ekf.measure(i, gyroVec_corrected, accelVec_corrected,isUpdateTime, CORRECT_VICON)

		prevTime = timeSec
		shankAngle = attitude_ekf.get_useful_angles(0, sideMultiplier)

		shankAngleVec.append(shankAngle) # shank angle is in the nice reference frame relative to the vertical, extension is positive, flexion is negative
		ankleAngleVec.append(ankleAngle_buffer * countToDeg) # ankle angle is not in the nice reference frame relative to the shank

		if i%10 == 0:
			print('shankAngle: ' + str(shankAngle))

		if t >= run_time:
			break
			# fxClose(devId)
		i += 1

	
	shankAngleOffset = np.mean(shankAngleVec)
	ankleAngleOffset = np.mean(ankleAngleVec)

	print('ankleAngleOffset: ' + str(ankleAngleOffset))

	writer.writerow([shankAngleOffset,ankleAngleOffset])


	return (shankAngleOffset, ankleAngleOffset)


def calibrateAngles_read(filename):

	data = np.loadtxt(filename, delimiter=',')

	# file.close()
	print(data)
	shankAngleOffset = data[0]
	ankleAngleOffset = data[1]


	print('shankAngleOffset')
	print(str(shankAngleOffset))
	print('Ankle angle offset')
	print(str(ankleAngleOffset))		

	return (shankAngleOffset, ankleAngleOffset)


def main(exo, filename, attitude_ekf):

	prompt = int(input('Do you want to calibrate the exo (1) or read offsets from file (2) ?'))

	while not (prompt == 1 or prompt == 2):

		print('Not an option')
		prompt = input('Do you want to calibrate the exo (1) or read offsets from file (2) ?')

	if prompt == 1:
		with open(filename, "w", newline="\n") as fd_l:
			writer = csv.writer(fd_l)
			(shankAngleOffset, ankleAngleOffset) = calibrateAngles_write(attitude_ekf, exo, writer)

	elif prompt == 2:

		(shankAngleOffset, ankleAngleOffset) = calibrateAngles_read(filename)

	return (shankAngleOffset, ankleAngleOffset)
	





	


	
