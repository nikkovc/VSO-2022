
# Variable Stiffness Orthosis Main Script
#Nikko Van Crey, Marcos Cavallin, Tyler Clites, Sprite, Max Shepherd, Shengwen Liu, Delaney Miller
# Summer 2021

import smbus, time, math, random, threading, numpy, csv
import mpu6050_lib2
import BLNKM as LED
from LS7366R import LS7366R
from Functions_Exp_VSO_2021 import *
import traceback
import wiringpi as wp
import numpy as np

#import EKF 
import os, sys
thisdir = os.path.dirname(os.path.abspath(__file__))
print(thisdir)
sys.path.append(thisdir)
sys.path.append(thisdir + '/EKF')

from evalBezierFuncs_3P import *
from attitude_ekf import AttitudeEKF
from phase_ekf import PhaseEKF
import callibrate_angles_nls as ca
from gait_model import GaitModel
from filter_classes import FirstOrderLowPassLinearFilter, FirstOrderHighPassLinearFilter
from heel_strike_detector import HeelStrikeDetector



# Set up wiringpi pins to be GPIO
wp.wiringPiSetupGpio()

# Assign motor driver pins and set modes
pwm_pin = 12
dir_pin = 16
enable_pin = 24
disable_pin = 23
wp.pinMode(enable_pin, 1)
wp.pinMode(disable_pin, 1)
wp.pinMode(dir_pin, 1)
wp.pinMode(pwm_pin, 2)

#Set up scaling factors for IMU==========
accelScaleFactor = 8192 #LSB/g
gyroScaleFactor = 32.8 #LSB/ deg/s

#account for small biases in the gyro measurment, in the IMU frame in rad/s
gyroX_IMU_bias = 0.021065806164927196
gyroY_IMU_bias = 0.01037782833021424
gyroZ_IMU_bias = 0.007913779656035359

#set up rotation matrices for the IMU
theta_correction = 39.1090 * np.pi/180
#correct to non tilted axes
Rot_unskew = np.array(  [[np.cos(theta_correction), -np.sin(theta_correction),0],[np.sin(theta_correction), np.cos(theta_correction),0],[0, 0, 1]])
# correct to z up, x forward, y left
Rot1 = np.array( [[1, 0, 0],[0,np.cos(-np.pi/2), -np.sin(-np.pi/2)],[0,np.sin(-np.pi/2), np.cos(-np.pi/2)]] )
Rot2 = np.array( [[np.cos(-np.pi/2), 0 ,np.sin(-np.pi/2)],[0,1, 0],[-np.sin(-np.pi/2),0, np.cos(-np.pi/2)]]  )

Rot_correct = Rot_unskew

#set up EKF parameters
attitude_ekf_args = {'sigma_gyro':0.0023,
                            'sigma_accel': 0.0032*5*1/5,
                            'sigma_q_AE':1e2,
                            'Q_pos_scale':1e-10}

#measurement covariance matrix
sigma_shank = 7
sigma_shank_vel = 20

R_meas = np.diag([
	sigma_shank**2,
	sigma_shank_vel**2,
	])

#process noise
sigma_q_phase=0
sigma_q_phase_dot=6e-4

attitude_ekf=AttitudeEKF(**attitude_ekf_args)

gait_model = GaitModel('EKF/GaitModel/VSPA_gait_model.csv',phase_order=20)
phase_ekf_args = {'gait_model':gait_model,
		'torque_profile':None,
		'R_meas':R_meas,
		'sigma_q_phase':sigma_q_phase,
		'sigma_q_phase_dot':sigma_q_phase_dot,
		}

phase_ekf = PhaseEKF(**phase_ekf_args)

#set up variables that control how often the attitude EKF updates
updateFHfreq = 20
isUpdateTime = True

#set up shank angle offset (We'll eventually need to add the calibration procedure here)
shankAngleOffset = 0

#set up side multiplier that changes sign of estimated shank angles
side = 'right'
sideMultiplier = 1
if (side == "left" or side == "l"):
    sideMultiplier = -1
elif (side == "right" or side == "r"):
    sideMultiplier = 1

# Sensor and Motor initialization==============================================================================
#Inertial Measurement Unit
mpu2 = mpu6050_lib2.mpu6050(0X68)

#Light Emitting Diode
LED.initialize()

# If unable to read in the motor encoder position, quit the program
try:
	current_position = encoder.readCounter() / scale
except:
	print('Encoder Counter is being lame. Fix it Human. Going to quit.')
	encoder.close()
	time.sleep(1)
	quit()

# Reset motor driver and configure PWM pin
# This is from Delaney's code (2017) and I don't know if it's relevant on this board -Max Shepherd
wp.digitalWrite(enable_pin, 0)
wp.digitalWrite(disable_pin, 1)
wp.pwmWrite(pwm_pin, 0)
wp.digitalWrite(dir_pin, 0)
time.sleep(1)
wp.digitalWrite(enable_pin, 1)
wp.digitalWrite(disable_pin, 0)
wp.pwmSetMode(0)
wp.pwmSetRange(100)

#Hall Effect Sensors
wp.pinMode(hall_pin_left,0)
wp.pinMode(hall_pin_right,0)

#ANKLE ENCODER
#wp.wiringPiI2CSetup(0x40)

t_start = time.time()

#==============================================================================================================


# Check battery voltage and provide warnings
# battery_V = check_battery()
# print "Assuming Board 7-9 with 3S 12V battery..."
# print "IMPORTANT: Do NOT use when battery voltage is less than 9.0 V"
# print "Battery Voltage: %.2f V" % battery_V
# if battery_V < 9.0:
#   print "BATTERY VOLTAGE TOO LOW!  Charge your battery."
#   LED.fadetoRGB(255,0,0)
#   quit()
print('Assuming board does not have battery voltage detection. Monitor your battery voltage.')


#Calibrate the Device

#Ankle Encoder needs time to warm up before it will work properly
print('Warm up ankle encoder. Do not move dial')
while time.time()-t_start<2.0:
	warmup_angle_ankle = SingleAngle_I2C()
	calib_offset_dial = SingleAngle_I2C_Dial()

unloaded_angle = str(input('Take VSO off person and put on tabletop. Time to home unloaded equilibrium (y/n).'))
if unloaded_angle == 'n':
	print('You need to home unloaded equilibrium or all data will be shifted by an unknown angle! Quitting!')
	quit()

if unloaded_angle == 'y':
	print('Equilibrium Calibrated!.')
	calib_offset_unloaded = SingleAngle_I2C()
	


#Run spring stroke calibration (Only need to run this once per lead screw assembly, if you reassembly run it again)
calibration = str(input('Calibrate stroke of lead screw? Do this for each new device or reassembly! (y/n/0) '))

if calibration == 'n' or calibration == '0':
	print('Assuming stroke is calibrated, make sure to home.')

if calibration == 'y':
    try:
        print('Starting calibration routine. Press Ctrl + C to terminate homing program.')
        home()
        encoder.clearCounter()
        print('Homing successful. Slider position: %s mm' % current_position)
        time.sleep(1)
        print('Now moving to 100% slider position')
        wp.digitalWrite(dir_pin, 1)
        home()
        print('At 100% slider position')
        encoder_number = encoder.readCounter()
        scale_perc_updated = encoder_number/ 100
        time.sleep(1)
        print('Homing Again')
        wp.digitalWrite(dir_pin, 0)
        home()
        encoder.clearCounter()
        print('The updated scale_perc is: %.2f , update this number in Functions.py' % scale_perc_updated)

    except KeyboardInterrupt:
        wp.pwmWrite(pwm_pin, 0)
        quit()


# Run encoder homing routine=================================================================================================
homing = str(input('Zero the spring support? (y/n/0) '))

if homing == 'n':
	print('Unable to determine position. Going to quit.')
	encoder.close()
	time.sleep(1)
	quit()

if homing == 'y':
    try:
        print('Starting homing routine. Press Ctrl + C to terminate homing program.')
        home()
        encoder.clearCounter()
        current_position = int(round(encoder.readCounter() / scale))
        print('Homing successful. Slider position: %s mm' % current_position)

    except KeyboardInterrupt:
        wp.pwmWrite(pwm_pin, 0)
        encoder.close()
        quit()

if homing == '0':
    print('Assuming you are at the zero position.')
    encoder.clearCounter()
    current_position = int(round(encoder.readCounter() / scale))

#=============================================================================================================================

#This will determine what standing angle the user prefers
user_input = input('After putting VSO on subject home equilibrium (y/n):')

if user_input == 'n':
	print('You need to do this step or the ankle mechanics the subject experiences will be shifted by arbitrary angle! Quitting!')
	#quit()

if user_input == 'y':
	print('Adjust shin attachment until angle is close to zero. Program will end automatically if adjustment is close enough.')
	i=0
	standing_angle = 1000; #random angle to instantiate variable and enter while loop
	while abs(standing_angle)>0.25:
		standing_angle = SingleAngle_I2C()-calib_offset_unloaded
		i = i+1
		if (i/20-int(i/20))==0:
			print('Ankle is :%s deg from equilibrium' % standing_angle)
		#Make sure device doesn't home while shin adjustment is dynamic
		if(abs(standing_angle)<0.25):
			time.sleep(6) #Don't change this time or you might get false homing
			standing_angle = SingleAngle_I2C()-calib_offset_unloaded
			print('Ankle is :%s deg from equilibrium' % standing_angle)


# ==============================================================================================================================

#Sampling Considerations (Entire Code Samples at 100Hz)
#Accelerometer: (250Hz Sampling) costs 4 miliseconds computation
#Gyro: (250Hz Sampling) costs 4 miliseconds computation
#Each file write cost 0.25 miliseconds
#Ankle Encoder (1000Hz Sampling) 1 milisecond computation
#Motor Encoder (2000Hz Sampling) 0.5 milisecond computation

#=========================================================================================================

#Constants
equilibrium = 0 #Unloaded VSPA Ankle Angle

#Needed for shank pitch velocity or vertical acceleartion calculations
imu_window = [0,0,0]

#Constants
R2D = 180.0/3.141592 #Converting from Radians to Degrees


while True:
	to_do = str(input('What do you want to do? (quit, home slider, read hall, read slider position, read ankle angle, read dial, read imu, record, dial, stiffness, gait detection) '))
#======================================================================================================

	if to_do == 'quit':
		print('Okay, quitting!')
		encoder.close()
		break

#======================================================================================================
	if to_do == 'home slider':
		try:
			print('Starting homing routine. Press Ctrl + C to terminate homing program.')
			home()
			encoder.clearCounter()
			current_position = int(round(encoder.readCounter() / scale))
			print ('Homing successful. Slider position: %s mm' % current_position)

		except KeyboardInterrupt:
			wp.pwmWrite(pwm_pin, 0)
			encoder.close()
			quit()


#======================================================================================================
	if to_do == 'read slider position':
		LED.fadetoRGB(102,0,204) # Purple #Checking Data/No Action
		current_position = int(round(encoder.readCounter() / scale))
		print('The current slider position is %s mm' % current_position)
		current_position_perc = int(round(encoder.readCounter() / scale_perc))
		print('The current slider position is %s %%' % current_position_perc)
		battery_V = check_battery()
		if battery_V>0.0:
			if battery_V < 9.0:
				print('BATTERY VOLTAGE TOO LOW!  Charge your battery.', battery_V)
				LED.fadetoRGB(255,0,0)
				quit()
		else:
			print('Inaccurate Battery Monitoring')

#======================================================================================================
	if to_do == 'read ankle angle':
		for i in range(50):
			LED.fadetoRGB(102,0,204) # Purple #Checking Data/No Action
			current_angle = SingleAngle_I2C()-calib_offset_unloaded
			print('angle = ', current_angle)
			# print 'stiffness =', current_position
			time.sleep(0.1)


#======================================================================================================
	if to_do == 'read dial':
		address_dial = 0x42
		for i in range(50):
			# LED.fadetoRGB(102,0,204) # Purple #Checking Data/No Action
			dial_angle = SingleAngle_I2C_Dial()-calib_offset_dial
			print('dial = ', dial_angle)
			time.sleep(0.1)
#======================================================================================================			
	if to_do == 'read imu':
		for i in range(100000):
			#imu_reading = IMU_TRIAL()
			#print 'xacc= ', imu_reading
			imu_window.append(IMU_TRIAL())
			imu_window.pop(0)
			imu = numpy.mean(imu_window)
			#print('Shank pitch velocity of %s unit unknown' % gyro_pitch)
			print('IMU vertical acceleration of %s unit unknown' % imu)
			#time.sleep(0.1)

#=========================================================================================================
	if to_do == 'read hall':
		switch = 0
		for i in range(100):
			left, right = readHall()
			cur_angle = SingleAngle_I2C() - calib_offset_unloaded
			#switch, left, right = Detectswitch(switch, left, right, cur_angle)
			#print(switch)
			print('left Hall', left)
			print('right Hall', right)
			#print 'Switched at ', switch
			# print 'stiffness =', current_position
			time.sleep(0.1)

#=========================================================================================================            
	if to_do == 'set_zero':
		while True:
			user_input = input('What should I think the current position is?: ')
			if RepresentsInt(user_input):
				current_position = eval(user_input)
				desired_position = current_position
				print('okay, I think the current position is ', current_position)
				break
			elif user_input == 'back':
				break
			else:
				print('failed... try an integer! or type back')

#======================================================================================================
	if to_do == 'record':
		#New file
		file_name = input('Name the file: ') + '.csv'
		with open(file_name, "wb") as myfile:
			writer = csv.writer(myfile, delimiter=',')
		user_input = input('for how many seconds?: ')
		experiment_duration = eval(user_input)  #Seconds
		sampling_frequency = 100 #Hz. rough!
		AddDataPoint(file_name,['Stiffness', current_position]) #Add the new stiffness to the csv file
		AddDataPoint(file_name,['Time', 'Angle'])
		start_time = time.time()
		t_elapsed = 0
		while t_elapsed < experiment_duration:
			time.sleep(1/sampling_frequency)
			current_angle = SingleAngle_I2C()-calib_offset_unloaded
			t_elapsed = time.time()-start_time
			AddDataPoint(file_name,[t_elapsed, current_angle])

#===================================================================================
	if to_do == 'dial' or to_do == 'stiffness' or to_do =='gait detection':
		# check_battery_or_quit() #Only compatible with boards 7-9
		current_position_mm = round(encoder.readCounter()/scale,2)
		current_position_perc = round(encoder.readCounter() / scale_perc,2)
		last_angles = [0,0,0]
		last_vels = [0,0,0]
		imu_window = [0,0,0]
		in_swing = 1
		in_stand = 0
		starting_position = current_position_mm
		arrivedFlag = False
		heel_strike = 0

		# We tryna record here?
		record_yn = str(input('do you want to record? y/n:'))
		if record_yn == 'y':
			file_name = input('Name the file: ') + '.csv'
			with open(file_name, "wb") as myfile:
				writer = csv.writer(myfile, delimiter=',')
			AddDataPoint(file_name,['Time', 'Slider Position (mm)', 'Slider Position (%)', 'Stiffness (Nm/rad)', 'Joint Angle (deg)', 'Joint Angle Velocity (deg/s)', 'Vertical Acceleration', 'Desired Position [mm]', 'Motor Encoder (cts)','Motor Current',"inSwing"])
		elif record_yn != 'n':
			keepAsking = 1
			while keepAsking:
				record_yn = str(input('Command not recognized. Do you want to record? y/n:'))
				if record_yn == 'y':
					keepAsking = 0
					file_name = input('Name the file: ') + '.csv'
					with open(file_name, "wb") as myfile:
						writer = csv.writer(myfile, delimiter=',')
					AddDataPoint(file_name,['Time', 'Slider Position (mm)', 'Slider Position (%)', 'Stiffness (Nm/rad)', 'Joint Angle (deg)', 'Joint Angle Velocity (deg/s)','Vertical Acceleration','Desired Position [mm]', 'Motor Encoder (cts)','Motor Current',"inSwing"])
				elif record_yn == 'n':
					keepAsking = 0


		t0 = time.time()

		if to_do == 'gait detection':
			#Initialize variables so that pwm code doesn't throw an error
			x_des = 0
			x_des_mm = 0

		# If in dial mode, initialize dial
		if to_do == 'dial':
			user_input = int(input('Desired Starting Position (0 to 100 [%] OR 180 to 1060 [Nm/rad]): '))
			if 0 <= user_input <= 1300:
				if 0 <= user_input <= slider_max_perc:
					desired_position_mm = user_input*scale_perc/scale
				elif 100<= user_input <= 1300:
					desired_position_mm = round(ConvertStiffnessToPosition(user_input),1)
				else:
					print('Not a known input')
			else:
				print('Out of range. Try again')

			starting_position = desired_position_mm


			first_dial_angle = SingleAngle_I2C_Dial() #We want to zero it if necessary
			last_dial_angle = SingleAngle_I2C_Dial() - first_dial_angle
			number_turns = 0


			encoder_value, last_dial_angle, number_turns = ReadDialCont(last_dial_angle, number_turns, first_dial_angle)
			desired_position_mm = round(encoder_value/360*slider_max_mm, 1) + starting_position #This is the desired position in %

			if 0 <= desired_position_mm <= slider_max_mm:
				desired_position_encoder = desired_position_mm*scale
			elif desired_position_mm > slider_max_mm:
				desired_position_encoder = slider_max_mm*scale
			elif desired_position_mm < slider_min_mm:
				desired_position_encoder = slider_min_mm*scale

			x_des_mm = desired_position_mm
			x_des = desired_position_encoder


		# If in stiffness mode, get dat stiffity stiffness
		elif to_do == 'stiffness':
			user_input = int(input('Desired Position (0 to 100 [%] OR 180 to 1060 [Nm/rad]): '))
			if 0 <= user_input <= 1300:
				if 0 <= user_input <= slider_max_perc:
					desired_position_stiffnessMode = user_input*scale_perc/scale
				elif 200<= user_input <= 1300:
					desired_position_stiffnessMode = round(ConvertStiffnessToPosition(user_input*scale_perc/scale),1)
				else:
					print('Not a known input')
			else:
				print('Out of range. Try again')

			x_des_mm = desired_position_stiffnessMode
			x_des = desired_position_stiffnessMode*scale

		dist = x_des - encoder.readCounter()
		dist_initial = dist

		# Label pins
		pwm_pin = 12
		dir_pin = 16
		enable_pin = 24
		disable_pin = 23

		# Define gains for PID control
		K_p = 0.015 #0.010
		#K_i = 0.8
		K_i = 2
		K_d = 0.0001

		e_D = 0
		iTerm = 0
		last_pwm = 100
		onFlag = 1

		# Initialize timer
		start_time = time.time()
		last_time = start_time
		dist_last = dist

		# current monitoring
		R = 0.791  # in Ohms
		K_v = 1470 # in rpm/V
		V_supply = check_battery()  # in V
		P_v = 0
		# b = [0.0]*5; bpast = [0.0]*5

		# Initialize variables
		current_time = time.time()
		current_position = encoder.readCounter()
		dTerm = 0
		dTermFiltered = 0
		motor_current = 0
		motor_current_filtered = 0
		edgeFlag = 0
		time_reset = 0


		cur_angle = SingleAngle_I2C() - calib_offset_unloaded
		angles = [cur_angle, cur_angle, cur_angle]
		CDflag = 0
		gait_flag = 1
		time_limit = 3
		switch = 0
		left, right = readHall()


		try:
		#if(True): #Used to debug because try operation won't show errors

			#This is the primary control loop	
			while True:
				#Safety first
				#if to_do == 'stiffness' or to_do == 'dial':
				if abs(motor_current_filtered) > current_limit: #Tyler had this at 10
					print('Motor current too high... quitting.')
					wp.pwmWrite(pwm_pin, 0)
					break


				#We dialin?
				if to_do == 'dial':

					#Figure out where the dial wants to go
					encoder_value, last_dial_angle, number_turns = ReadDialCont(last_dial_angle, number_turns, first_dial_angle)
					#print encoder_value, first_dial_angle
					desired_position_mm = round(encoder_value/360*slider_max_mm, 1) + starting_position #This is the desired position in %
					if abs(desired_position_mm - current_position_mm) > 0.4: #This removes jittery movements caused by encoder noise. -Max 7/17/18
						if 0 <= desired_position_mm <= slider_max_mm:
							edgeFlag = 0;
							desired_position_encoder = desired_position_mm*scale
						elif desired_position_mm > slider_max_mm:
							desired_position_encoder = slider_max_mm*scale
							desired_position_mm = slider_max_mm;
							edgeFlag = 1;
						elif desired_position_mm < slider_min_mm:
							desired_position_encoder = slider_min_mm*scale
							desired_position_mm = slider_min_mm;
							edgeFlag = 1;

						x_des_mm = desired_position_mm
						x_des = desired_position_encoder
						onFlag = 1
					else:
						e_D = 0
						iTerm = 0
						last_pwm = 100
						dist_last = dist
						onFlag = 0

				#We commanding stiffness from command line?
				if to_do == 'stiffness':
					if arrivedFlag:
						#reset everything
						#arrivedFlag = False
						#print('Got Here') #testing
						onFlag = 0
						break #
						#raise Exception('Made it!') #testing

				#Are we in swing? Sample sensors and find out
				imu_window.append(IMU_TRIAL())
				#imu_window.append(0)
				imu_window.pop(0)
				imu = numpy.mean(imu_window)

				#===============EKF CODE BLOCK===============
				#read from IMU for EKF
				accelX = exoState.accelx/accelScaleFactor # in units of g
				accelY = exoState.accely/accelScaleFactor
				accelZ = exoState.accelz/accelScaleFactor

				gyroX = exoState.gyrox/gyroScaleFactor * np.pi/180 #in units of rad/s
				gyroY = exoState.gyroy/gyroScaleFactor * np.pi/180
				gyroZ = exoState.gyroz/gyroScaleFactor * np.pi/180

				accelVec = np.array([accelX,accelY,accelZ])
				accelVec_corrected = Rot_correct @ (accelVec)
				gyroVec = np.array([gyroX,gyroY,gyroZ])
				gyroVec_corrected = Rot_correct @ (gyroVec)
				gyroVec_corrected = gyroVec_corrected - Rot_correct @ np.array([gyroX_IMU_bias,gyroY_IMU_bias,gyroZ_IMU_bias])


				#step through EKFs
				isUpdateTime = (timeSec % 1/updateFHfreq  < 1e-2)
				attitude_ekf.step(i, dt, isUpdateTime)
				phase_ekf.step(i,dt)

				#attitude EKF measurement step
				attitude_ekf.measure(i, gyroVec_corrected, accelVec_corrected, isUpdateTime, CORRECT_VICON=True)
				
				shankAngle_meas = attitude_ekf.get_useful_angles(sideMultiplier)
				shankAngle_meas = shankAngle_meas - shankAngleOffset

				shankAngleVel_meas = sideMultiplier * -1 * gyroVec_corrected[1] * 180/np.pi
				z_measured = np.array([shankAngle_meas, shankAngleVel_meas])

				#update phase EKF
				phase_ekf.update(i, dt, z_measured)
				x_state_update = phase_ekf.get_x_state_update()

				#===============END EKF CODE BLOCK===============


				last_position = current_position
				current_time = time.time()
				dt = current_time - last_time
				cur_angle = SingleAngle_I2C()-calib_offset_unloaded
				x_act = encoder.readCounter()
				cur_stiff = ConvertPositionToStiffness(x_act/scale)

				#Detect Switching During Stance
				#if switch == 0:
					#switch, left, right = Detectswitch(switch, left, right, cur_angle)

				#Check if we are in swing
				#in_swing, angles, cur_vel, CDflag = InSwingDetection(angles, cur_angle, CDflag, cur_stiff)
				in_swing, in_stand, angles, CDflag, cur_vel = InSwingDetection_Fall22(angles, cur_angle, imu, CDflag, cur_stiff, in_swing, in_stand, current_time, time_reset, last_vels)
				
				#PRINT VALUES DURING GAIT DETECTION
				# print(cur_vel)
				#print(cur_angle)
				#in_swing  = 1 #testing
				#i = i+1 #testing
				#if (i/200-int(i/200))==0: #testing
					#in_swing = 1


				#update error term
				dist_last = dist
				dist = int(x_des - x_act)
				#print('dist', dist) #testing
				#print('x_des', x_des) #testing
				#print('x_act', x_act) #testing

				if in_swing or in_stand:
					switch = 0
					if in_swing:
						LED.fadetoRGB(5,0,255) # Fade to Blue
					if in_stand:
						LED.fadetoRGB(0,255,0) # Green
					#print cur_angle
					if(gait_flag):
						#print('SWING!!')
						gait_flag = 0
					
					current_position = encoder.readCounter()
					v_rot = (current_position - last_position) / dt  # in counts/s
					v_rot = v_rot * 60 / 4096  # in rpm
					back_emf = v_rot / K_v  # calculate back emf

					pTerm = K_p * dist
					e_D = (dist-dist_last)/(dt)

					dTermLast = dTerm
					dTermFilteredLast = dTermFiltered
					dTerm = e_D * K_d
					dTermFiltered = lpfilter1([dTermLast, dTerm], [dTermFilteredLast])


					# only integrate when not saturated (prevent windup)
					if -70 < last_pwm < 70:
						iTerm = iTerm + (K_i * dist * dt)


					#Set PWM frequency
					pwm_feedback = int(pTerm + iTerm + dTermFiltered)
					last_pwm = pwm_feedback

					# limit pwm feedback to between 7% and 70% duty cycle
					#print 'pwm = ', pwm_feedback
					pwm_upper_limit = 70


					# pwm_lower_limit = 5	
					if pwm_feedback > pwm_upper_limit:
						pwm_feedback = pwm_upper_limit
					elif pwm_feedback < -pwm_upper_limit:
						pwm_feedback = -pwm_upper_limit
					elif -7 <= pwm_feedback < -2:
						pwm_feedback = -7
					elif -2 <= pwm_feedback < 2:
						pwm_feedback = 0
					elif 2 <= pwm_feedback < 7:
						pwm_feedback = 7


					pwm = abs(pwm_feedback)
					motor_current_Last = motor_current
					motor_current_filtered_last = motor_current_filtered
					motor_current = get_current()
					motor_current_filtered = lpfilter1([motor_current_Last, motor_current], [motor_current_filtered_last])

					if edgeFlag == 1 and abs(dist)<4000:
						pwm = 0

					t_elapsed = time.time() - start_time

					#figure out direction
					if pwm_feedback < 0:
						wp.digitalWrite(dir_pin, 0)
					elif pwm_feedback > 0:
						wp.digitalWrite(dir_pin, 1)
					
					pwm = pwm * onFlag
					if to_do == 'gait detection':
						pwm = 0; 
					wp.pwmWrite(pwm_pin, pwm) #Power to Motor
					current_position_mm = round(encoder.readCounter()/scale,2)
					current_position_perc = round(encoder.readCounter() / scale_perc,2)
					#print('pTerm', abs(pTerm))#testing
					if to_do == 'stiffness':
						if abs(pTerm) < 15: #or t_elapsed > time_limit:
							if arrivedFlag == False:
								print('The new slider position is:', current_position_mm, 'mm ', current_position_perc,'% of stiffness:', round(ConvertPositionToStiffness(current_position_mm),1))
								wp.pwmWrite(pwm_pin, 0)
								arrivedFlag = True


				else:
					wp.pwmWrite(pwm_pin, 0)
					start_time = time.time()
					# print calib_offset_unloaded, cur_angle 
					LED.fadetoRGB(255,0,5) # Fade to Red
					if(gait_flag==0):
						#print('STANCE!!')
						gait_flag = 1


				last_time = current_time
				last_position = current_position

				if to_do == 'dial':
					print('The slider position is:', current_position_mm, 'mm ', current_position_perc,'% of stiffness:', round(ConvertPositionToStiffness(current_position_mm),1))

				if record_yn == 'y':
					AddDataPoint(file_name,[time.time()-t0, current_position_mm, current_position_perc, ConvertPositionToStiffness(current_position_mm), cur_angle, cur_vel, imu, x_des_mm, encoder.readCounter(),motor_current_filtered, in_swing])

		except:
		#else: #Used to debug because try operation won't show errors
			if to_do == 'dial':
				wp.pwmWrite(pwm_pin, 0)
				print('Interrupted...')
				break
			elif to_do == 'stiffness':
				wp.pwmWrite(pwm_pin, 0)
				arrivedFlag = False
				onFlag = 1

				start_time = time.time()
				e_D = 0
				iTerm = 0
				last_pwm = 100
				dist_last = dist

				user_input = int(input('Desired Position (0 to 100 [%] OR 180 to 1060 [Nm/rad]): '))
				if 0 <= user_input <= 1300:
					if 0 <= user_input <= slider_max_perc:
						desired_position_stiffnessMode = user_input*scale_perc/scale
					elif 200<= user_input <= 1300:
						desired_position_stiffnessMode = round(ConvertStiffnessToPosition(user_input*scale_perc/scale),1)
						current_angle = SingleAngle_I2C()
				else:
					print('Out of range. Try again')

				x_des_mm = desired_position_stiffnessMode
				x_des = desired_position_stiffnessMode*scale
				break

wp.pwmWrite(pwm_pin, 0)