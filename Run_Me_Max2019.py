
import smbus, time, math, random, threading, numpy, csv
import mpu6050_lib2
import BLNKM as LED
from LS7366R import LS7366R
from Functions_Max2019 import *
import traceback
import wiringpi as wp

NUM_TURNS_FOR_CATHERINE = -1; #Change to either 0 or -1 if having problems. This is a bug fix for the dial.



# Set up wiringpi pins to be GPIO
wp.wiringPiSetupGpio()

# Assign motor driver pins and set modes
#Comment out what all of these are
pwm_pin = 12
dir_pin = 16
enable_pin = 24
disable_pin = 23
wp.pinMode(enable_pin, 1)
wp.pinMode(disable_pin, 1)
wp.pinMode(dir_pin, 1)
wp.pinMode(pwm_pin, 2)


# Sensor and Motor initialization==============================================================================
#Inertial Measurement Unit
mpu2 = mpu6050_lib2.mpu6050(0X68)

#Light Emitting Diode
LED.initialize()
# MOTOR ENCODER
CSX = 0     # chip select channel (0 or 1)
CLK = 1000000   # SPI clock speed (0.5 MHz)
BTMD = 4        # bytemode resolution of counter (1-4)
encoder = LS7366R(CSX, CLK, BTMD)
scale = 12578.0 # encoder conversion scale (counts to mm)
scale_perc = 7169.46 # encoder conversion scale (counts to percentage throw)
# If unable to read in the motor encoder position, quit the program
try:
	current_position = encoder.readCounter() / scale
except:
	print "Encoder Counter is being lame. Fix it Human. Going to quit."
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

#ANKLE ENCODER
#wp.wiringPiI2CSetup(0x40)

t_start = time.time()

#==============================================================================================================

LED.fadetoRGB(102,0,204) # Purple

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
print 'Warm up'
current_angle = SingleAngle_I2C()
while time.time()-t_start<2.0:
	current_angle = SingleAngle_I2C()
	#print 'angle =', current_angle


# Run encoder homing routine=================================================================================================
homing = str(raw_input('Okay to proceed with homing routine to set slider position? (y/n/0) '))

if homing == 'n':
	print "Unable to determine position. Going to quit."
	encoder.close()
	time.sleep(1)

if homing == 'y':
	try:
		print "Starting homing routine. Press Ctrl + C to terminate homing program."
		time.sleep(1)
		keep_going = True

		while keep_going:
		#for x in range(20):
			last_position = encoder.readCounter()
			keep_going = sliderPosition(last_position-3*scale, 'low')
			time.sleep(0.1)
		
		wp.pwmWrite(pwm_pin, 0)

		encoder.clearCounter()
		current_position = int(round(encoder.readCounter() / scale))

		x_des = 25*scale #move to 25 mm
		sliderPosition(x_des)
		print "Homing successful. Slider position: %s mm" % current_position

	except KeyboardInterrupt:
		wp.pwmWrite(pwm_pin, 0)
		encoder.close()
		quit()

if homing == '0':
	print "Going to assume you are at the zero position then."
	encoder.clearCounter()
	current_position = int(round(encoder.readCounter() / scale))

#=============================================================================================================================

#This will determine what standing angle the user prefers
print 'Determining equilibrium angle... Have subject lift foot'
user_input = raw_input('Calibrate Equilibrium Position (y/n):')
if user_input == 'y':
	calib_offset = SingleAngle_I2C()
	current_angle = SingleAngle_I2C()-calib_offset
    # standing_angle = SingleAngle_I2C()-calib_offset

# #This calibration just creates an offset that sets the unloaded position of the device to the equilibrium 0 degree angle
# print 'Calibrating Encoder Offset'
# calib_offset = SingleAngle_I2C()
# current_angle = SingleAngle_I2C()-calib_offset
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

#Needed for ankle rotational velocity calculations
t_theta_prev = 0.0
theta_ankle_prev = 0.0
omega_ankle_prev = 0.0
omega_ankle_prev2 = 0.0

#Constants
R2D = 180.0/3.141592 #Converting from Radians to Degrees


while True:
	to_do = str(raw_input('What do you want to do? (stiffness, read, record, exp1, zero ankle, quit) '))
#======================================================================================================

	if to_do == 'quit':
		LED.fadetoRGB(102,0,204) # Purple #Checking Data/No Action
		print "Okay, quitting!"
		encoder.close()
		break

	if to_do == 'zero ankle':
		print 'Determining equilibrium angle... Have subject lift foot'
		user_input = raw_input('Ready to Calibrate Equilibrium Position? (y/n):')
		if user_input == 'y':
			calib_offset = SingleAngle_I2C()
			current_angle = SingleAngle_I2C()-calib_offset
			print "current angle: ", current_angle


#======================================================================================================

	if to_do == 'read':
		current_position = int(round(encoder.readCounter() / scale))
		print 'I think the current position is ', current_position
		print 'I think the current stiffness is', ConvertPositionToStiffness(current_position)
		current_angle = SingleAngle_I2C()-calib_offset
		print 'angle = ', current_angle

#======================================================================================================
	if to_do == 'record':
		#New file
		file_name = raw_input('Name the file: ') + '.csv'
		with open(file_name, "wb") as myfile:
			writer = csv.writer(myfile, delimiter=',')
		user_input = raw_input('for how many seconds?: ')
		experiment_duration = eval(user_input)  #Seconds
		sampling_frequency = 30 #Hz. rough!
		current_position = int(round(encoder.readCounter() / scale))
		current_stiffness = ConvertPositionToStiffness(current_position)
		AddDataPoint(file_name,['Stiffness', current_stiffness]) #Add the new stiffness to the csv file
		AddDataPoint(file_name,['Time', 'Angle'])
		start_time = time.time()
		t_elapsed = 0
		while t_elapsed < experiment_duration:
			time.sleep(1/sampling_frequency)
			current_angle = SingleAngle_I2C()-calib_offset
			t_elapsed = time.time()-start_time
			AddDataPoint(file_name,[t_elapsed, current_angle])

#===================================================================================
	if to_do == 'exp1':
		LED.fadetoRGB(255,0,5) # Fade to Red
		PK = int(raw_input('What is the nominal stiffness? 195 - 1300 [Nm/rad]): '))
		upperlim = 1.5 #What is the largest factor to be tested?
		a = numpy.log10(1/upperlim)
		print(a)
		b = numpy.log10(upperlim)
		n = 13
		Scale_Factor = numpy.logspace(a,b,n) #The scale factors
		Stiffness_Values = PK*Scale_Factor
		print(Stiffness_Values)

		while True:
			user_input = raw_input('Desired Stiffness Number (1 - 13): ')
			if RepresentsInt(user_input):
				user_input = eval(user_input)
				if 1<= user_input <= 13:
					desired_stiffness = Stiffness_Values[user_input-1]
					if desired_stiffness < 191:
						desired_stiffness = 191
						print('Unable to produce such a low stiffness, changing instead to 191')
					elif desired_stiffness > 1300:
						desired_stiffness = 1300
						print('Unable to produce such a high stiffness, changing instead to 1300')

					desired_position = round(ConvertStiffnessToPosition(desired_stiffness),1)
					x_des = desired_position*scale

					#JOG IT AWAY AND BACK
					if desired_position < 30:
						sliderPosition(x_des+5*scale)
					else:
						sliderPosition(x_des-5*scale)
					time.sleep(0.1)
					sliderPosition(x_des)
					time.sleep(0.2)
					current_position_mm = round(encoder.readCounter()/scale,2)
					print 'The new slider position is:', current_position_mm, ' of stiffness:', round(ConvertPositionToStiffness(current_position_mm),1)
				else:
					print 'Not within 1-13'
			else:
				break

		

#===================================================================================
	if to_do == 'stiffness':
		LED.fadetoRGB(255,0,5) # Fade to Red
		while True:
			user_input = raw_input('Desired Position (0 to 56 [mm] OR 191 to 1300 [Nm/rad]): ')
			if RepresentsInt(user_input):
				user_input = int(user_input)
				if 0 <= user_input <= 1300:
					if 0 <= user_input <= 56:
						desired_position_stiffnessMode = user_input
					elif 191<= user_input <= 1300:
						desired_position_stiffnessMode = round(ConvertStiffnessToPosition(user_input),1)
					else:
						print 'Not a known input'
				else:
					print 'Out of range. breaking out'
			else:
				break

			x_des_mm = desired_position_stiffnessMode
			x_des = desired_position_stiffnessMode*scale
			sliderPosition(x_des)
			time.sleep(0.2)

			#Display the new position
			current_position_mm = round(encoder.readCounter()/scale,2)
			print 'The new slider position is:', current_position_mm, ' of stiffness:', round(ConvertPositionToStiffness(current_position_mm),1)


#===================================================================================
	# if to_do == 'dial' or to_do == 'stiffness':
	if to_do == 'dial':
		LED.fadetoRGB(255,0,5) # Fade to Red
		# check_battery_or_quit() #Only compatible with boards 7-9
		current_position_mm = round(encoder.readCounter()/scale,2)
		last_angles = [0,0,0]
		in_swing = 0
		starting_position = current_position_mm
		arrivedFlag = False

		# We tryna record here?
		record_yn = str(raw_input('do you want to record? y/n:'))
		if record_yn == 'y':
			file_name = raw_input('Name the file: ') + '.csv'
			with open(file_name, "wb") as myfile:
				writer = csv.writer(myfile, delimiter=',')
			AddDataPoint(file_name,['Time', 'Slider Position (mm)', 'Stiffness (Nm/rad)', 'Joint Angle (deg)', 'Desired Stiffness', 'Motor Encoder (cts)','Motor Current',"inSwing"])
		elif record_yn != 'n':
			keepAsking = 1
			while keepAsking:
				record_yn = str(raw_input('Command not recognized. Do you want to record? y/n:'))
				if record_yn == 'y':
					keepAsking = 0
					file_name = raw_input('Name the file: ') + '.csv'
					with open(file_name, "wb") as myfile:
						writer = csv.writer(myfile, delimiter=',')
					AddDataPoint(file_name,['Time', 'Slider Position (mm)', 'Stiffness (Nm/rad)', 'Joint Angle (deg)', 'Desired Stiffness', 'Motor Encoder (cts)','Motor Current',"inSwing"])
				elif record_yn == 'n':
					keepAsking = 0


		print('starting_position: ', starting_position)
		t0 = time.time()

		# If in dial mode, initialize dial
		if to_do == 'dial':
			user_input = int(raw_input('Desired Starting Position (0 to 56 [mm] OR 180 to 1060 [Nm/rad]): '))
			if 0 <= user_input <= 1300:
				if 0 <= user_input <= 56:
					desired_position_mm = user_input
				elif 100<= user_input <= 1300:
					desired_position_mm = round(ConvertStiffnessToPosition(user_input),1)
				else:
					print 'Not a known input'
			else:
				print 'Out of range. Try again'

			starting_position = desired_position_mm


			first_dial_angle = SingleAngle_I2C_Dial() #We want to zero it if necessary
			last_dial_angle = SingleAngle_I2C_Dial()
			number_turns = NUM_TURNS_FOR_CATHERINE

			encoder_value, last_dial_angle, number_turns = ReadDialCont(last_dial_angle, number_turns, first_dial_angle)
			desired_position_mm = round(encoder_value/360*56, 1) + starting_position #This is the desired position in %

			if 0 <= desired_position_mm <= 56:
				desired_position_encoder = desired_position_mm*scale
			elif desired_position_mm > 56:
				desired_position_encoder = 56*scale
			elif desired_position_mm < 0:
				desired_position_encoder = 0*scale

			x_des_mm = desired_position_mm
			x_des = desired_position_encoder

		# If in stiffness mode, get dat stiffity stiffness
		elif to_do == 'stiffness':
			user_input = int(raw_input('Desired Position (0 to 56 [mm] OR 180 to 1060 [Nm/rad]): '))
			if 0 <= user_input <= 1300:
				if 0 <= user_input <= 56:
					desired_position_stiffnessMode = user_input
				elif 191<= user_input <= 1300:
					desired_position_stiffnessMode = round(ConvertStiffnessToPosition(user_input),1)
				else:
					print 'Not a known input'
			else:
				print 'Out of range. Try again'

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
		K_p = 0.010
		K_i = 0.8
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

		cur_angle = SingleAngle_I2C() - calib_offset
		angles = [cur_angle, cur_angle, cur_angle]
		CDflag = 0;


		#This is the primary control loop	
		while True:
			#Safety first
			if abs(motor_current_filtered) > 8:
				print 'Motor current too high... quitting.'
				wp.pwmWrite(pwm_pin, 0)
				break

			try:

				#We dialin?
				if to_do == 'dial':

					#Figure out where the dial wants to go
					encoder_value, last_dial_angle, number_turns = ReadDialCont(last_dial_angle, number_turns, first_dial_angle)
					#print encoder_value, first_dial_angle
					desired_position_mm = round(encoder_value/360*56, 1) + starting_position #This is the desired position in %
					if abs(desired_position_mm - current_position_mm) > 0.4: #This removes jittery movements caused by encoder noise. -Max 7/17/18
						if 0 <= desired_position_mm <= 56:
							edgeFlag = 0;
							desired_position_encoder = desired_position_mm*scale
						elif desired_position_mm > 56:
							desired_position_encoder = 56*scale
							desired_position_mm = 56;
							edgeFlag = 1;
						elif desired_position_mm < 0:
							desired_position_encoder = 0*scale
							desired_position_mm = 0;
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
						onFlag = 0

				#Are we in swing?
				last_position = current_position
				current_time = time.time()
				dt = current_time - last_time
				cur_angle = SingleAngle_I2C()-calib_offset
				x_act = encoder.readCounter()
				cur_stiff = ConvertPositionToStiffness(x_act/scale)
				in_swing, angles, CDflag = InSwingDetection(angles, cur_angle, CDflag, cur_stiff)

				#update error term
				dist_last = dist
				dist = int(x_des - x_act)

				if in_swing:

					#if to_do == 'dial':
						#print 'SWING!!'

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
					wp.pwmWrite(pwm_pin, pwm)
					current_position_mm = round(encoder.readCounter()/scale,2)


					if to_do == 'stiffness':
						if abs(pTerm) < 15: #or t_elapsed > time_limit:
							if arrivedFlag == False:
								print 'The new slider position is:', current_position_mm, ' of stiffness:', round(ConvertPositionToStiffness(current_position_mm),1)
								wp.pwmWrite(pwm_pin, 0)
								arrivedFlag = True


				else:
					wp.pwmWrite(pwm_pin, 0)
					start_time = time.time()
					#if to_do == 'dial':
						#print "STANCE!!"

				last_time = current_time
				last_position = current_position

				if to_do == 'dial':
					print 'The slider position is:', current_position_mm, ' of stiffness:', round(ConvertPositionToStiffness(current_position_mm),1)

				if record_yn == 'y':
					AddDataPoint(file_name,[time.time()-t0, current_position_mm, ConvertPositionToStiffness(current_position_mm), cur_angle, x_des_mm, encoder.readCounter(),motor_current_filtered, in_swing])

			except:
				if to_do == 'dial':
					print 'Interrupted...'
					break
				elif to_do == 'stiffness':
					arrivedFlag = False
					onFlag = 1

					start_time = time.time()
					e_D = 0
					iTerm = 0
					last_pwm = 100
					dist_last = dist

					user_input = int(raw_input('Desired Position (0 to 56 [mm] OR 180 to 1060 [Nm/rad]): '))
					if 0 <= user_input <= 1300:
						if 0 <= user_input <= 56:
							desired_position_stiffnessMode = user_input
						elif 100<= user_input <= 1300:
							desired_position_stiffnessMode = round(ConvertStiffnessToPosition(user_input),1)
							current_angle = SingleAngle_I2C()
					else:
						print 'Out of range. Try again'

					x_des_mm = desired_position_stiffnessMode
					x_des = desired_position_stiffnessMode * scale

wp.pwmWrite(pwm_pin, 0)

LED.fadetoRGB(102,0,204) # Purple #Checking Data/No Action
