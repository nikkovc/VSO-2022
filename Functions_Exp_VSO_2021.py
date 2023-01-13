# Variable Stiffness Orthosis Functions
#Nikko Van Crey, Tyler Clites, Marcos Cavallin. Sprite, Max Shepherd, Shengwen Liu, Delaney Miller
# Summer 2021

# Dependencies: time, numpy, csv, wiringpi, LS7366R.py

#make I2C bus go faster, overclock pi, hard code I2C library

# Import functions
import time
import numpy
import csv
import wiringpi as wp
from ADC import MCP3008
from LS7366R import LS7366R
import Adafruit_GPIO.SPI as SPI
import mpu6050_lib2

# Label pins
pwm_pin = 12
dir_pin = 16
enable_pin = 24
disable_pin = 23

# Set up Hall Effect sensor read
hall_pin_left = 14
hall_pin_right = 15


# Initialize motor encoder
CSX = 0		# chip select channel (0 or 1)
CLK = 1000000	# SPI clock speed (0.5 MHz)
BTMD = 4		# bytemode resolution of counter (1-4)
encoder = LS7366R(CSX, CLK, BTMD)
scale = 12578.0 # encoder conversion scale (counts to mm)
scale_perc = 7169 #encoder conversion scale (counts to percentage throw)
current_limit = 10 #Amps.
slider_max_perc = 98;
slider_min_mm = 1;
slider_max_mm = slider_max_perc*scale_perc/scale #stiffest slider position in mm allowed (first number is % of stroke)
plantar_switch = 5
dorsi_switch = -10

# Stole these functions from Max's code (max_functions_pi.py)

# Initialize ankle angle encoder
SPIchannel = 0 #SPI Channel (CE0)
#SPIchannel = 1 #SPI Channel (CE1)
SPIspeed = 500000 #Clock Speed in Hz
#wp.wiringPiSPISetupMode(SPIchannel, SPIspeed,1)
# Intialize encoders
h1 = wp.wiringPiI2CSetup(0x42)
h2 = wp.wiringPiI2CSetup(0x40)
my_data = numpy.zeros((50000, 2))

# Initialize ADC
adc = MCP3008(spi=SPI.SpiDev(0, 1))

#Initialize IMU
mpu2 = mpu6050_lib2.mpu6050(0X68)

#Clear error flag?
data1 = 0b01000000
foo1 = chr(data1)
data2 = 0b00000001
foo2 = chr(data2)
foobar = ''.join([foo1, foo2])
sendData = foobar
#recvData = wp.wiringPiSPIDataRW(SPIchannel, sendData)

#Import horizontal vector (csv) describring dorsiflexion stiffness as a function of slider position (in 1 mm increments)
slider2stiffness = numpy.genfromtxt('slider2stiffness.csv', delimiter=',') #This file is for a cam that max made.


# An offset term may or may not be necessary when using the ADC
def get_current():
       	voltage = (adc.read_adc(0)*5.0)/1024.0
        Uc = 5.0
        offset = .5*Uc
        #G = .08 #use with LEM GHS10SME 10A current sensor
        G = .2 # use with Allegro ACS724 10A current sensor (Boards 7-9, plus others)
        current = ((5.0/Uc)*voltage-offset)*(1.0/G)
        return current

def check_battery():
	# note that this function will give back a nonsense number for VSPA boards without the battery monitor voltage divider
        adc_voltage = (adc.read_adc(1)*5.0)/1024.0
        R6 = 71.5 * (10^3)
        R5 = 28.7 * (10^3)
        batt_voltage = ((R6+R5)/R5)*adc_voltage
        return batt_voltage

def check_battery_or_quit():
    # note that this function will give back a nonsense number for VSPA boards without the battery monitor voltage divider
	adc_voltage = (adc.read_adc(1)*5.0)/1024.0
	R6 = 71.5 * (10^3)
	R5 = 28.7 * (10^3)
	batt_voltage = ((R6+R5)/R5)*adc_voltage
	if batt_voltage < 9.0:
		print('Battery Voltage got too low so I quit!')
		quit()

#This is for an SPI version of the encoder AS5048A
def SingleAngle():	#reports back the current ankle angle
	foo1 = chr(0b11111111)
	foo2 = chr(0b11111111)
	sendData = ''.join([foo1, foo2])
	recvData = wp.wiringPiSPIDataRW(SPIchannel, sendData)
	#Need to do it a second time to get the READ
	foo1 = chr(0b11111111)
	foo2 = chr(0b11111111)
	sendData = ''.join([foo1, foo2])
	recvData = wp.wiringPiSPIDataRW(SPIchannel, sendData)
	datas = recvData[1]
	datachars = list(datas)
	data1 = bin(ord(datachars[0]))
	data2 = bin(ord(datachars[1]))
	rawbin1 = data1[2:]
	rawbin2 = data2[2:]
	bitstring1 = -len(rawbin1) % 8 * '0' + rawbin1
	bitstring1 = '00' + bitstring1[2:]
	bitstring2 = -len(rawbin2) % 8 * '0' + rawbin2
	byte1 = int(bitstring1,2)
	byte2 = int(bitstring2,2)
	value1 = byte1*256 + byte2
	encoder_offset = 258.75
	angle = value1*360.0/16384.0 - encoder_offset
	print('value1 = ', value1)
	return angle

#This is for an I2C version of the encoder AS5048B
def SingleAngle_I2C():	#reports back the current ankle angle
	#recvData = wp.wiringPiI2CReadReg16(h1,0xFF)
	#encoder_offset = 0
	#angle = 0.0219*recvData/4 - encoder_offset
	#return angle
	recvData1 = wp.wiringPiI2CReadReg8(h1,0xFF) # We used to have to add +1 to these numbers, but updating WiringPi fixed this issue
	recvData2 = wp.wiringPiI2CReadReg8(h1,0XFE) # We used to have to add +1 to these numbers, but updating WiringPi fixed this issue
	value1 = (recvData1*360.0/256.0)/64.0
	value2 = recvData2*64*360/16384.0
	angle = value1 + value2
	if(angle>300): #sometimes the encoder flips 360 degrees for reasons I don't understand (Nikko)
		angle = angle-360
	return angle


#This is for an I2C version of the encoder AS5048B
def SingleAngle_I2C_Dial():	#reports back the current ankle angle
	recvData1 = wp.wiringPiI2CReadReg8(h2,0xFF)
	recvData2 = wp.wiringPiI2CReadReg8(h2,0xFE)
	value1 = (recvData1*360.0/256.0)/64.0
	value2 = recvData2*64*360/16384.0
	angle = value1 + value2
	return angle


def ReadDialCont(last_angle, number_turns, first_angle): #This allows a dial with infinite rotation to be used.
	raw_angle = SingleAngle_I2C_Dial() - first_angle
	jump = last_angle - raw_angle
	if jump > 200:
		number_turns = number_turns + 1
		print('FLIP!')
	if jump < -200:
		number_turns = number_turns - 1
		print('FLIP!')
	true_angle = -number_turns*360 - raw_angle
	return(true_angle, raw_angle, number_turns)

def InSwingDetection(last_angles, cur_angle, CDflag, cur_stiff):
	last_angles.append(cur_angle)
	last_angles.pop(0)
	last_vel = last_angles[-2]-last_angles[-3]
	cur_vel = last_angles[-1]-last_angles[-2]
	torque_lim = 36 #Nm
	angle_lim = torque_lim/cur_stiff*180/3.14159

	if last_vel < 0 and cur_vel >= 0 and cur_angle < -3-(1050-cur_stiff)/(1050-181)*5:
		CDflag = 1
	elif last_vel > 0 and cur_vel <= 0 and cur_angle > 5+(1050-cur_stiff)/(1050-181)*3:
		CDflag = 0


	if all(i < angle_lim and i > -angle_lim for i in last_angles) and CDflag == 0:
		return True, last_angles, cur_vel, CDflag
	else:
		return False, last_angles, cur_vel, CDflag

def InSwingDetection_Spr22(last_angles, cur_angle, CDflag, cur_stiff, in_swing, in_stand, current_time,time_reset, last_vels):
	last_angles.append(cur_angle)
	last_angles.pop(0)

	last_vel = last_angles[-2]-last_angles[-3]
	last_vels.append(last_vel)
	last_vels.pop(0)
	cur_vel = numpy.mean(last_vels)
	
	wait_time = current_time

	if cur_angle < -12 or cur_angle > 12: #8, 18
		CDflag = 1
		in_swing = False
		in_stand = False
		return in_swing, in_stand, last_angles, CDflag, cur_vel
	else:
		if cur_angle > -3 and cur_angle < 3 and numpy.abs(cur_vel)<0.05:
			in_swing = False
			in_stand = True
			return in_swing, in_stand, last_angles, CDflag, cur_vel
		if cur_vel < -0.1 and cur_angle > -5: #-0.1, 5
			CDflag = 1
			in_swing = False
			return in_swing, in_stand, last_angles, CDflag, cur_vel
		if cur_vel > 1 and cur_angle > 0: #1, 5
			CDflag = 0
			in_swing = True
			in_stand = False
			return in_swing, in_stand, last_angles, CDflag, cur_vel
		else:
			return in_swing, in_stand, last_angles, CDflag, cur_vel


def InSwingDetection_Summer22(last_angles, cur_angle, gyro_pitch, CDflag, cur_stiff, in_swing, in_stand, current_time,time_reset, last_vels):
	last_angles.append(cur_angle)
	last_angles.pop(0)

	last_vel = last_angles[-2]-last_angles[-3]
	last_vels.append(last_vel)
	last_vels.pop(0)
	cur_vel = numpy.mean(last_vels)

	if cur_angle < -12 or cur_angle > 12: #8, 18
		CDflag = 1
		in_swing = False
		in_stand = False
		return in_swing, in_stand, last_angles, CDflag, cur_vel
	else:
		if cur_angle > -3 and cur_angle < 3 and numpy.abs(cur_vel)<0.05 and numpy.abs(gyro_pitch)<2:
			in_swing = False
			in_stand = True
			return in_swing, in_stand, last_angles, CDflag, cur_vel
		if gyro_pitch > 40 and cur_angle > 0: #Heel strike to footflat gyro pitch event
			CDflag = 1
			in_swing = False
			return in_swing, in_stand, last_angles, CDflag, cur_vel
		if cur_vel > 1 and cur_angle > 0: #1, 5
			CDflag = 0
			in_swing = True
			in_stand = False
			return in_swing, in_stand, last_angles, CDflag, cur_vel
		else:
			return in_swing, in_stand, last_angles, CDflag, cur_vel









def InSwingDetection_Fall22(last_angles, cur_angle, imu, CDflag, cur_stiff, in_swing, in_stand, current_time,time_reset, last_vels):
	last_angles.append(cur_angle)
	last_angles.pop(0)

	last_vel = last_angles[-2]-last_angles[-3]
	last_vels.append(last_vel)
	last_vels.pop(0)
	cur_vel = numpy.mean(last_vels)
	

	if (cur_angle > 7 or cur_angle < -6) and in_swing == True:
		print('STANCE!!')
		CDflag = 1
		in_swing = False
		in_stand = False
		#heel_strike = current_time
		return in_swing, in_stand, last_angles, CDflag, cur_vel
	else:
		if imu>-4 and in_swing == True:
			print('IMU STANCE!!')
			CDflag = 1
			in_swing = False
			in_stand = False
			#heel_strike = current_time
			return in_swing, in_stand, last_angles, CDflag, cur_vel
		#buffer_time = current_time-heel_strike
		if cur_vel > 1 and cur_angle < 7 and cur_angle > -6 and in_swing == False:
			print('SWING!!')
			CDflag = 0
			in_swing = True
			in_stand = False
			return in_swing, in_stand, last_angles, CDflag, cur_vel
		else:
			return in_swing, in_stand, last_angles, CDflag, cur_vel

def NewStepDetection(last_in_swing, in_swing):
	#Basically says it's a new step if you weren't just in swing, and are now in swing
	if last_in_swing == False and in_swing == True:
		return(True,in_swing) #(NewStep flag?, current value of in_swing)
	else:
		return(False,in_swing)

def ConvertPositionToStiffness(position): #Converts Position (in mm) to dorsiflexion stiffness for a linear cam
	i_float = position
	i = round(i_float)                        #Rounds to nearest integer, so to the nearest mm
	#print "Checking stiffness at position:", i
	if i < 0 or i > 56:
		print('Tried to access the stiffness at an impossible slider position')
		quit()
	i = int(i)
	y = slider2stiffness[i]
	if position < 1:
		m = (slider2stiffness[i+1]-slider2stiffness[i])
	else:
		m = (slider2stiffness[i]-slider2stiffness[i-1])
	y_float = y + m*(i_float-i)
	stiffness = y_float
	return stiffness

def ConvertStiffnessToPosition(stiffness):
	position_int = min(range(len(slider2stiffness)), key=lambda i: abs(slider2stiffness[i]-stiffness))
	stiffness_int = ConvertPositionToStiffness(position_int)
	if stiffness > stiffness_int:
		stiffness_int_next = ConvertPositionToStiffness(position_int+1)
		m = stiffness_int_next-stiffness_int
		position = (stiffness-stiffness_int)/m + position_int
	else:
		stiffness_int_previous = ConvertPositionToStiffness(position_int-1)
		m = stiffness_int-stiffness_int_previous
		position = (stiffness-stiffness_int_previous)/m + position_int-1
	return position


def AddDataPoint(file_name, data_to_add):
	with open(file_name, "a") as output:
	    writer = csv.writer(output, lineterminator='\n',quotechar='|')
	    writer.writerow(data_to_add)

def RepresentsInt(s):	#Just checks to see if it can be converted from string to int
    try: 
        int(s)
        return True
    except ValueError:
        return False

def knownPWM(pwm):
	pwm_pin = 12
	dir_pin = 16
	wp.digitalWrite(dir_pin, 0)
	wp.pwmWrite(pwm_pin, pwm)

	i = 0
	start_time = time.time()
	t_elapsed = 0
	my_data = numpy.zeros((50000, 9))
	while t_elapsed < 2:
		print('i = ', i)
		t_elapsed = time.time()-start_time
		motor_current = get_current()
		my_data[i,0] = t_elapsed
		my_data[i,1] = motor_current
		i = i+1

	my_data = my_data[:i]
	print("Done recording. Data stored in my_data.csv")
	column_names = ['time','motor_current']
	with open("my_data.csv", "w") as output:
		writer = csv.writer(output,delimiter=',')
		writer.writerow(column_names)
		writer = csv.writer(output, lineterminator='\n',quotechar='|')
		writer.writerows(my_data[1:i,0:2])
	wp.pwmWrite(pwm_pin, 0)

def lpfilter1(x,ypast):
	a1 = [1, -0.509525449494429]
	b1 = [0.245237275252786, 0.245237275252786]
	"send it last 1 filtered points and last 2 unfiltered points"
	y = -(a1[1]*ypast[0]) + b1[0]*x[0] + b1[1]*x[1];
	return(y)

def home():
	time.sleep(1)
	motor_current = 0
	motor_current_filtered = 0
	last_position = encoder.readCounter()
	sample_rate = 0.05
	keep_going = True
	wp.pwmWrite(pwm_pin, 40)  # (30) This sets the PWM frequency with which to jam it into the hard stop...
	time.sleep(0.1)

	while keep_going:
		last_position = encoder.readCounter()
		time.sleep(sample_rate)
		current_position = encoder.readCounter()
		motor_current_Last = motor_current
		motor_current_filtered_last = motor_current_filtered
		motor_current = get_current()
		motor_current_filtered = lpfilter1([motor_current_Last, motor_current], [motor_current_filtered_last])
		print("Motor current: %.2f A" % motor_current)
        #Safety first
		if abs(motor_current_filtered) > current_limit: #Tyler had this at 10
			print('Motor current too high... quitting.')
			wp.pwmWrite(pwm_pin, 0)
			break
		#Check if motor stopped
		error = current_position - last_position
		if -200 <= error <= 200:
			wp.pwmWrite(pwm_pin, 0)
			keep_going = False

def IMU_TRIAL():
	#gyro_data = mpu2.get_gyro_data()
	#gyro_pitch = gyro_data['x']
	acc_data = mpu2.get_accel_data()
	accel_y = acc_data['y'] #vertical
	#return [acc_pitch,gyro_data]
	#return gyro_pitch
	return accel_y

def readHall():
	if wp.digitalRead(hall_pin_left) == 1:
		#currenttime = time.time()
		#print 'Dorsi Cam (Left)'
		left = 1
	else:
		left = 0

	if wp.digitalRead(hall_pin_right) == 1:
		#currenttime = time.time()
		#print 'Plantar Cam (Right)'
		right = 1
	else:
		right = 0
	return left, right

def Detectswitch(switch, left_last, right_last, cur_angle):
	elec_error = 0
	mech_error = 0
	if wp.digitalRead(hall_pin_left) == 1:
		#currenttime = time.time()
		#print 'Dorsi Cam (Left)'
		left = 1
	else:
		left = 0

	if wp.digitalRead(hall_pin_right) == 1:
		#currenttime = time.time()
		#print 'Plantar Cam (Right)'
		right = 1
	else:
		right = 0
	if cur_angle>dorsi_switch and cur_angle<plantar_switch: #If ankle angle is within switching range
		#What do hall effect sensors detect at this moment

		#Detect Switch
		if left_last==1 and left ==0:
			print('Plantar Switch')
			#print "Dorsi Cam"
			switch =1

		if right_last==1 and right ==0:
			print('Dorsi Switch')
			#print "Plantar Cam"
			switch = 1


		#Error Detection
		if left == 1 and right== 1:
			print('Hall Effect Sensor Error 3')
			elec_error = 1

		#if switch==0 and elec_error ==0:
			#print "Didn't Switch"
	else:
		#Mechanical Failure
		if switch==0 and elec_error ==0:
			print('Mechanical Failure')
			mech_error = 1

	#Reset error to zero
	error = 0
	mech_error = 1

	return switch, left, right

def sliderPosition(x_des,faster):

	dist = x_des - encoder.readCounter()
	dist_initial = dist

	# Label pins
	pwm_pin = 12
	dir_pin = 16
	enable_pin = 24
	disable_pin = 23

	# Define gains for PID control
	K_p = 0.001 #0.010
	K_i = 0.01 #0.8
	K_d = 0.0001 #0.0001

	if faster:
		K_p = 0.01
		K_i = 2.0


	e_D = 0
	iTerm = 0
	last_pwm = 100
	
	start_time = time.time()
	last_time = start_time
	dist_last = dist

	# current monitoring
	R = 0.791  # in Ohms
	K_v = 1470 # in rpm/V
	V_supply = check_battery()  # in V
	P_v = 0

	# Initialize variables
	current_time = time.time()
	current_position = encoder.readCounter()
	dTerm = 0
	dTermFiltered = 0
	motor_current = 0
	motor_current_filtered = 0

	# print "Press CTRL + C to return to desired position menu."
	time_limit = 1 # Max time for position control loop to execute (safety)
	if faster:
		time_limit = 5

	PWM_Loop = True
	while PWM_Loop:
		try:
			if motor_current > current_limit: #8
				print('Motor current too high... quitting.')
				break
			last_position = current_position

			current_time = time.time()
			dt = current_time - last_time


			current_position = encoder.readCounter()
			v_rot = (current_position - last_position) / dt  # in counts/s
			v_rot = v_rot * 60 / 4096  # in rpm
			back_emf = v_rot / K_v  # calculate back emf

			pTerm = K_p * dist
			e_D = (dist-dist_last)/(dt)
			
			#Filter the derivative term
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

			
			#print "Motor current: %.2f A" % motor_current
			t_elapsed = time.time() - start_time


			# figure out direction
			if pwm_feedback < 0:
				wp.digitalWrite(dir_pin, 0)
			elif pwm_feedback > 0:
				wp.digitalWrite(dir_pin, 1)

			wp.pwmWrite(pwm_pin, pwm)
			
			current_position_mm = round(encoder.readCounter()/scale,2)
			# update error term
			x_act = encoder.readCounter()
			
			#print('current_position:', x_act)
			
			dist_last = dist
			dist = int(x_des - x_act)


			if abs(pTerm) < 15 and abs(iTerm) < 15 and abs(dTerm) < 15 or t_elapsed > time_limit:
				PWM_Loop = False
				wp.pwmWrite(pwm_pin, 0)

				if t_elapsed > time_limit:
					return False
				else:
					return True
				#print "Operation Complete - elapsed time:",t_elapsed


			last_time = current_time
			last_position = current_position



		except KeyboardInterrupt:
			print('Interrupted...')
			break

	wp.pwmWrite(pwm_pin, 0)

	
