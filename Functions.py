# Variable Stiffness Ankle Functions
# Nikko Van Crey, Marcos Cavallin
#Uses a lot of code written by Delaney Miller
# Summer 2017

# Description: Helper functions to run homing routine and move ankle to desired position

# Dependencies: time, numpy, csv, wiringpi, LS7366R.py

# Import functions
import time
import numpy
import csv
import wiringpi as wp
from ADC import MCP3008
from LS7366R import LS7366R
import Adafruit_GPIO.SPI as SPI

# Label pins
pwm_pin = 12
dir_pin = 16
enable_pin = 24
disable_pin = 23

# Initialize motor encoder
CSX = 0			# chip select channel (0 or 1)
CLK = 1000000	# SPI clock speed (0.5 MHz)
BTMD = 4		# bytemode resolution of counter (1-4)
encoder = LS7366R(CSX, CLK, BTMD)
scale = 12578.0 # encoder conversion scale (counts to mm)
time_limit = 6.0 # Max time for position control loop to execute (safety)

# Stole these functions from Max's code (max_functions_pi.py)

# Initialize ankle angle encoder
SPIchannel = 0 #SPI Channel (CE0)
#SPIchannel = 1 #SPI Channel (CE1)
SPIspeed = 500000 #Clock Speed in Hz
#wp.wiringPiSPISetupMode(SPIchannel, SPIspeed,1)
# Intialize encoders
h1 = wp.wiringPiI2CSetup(0x40)
h2 = wp.wiringPiI2CSetup(0x42)
my_data = numpy.zeros((50000, 2))

# Initialize ADC
adc = MCP3008(spi=SPI.SpiDev(0, 1))

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
	print 'value1 = ', value1
	return angle

#This is for an I2C version of the encoder AS5048B
def SingleAngle_I2C():	#reports back the current ankle angle
	recvData = wp.wiringPiI2CReadReg16(h1,0xFE)
	encoder_offset = 0
	angle = 0.0219*recvData/4 - encoder_offset
	return angle

#This is for an I2C version of the encoder AS5048B
def SingleAngle_I2C_Dial():	#reports back the current ankle angle
	recvData = wp.wiringPiI2CReadReg16(h2,0xFE)
	encoder_offset = 0
	angle = 0.0219*recvData/4 - encoder_offset
	return angle

def ReadDialCont(last_angle, number_turns, first_angle): #This allows a dial with infinite rotation to be used.
	raw_angle = SingleAngle_I2C_Dial() - first_angle
	jump = last_angle - raw_angle
	if jump > 270:
		number_turns = number_turns + 1
	if jump < -270:
		number_turns = number_turns - 1
	true_angle = number_turns*360 + raw_angle
	return(true_angle, raw_angle, number_turns)

def InSwingDetection(last_angles, current_angle):
	last_angles.append(current_angle)
	last_angles.pop(0)
	if all(i < 1.0 and i > -1.0 for i in last_angles):
		return True, last_angles
	else:
		return False, last_angles

def NewStepDetection(last_in_swing, in_swing):
	#Basically says it's a new step if you weren't just in swing, and are now in swing
	if last_in_swing == False and in_swing == True:
		return(True,in_swing) #(NewStep flag?, current value of in_swing)
	else:
		return(False,in_swing)

def ConvertPositionToStiffness(position): #Converts Position (in mm) to dorsiflexion stiffness for a linear cam
	i_float = position
	i = round(i_float)                          #Rounds to nearest integer, so to the nearest mm
	#print "Checking stiffness at position:", i
	if i < 0 or i > 56:
		print('Tried to access the stiffness at an impossible slider position')
		quit()
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


def sliderPosition(x_des):

	DEBUG = 0
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

	#Nichols Ziegler tuning ---unstable! Max Shepherd 7/17/18
	# KUltimate = 0.015
	# OscPeriod = 1/24
	# K_p = 0.009
	# K_i = 0.02
	# K_d = 0.001

	e_D = 0
	iTerm = 0
	last_pwm = 100
	# x2 = [0.0]*5; xpast2 = [0.0]*5


	if DEBUG:
		# track data
		my_data = numpy.zeros((5000, 10))
		i = 0
	
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


	# print "Press CTRL + C to return to desired position menu."

	while True:
		try:
			if motor_current > 6:
				print 'Motor current too high... quitting.'
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

			if DEBUG:
				# store some data	
				if t_elapsed < 1:
					my_data[i,0] = t_elapsed
					my_data[i,1] = pwm_feedback
					my_data[i,2] = dist
					my_data[i,3] = encoder.readCounter()
					my_data[i,4] = int(pTerm)
					my_data[i,5] = iTerm
					my_data[i,6] = int(dTerm)
					my_data[i,7] = dTermFiltered
					my_data[i,8] = motor_current
					my_data[i,9] = motor_current_filtered
					i = i+1

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
			last_time = current_time
			last_position = current_position


			if abs(pTerm) < 10 and abs(iTerm) < 10 and abs(dTerm) < 10 or t_elapsed > time_limit:
				PWM_Loop = False
				#print "Operation Complete - elapsed time:",t_elapsed

				if DEBUG:
					my_data = my_data[:i]
					# print("Done recording. Data stored in my_data.csv")
					column_names = ['time','pwm','error','position','K_p','K_i','K_d','dTermFiltered','motor_current','motor_current_filtered']
					with open("my_data.csv", "w") as output:
						writer = csv.writer(output,delimiter=',')
						writer.writerow(column_names)
						writer = csv.writer(output, lineterminator='\n',quotechar='|')
						writer.writerows(my_data[1:i,0:10])

				break

		except KeyboardInterrupt:
			print 'Interrupted...'
			break

	wp.pwmWrite(pwm_pin, 0)
	
	