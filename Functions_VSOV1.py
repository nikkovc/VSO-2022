# Variable Stiffness Ankle Functions
# Nikko Van Crey, Marcos Cavallin
# Uses a lot of code written by Delaney Miller
# Summer 2017

# Description: Helper functions to run homing routine and move ankle to desired position

# Dependencies: time, numpy, csv, wiringpi, LS7366R.py

# Import functions
import time
import numpy
import csv
import wiringpi as wp
#import AS5048B as AS
from ADC import MCP3008
from LS7366R import LS7366R
import Adafruit_GPIO.SPI as SPI
import mpu6050_lib2
import BLNKM as LED
#import bme280 as bme

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
# old value scale_perc = 2333.56 #Stroke is 245,934 encoder counts, so each percent is 2,459.34 counts, then subtract a little for safety
scale_perc = 7073.85 #VSOV2: Stroke was 57.2398mm, subtract 1mm for safety and converto to encoder counts I get 707385 VSOV1:Stroke is 327402 encoder counts, so each percent is 3274.02 counts, then subtract a little for safety

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

#Initialize IMU
mpu2 = mpu6050_lib2.mpu6050(0X68)

#Initialize LED
LED.initialize()

#Initialize Barometer
#bme.setup()


#Clear error flag?
data1 = 0b01000000
foo1 = chr(data1)
data2 = 0b00000001
foo2 = chr(data2)
foobar = ''.join([foo1, foo2])
sendData = foobar
#recvData = wp.wiringPiSPIDataRW(SPIchannel, sendData)

#Import horizontal vector (csv) describring dorsiflexion stiffness as a function of slider position (in 1 mm increments)
#slider2stiffness = numpy.genfromtxt('slider2stiffness.csv', delimiter=',') #This file is for a cam that max made.


# An offset term may or may not be necessary when using the ADC
def get_current():
       	voltage = (adc.read_adc(0)*5.0)/1024.0
        Uc = 5.0
        offset = .5*Uc
        #G = .08 #use with LEM GHS10SME 10A current sensor
        G = 0.2 # use with Allegro ACS724 10A current sensor (Boards 7-9, plus others)
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
def SingleAngle_I2C():    #reports back the current ankle angle
   
   recvData1 = wp.wiringPiI2CReadReg8(h1,0xFF+1) #8bit value
   recvData2 = wp.wiringPiI2CReadReg8(h1,0xFE+1) #6bit value
   value1 = (recvData1*360.0/256.0)/64.0
   value2 = recvData2*64*360/16384.0
   angle = value1 + value2
   
   #print 'angle = ', angle
   return angle

#This is for an I2C version of the encoder AS5048B
def SingleAngle_I2C_Dial():	#reports back the current ankle angle
	recvData1 = wp.wiringPiI2CReadReg8(h2,0xFE)
	recvData2 = wp.wiringPiI2CReadReg8(h2,0xFF)
	recvData1 = recvData1 * (2**8)
	recvData2 = recvData2 * 4
	recvData = recvData1+recvData2
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
	
	# track data
	my_data = numpy.zeros((5000, 4))
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

			# print "Motor current: %.2f A" % motor_current

			# store some data
			t_elapsed = time.time() - start_time
			if t_elapsed < 1:
				my_data[i,0] = t_elapsed
				#my_data[i,1] = pwm_feedback
				my_data[i,3] = dist
				#my_data[i,3] = encoder.readCounter()
				#my_data[i,4] = int(pTerm)
				#my_data[i,5] = iTerm
				#my_data[i,6] = int(dTerm)
				#my_data[i,7] = dTermFiltered
				my_data[i,1] = motor_current
				my_data[i,2] = motor_current_filtered
				i = i+1
			
			# figure out direction
			
			if pwm_feedback < 0:
				wp.digitalWrite(dir_pin, 0)
			elif pwm_feedback > 0:
				wp.digitalWrite(dir_pin, 1)


			
			wp.pwmWrite(pwm_pin, pwm)

			# update error term
			x_act = encoder.readCounter()
			dist_last = dist
			dist = int(x_des - x_act)
			last_time = current_time
			last_position = current_position


			if abs(pTerm) < 10 and abs(iTerm) < 10 and abs(dTerm) < 10 or t_elapsed > 0.6:
				PWM_Loop = False
				# print "Operation Complete"

				my_data = my_data[:i]
				# print("Done recording. Data stored in my_data.csv")
				#column_names = ['time','pwm','error','position','K_p','K_i','K_d','dTermFiltered','motor_current','motor_current_filtered']
				with open("my_data.csv", "w") as output:
					writer = csv.writer(output,delimiter=',')
					#writer.writerow(column_names)
					writer = csv.writer(output, lineterminator='\n',quotechar='|')
					writer.writerows(my_data[1:i,0:10])


				break

		except KeyboardInterrupt:
			break

	wp.pwmWrite(pwm_pin, 0)
	my_data = my_data[:i]

	#print("Done recording. Data stored in my_data.csv")

	#with open("my_data.csv", "w") as output:
		#writer = csv.writer(output,delimiter=',')
		#writer = csv.writer(output, lineterminator='\n',quotechar='|')
		#writer.writerows(my_data[1:i,0:9])





from collections import deque
#import BLNKM as LED
from scipy.ndimage import gaussian_filter1d
import math

class SwingPhaseDetector:
    ''' Sprite: This class deals with swing phase detection '''
    
    CONST_MEDFILT_SIZE = 5
    CONST_CACHE_SIZE = 20

    # These thresholds are reserved for parameters of the DoG filter
    CONST_SIGMA_ENCODER = 10
    CONST_SIGMA_GYRO = 10 #4
    CONST_ORDER = 1
    CONST_MODE = 'reflect'

    def __init__(self, init_time_in, calib_offset_in, init_ang_in, height_in, standing_angle_in, accel_in, gyro_in,\
     filename_in = ['DAQ_ankle_angle.csv','DAQ_gyro.csv','DAQ_acceleration.csv','DAQ_height.csv']):
        ''' Sprite: The constructor for initializing the instance'''
        
        self.calib_offset = calib_offset_in
        self.standing_angle = standing_angle_in
        self.state = 'stance'

        #For Updating Ankle Angle
        self.curr_ang = init_ang_in
        self.prev_ang = init_ang_in
        self.ang_medfilt = deque(self.CONST_MEDFILT_SIZE * [init_ang_in])
        self.ang_cache = deque(self.CONST_CACHE_SIZE * [init_ang_in])

        #For Updating Gyro
        self.curr_gyro = gyro_in
        self.prev_gyro = gyro_in
        self.gyro_medfilt = deque(self.CONST_MEDFILT_SIZE * [0])
        self.gyro_cache = deque(self.CONST_CACHE_SIZE * [0])

        #For Updating Acceleration
        self.curr_accel = accel_in
        self.prev_accel = accel_in
        self.accel_medfilt = deque(self.CONST_MEDFILT_SIZE * [0])
        self.accel_cache = deque(self.CONST_CACHE_SIZE * [0])


        #For Updating Barometer
        self.curr_height = height_in
        self.prev_height = height_in
        self.height_medfilt = deque(self.CONST_MEDFILT_SIZE * [0])
        self.height_cache = deque(self.CONST_CACHE_SIZE * [0])

        
        #For Updating Time
        self.t_start = init_time_in
        self.t_curr = init_time_in

        self.CONST_THRESHOLD_SWING = dict() 
        self.CONST_THRESHOLD_STANCE = dict() 
        self.CONST_THRESHOLD_STANDING = dict()
        self.curr_filter_min = dict()
        self.prev_filter_min = dict()
        self.curr_filter_max = dict()
        self.prev_filter_max = dict()

        #Flags for the state_machine
        self.detect_true = False

        #Sensor Outputs for Gait Detection
        self.sensors = ['ankle_ang','gyro','accel','height']
        self.data_logger = dict()
        for sensor in self.sensors:
        	self.CONST_THRESHOLD_SWING[sensor] = 0 
    		self.CONST_THRESHOLD_STANCE[sensor] = 0 
    		self.CONST_THRESHOLD_STANDING[sensor] = 0
        	self.prev_filter_min[sensor] = 0
	        self.curr_filter_min[sensor] = 0
        	self.prev_filter_max[sensor] = 0
        	self.curr_filter_max[sensor] = 0
	        self.data_logger[sensor] = [[self.t_curr-self.t_start, 0,\
	         self.curr_filter_min[sensor], self.curr_filter_max[sensor], int(self.detect_true)]]

        #This is done because ankle has initial angle (Might not be necessary)
        self.data_logger['ankle_ang'] = [[self.t_curr-self.t_start, init_ang_in ,\
         self.curr_filter_min[sensor], self.curr_filter_max[sensor], int(self.detect_true)]]

        self.filename = filename_in

    __init__.__annotations__ = {'init_time_in': float, 'calib_offset_in': float,\
     'init_ang_in': float, 'filename_in': str, 'return': None}


    def gaussian_filter(self, cache, sensor, CONST_SIGMA):
    	_ = gaussian_filter1d(cache, CONST_SIGMA,\
        order = self.CONST_ORDER, mode = self.CONST_MODE)

        self.prev_filter_min[sensor] = self.curr_filter_min[sensor]
        self.curr_filter_min[sensor] = min(_)
        self.prev_filter_max[sensor] = self.curr_filter_max[sensor]
        self.curr_filter_max[sensor] = max(_)
    
    def update_sensor_logger(self, sensor_value, sensor):
    	''' Nikko: This filters gait detection sensor outputs (Builds upon Sprite's code) '''
		
        self.data_logger[sensor].append([self.t_curr-self.t_start, sensor_value,\
         self.curr_filter_min[sensor], self.curr_filter_max[sensor], int(self.detect_true)])

    def update_gyro_pitch(self, gyro_pitch, curr_time_in):
        ''' Nikko: This method updates the shank acceleration '''
        sensor = 'gyro'
        self.t_curr = curr_time_in
        self.gyro_medfilt.popleft()
        self.gyro_medfilt.append(gyro_pitch)

        gyro_to_store = numpy.median(self.gyro_medfilt)

        self.prev_gyro = self.curr_gyro
        self.curr_gyro = gyro_to_store

        self.gyro_cache.popleft()
        self.gyro_cache.append(self.curr_gyro )

        self.gaussian_filter(self.gyro_cache,sensor, self.CONST_SIGMA_GYRO)
        self.update_sensor_logger(self.curr_gyro,sensor)

    def update_shank_accel(self, accel_forward, curr_time_in):
        ''' Nikko: This method updates the shank acceleration '''
        sensor = 'accel'
        self.t_curr = curr_time_in
        self.accel_medfilt.popleft()
        self.accel_medfilt.append(accel_forward)

        accel_to_store = numpy.median(self.accel_medfilt)

        self.prev_accel = self.curr_accel
        self.curr_accel = accel_to_store

        self.accel_cache.popleft()
        self.accel_cache.append(self.curr_accel )

        self.gaussian_filter(self.accel_cache,sensor, self.CONST_SIGMA_ENCODER)
        self.update_sensor_logger(self.curr_accel,sensor)



    def update_barometer(self, height, curr_time_in):
        ''' Nikko: This method updates the shank acceleration '''
        sensor = 'height'
        self.t_curr = curr_time_in
        self.height_medfilt.popleft()
        self.height_medfilt.append(height-height_in)

        height_to_store = numpy.median(self.height_medfilt)

        self.prev_height = self.curr_height
        self.curr_height = height_to_store

        self.height_cache.popleft()
        self.height_cache.append(self.curr_height)

        #self.gaussian_filter(self.height_cache,sensor,self.CONST_SIGMA_ENCODER)
        #self.update_sensor_logger(self.curr_height,sensor)

    def update_ankle_ang(self, curr_ang_in, curr_time_in):
        ''' Sprite: This method updates the ankle angle '''
        sensor = 'ankle_ang'
        self.t_curr = curr_time_in
        self.ang_medfilt.popleft()
        self.ang_medfilt.append(curr_ang_in - self.calib_offset)

        ang_to_store = numpy.median(self.ang_medfilt)

        self.prev_ang = self.curr_ang
        self.curr_ang = ang_to_store

        self.ang_cache.popleft()
        self.ang_cache.append(self.curr_ang)

        self.gaussian_filter(self.ang_cache,sensor,self.CONST_SIGMA_ENCODER)
        self.update_sensor_logger(self.curr_ang,sensor)
        # print([self.t_curr-self.t_start, self.curr_ang,\
        #  self.curr_filter_min, self.curr_filter_max, self.detect_true]) #debugging

    update_ankle_ang.__annotations__ = {'curr_ang_in': float, 'curr_time_in': float,\
     'return': None}


    def reset_time(self, curr_time_in):
        ''' This method resets the time '''

        self.t_start = curr_time_in
        self.t_curr = curr_time_in
    
    reset_time.__annotations__ = {'curr_time_in': float, 'return': None}


    def reset_calib_offset(self, calib_offset_in):
        ''' This method resets the ankle angle offset '''

        self.calib_offset = calib_offset_in
    
    reset_calib_offset.__annotations__ = {'calib_offset_in': float, 'return': None}


    def write_out(self):
		''' Nikko: This method writes out all the data for all sensors an csv file (Modified version of Sprite's code)'''

		print('Storing data...\n')
		for i,filename in enumerate(self.filename):
			with open(filename, 'w') as f:
				out_data = []
				for row in self.data_logger[self.sensors[i]]:
					for entry in row:
						out_data.append(str(entry)+', ')
					out_data.append('\n')
				f.write(''.join(out_data))
				f.write('\n')

		self.data_logger = []
    
    write_out.__annotations__ = {'return': None}


    def state_machine(self):
		''' Sprite: This is the state_machine method. '''
		# State variable definitions===========================================

		# Swing, State 
		swing_accel_thresh = -4 
		self.CONST_THRESHOLD_SWING['ankle_ang'] = -0.1 #-0.375 #SIGMA 10
		self.CONST_THRESHOLD_SWING['gyro'] = -2 #-7(works well stand alone) #SIGMA 10
		#self.CONST_THRESHOLD_SWING['gyro'] = -20 #SIGMA 4

		# Stance, State 
		self.CONST_THRESHOLD_STANCE['ankle_ang'] = -0.3 #SIMGA 10 (min)
		self.CONST_THRESHOLD_STANCE['gyro'] =  0 #sigma10 


		# Standing, State
		self.CONST_THRESHOLD_STANDING['ankle_ang'] = 0.05 #SIGMA 10
		self.CONST_THRESHOLD_STANDING['gyro'] = 0.25 #SIGMA 10
		#self.CONST_THRESHOLD_STANDING['gyro'] = 1 #SIGMA 4 
		standing_band = 5 #Degrees


		#print self.curr_gyro
		#print self.curr_height
        #print self.curr_accel

        #SWING
		if (not self.detect_true) and self.curr_filter_min['gyro'] < self.CONST_THRESHOLD_SWING['gyro'] and self.curr_filter_min['ankle_ang'] < self.CONST_THRESHOLD_SWING['ankle_ang']:
			self.detect_true = True
			self.state = 'swing'
			return True
		#STANCE
		elif self.detect_true and self.curr_filter_min['ankle_ang'] > self.CONST_THRESHOLD_STANCE['ankle_ang'] and self.curr_filter_min['ankle_ang'] < 0.1 and self.curr_filter_min['gyro'] > self.CONST_THRESHOLD_STANCE['gyro'] : # and self.curr_accel<swing_accel_thresh:
			self.detect_true = False
			self.state = 'stance'
			return False
		#STANDING
		elif self.state == 'stance' and math.fabs(self.curr_ang)<self.standing_angle+standing_band and math.fabs(self.curr_filter_max['ankle_ang']) < self.CONST_THRESHOLD_STANDING['ankle_ang'] and math.fabs(self.curr_filter_max['gyro']) < self.CONST_THRESHOLD_STANDING['gyro']:
			self.detect_true = False
			self.state = 'standing'
			return False
			
		#LOADING TRANSMISSION WHILE STANDING
		elif self.state == 'standing' and (math.fabs(self.curr_ang)>self.standing_angle+standing_band or math.fabs(self.curr_filter_max['ankle_ang']) > self.CONST_THRESHOLD_STANDING['ankle_ang'] or math.fabs(self.curr_filter_max['gyro']) > self.CONST_THRESHOLD_STANDING['gyro']):
			self.detect_true = False
			self.state = 'stance'
			return False

		else:
			return self.detect_true
    state_machine.__annotations__ = {'return': bool}

		

def main():
	''' Sprite: Tester for SwingPhaseDetector '''

	#Ankle Encoder needs time to warm up before it will work properly
	print('Ankle Encoder Warming up\n')
	AS.setup('0x40', '1')
	t_start = time.time()
	current_angle = AS.read_angle()
	while time.time()-t_start<2.0:
		current_angle = AS.read_angle()

	#This calibration just creates an offset that sets the unloaded position of the device to the equilibrium 0 degree angle
	print('Calibrating Encoder Offset\n')
	calib_offset = AS.read_angle()
	current_angle = AS.read_angle()-calib_offset

	#Create an instance of SwingPhaseDetector
	sp_detector = SwingPhaseDetector(init_time_in = time.time(),\
	 calib_offset_in = calib_offset, init_ang_in = current_angle, height_in = height)
	
	#Loop for data acquisition
	while True:
		try:
			curr_ang_in = AS.read_angle()
			sp_detector.update_ankle_ang(curr_ang_in = curr_ang_in, curr_time_in = time.time())
			if sp_detector.state_machine():
				print("Swing Phase!\n")
			else:
				print("Nah\n")
		except KeyboardInterrupt:
			print('Stop!\n')
			sp_detector.write_out()
			break

class StiffnessSelector:
    ''' Sprite: This class deals with the dial to select desired stiffness 
    
        Note the current and desired positions in this class are all in mm,
        as opposed to SliderDriver class where positions are in encoder count,
        they will be converted to each other by scale = 12578.0
    '''
    
    CONST_MIN_POS = 1      #mm
    CONST_MAX_POS = 15     #mm
    CONST_SCALE = 5       #mm
    CONST_THRESHOLD = 10.0 #degree
    CONST_SET_FREQ = 1.0  #s, get new dial reading every CONST_SET_FREQ second

    def __init__(self, init_ang_in, curr_position_mm_in, curr_time_in):
        self.prev_ang = init_ang_in
        self.curr_pos_mm = curr_position_mm_in
        self.prev_time = curr_time_in
        self.desired_position_mm = 0.0
        self.new_stiffness = False
    
    __init__.__annotations__ = {'init_ang_in': float,\
    'curr_position_mm_in': float, 'curr_time_in': float, 'return': None}


    def set_stiffness(self, curr_ang_in, curr_time_in):
    	#CODE BELOW IS FOR THE DIAL
    	
        if curr_time_in - self.prev_time > self.CONST_SET_FREQ:
            # The modulo operator always yields a result 
            # with the same sign as its second operand (or zero);
            angle_change = curr_ang_in - self.prev_ang
            if abs(angle_change) > self.CONST_THRESHOLD:
                if angle_change < -180:
                    angle_change = 360.0+curr_ang_in - self.prev_ang
                if angle_change > 180:
                    angle_change = -self.prev_ang+curr_ang_in - 360.0
                
                #print(angle_change)

                desired_position_mm = round(angle_change / 360.0 * self.CONST_SCALE,2)\
                + self.curr_pos_mm
                if desired_position_mm > self.CONST_MAX_POS:
                    desired_position_mm = self.CONST_MAX_POS
                elif desired_position_mm < self.CONST_MIN_POS:
                    desired_position_mm = self.CONST_MIN_POS
                
                self.desired_position_mm = desired_position_mm
                
                self.prev_time = curr_time_in
                self.prev_ang = curr_ang_in

                self.new_stiffness = True
                return self.desired_position_mm
            else:
                self.new_stiffness = False
                return self.desired_position_mm
        else:
            return self.desired_position_mm
		
    set_stiffness.__annotations__ = {'curr_ang_in': float, 'return': None}
    

    def set_curr_pos(self, curr_position_mm_in):
        self.curr_pos_mm = curr_position_mm_in
    
    set_curr_pos.__annotations__ = {'curr_position_mm_in': float, 'return': None}


    def new_stiffness_status(self):
        return self.new_stiffness
    
    new_stiffness_status.__annotations__ = {'return': bool}

    
    def set_new_stiffness(self, bool_in):
        self.new_stiffness = bool_in
    
    set_new_stiffness.__annotations__ = {'bool_in': bool, 'return': None}




class SliderDriver:
    ''' Sprite: This class drives the slider to the desired position '''
    
    CONST_PWM_LIMIT = 75
    CONST_REACH_TARGET_THRESHOLD = 2000
    CONST_T_THRESHOLD = 2.0

    def __init__(self, pwm_pin_in = 12, dir_pin_in = 16, enable_pin_in = 24,\
    disable_pin_in = 23, kp_in = 0.01, ki_in = 0.8, kd_in = 0.0001):
        # Label pins
        self.pwm_pin = pwm_pin_in
        self.dir_pin = dir_pin_in
        self.enable_pin = enable_pin_in
        self.disable_pin = disable_pin_in
        # Define gains for PID control
        self.kp = kp_in
        self.ki = ki_in
        self.kd = kd_in
        # Track Data
        self.my_data = numpy.zeros((5000, 10))
        self.i = 0
        self.start_time = time.time()
        self.last_time = self.start_time
        self.curr_time = self.start_time
        self.current_position = encoder.readCounter()
        self.last_position = self.current_position
        self.motor_current = 0
        self.motor_current_filtered = 0
        self.last_pwm = 100
        # PID terms
        self.p_term = 0
        self.d_term = 0
        self.d_term_fil = 0
        self.i_term = 0
        self.e_d = 0
        # Target Distance
        self.dist = 0.0
        self.dist_last = self.dist
        # Flag
        self.position_reached = False

    __init__.__annotations__ = {'pwm_pin_in': int, 'dir_pin_in': int, 'enable_pin_in': int,\
    'disable_pin_in': int, 'kp_in': float, 'ki_in': float, 'kd_in': float, 'return': None}


    def update_slider(self, x_des):
        self.last_position = self.current_position
        self.current_position = encoder.readCounter()
        if abs(self.current_position - self.last_position) > 20000:
            self.current_position = self.last_position
            print('abnormal value from motor encoder!')
            self.i_term = 0
        self.dist = x_des - self.current_position
        self.curr_time = time.time()
        
        self.current_time = time.time()
        dt = self.current_time - self.last_time

        v_rot = (self.current_position - self.last_position) / dt  # in counts/s
        v_rot = v_rot * 60.0 / 4096.0  # in rpm

        self.p_term = self.kp * self.dist
        self.e_d = (self.dist-self.dist_last)/(dt)

        #Filter the derivative term
        d_term_last = self.d_term
        d_term_filtered_last = self.d_term_fil
        self.d_term = self.e_d * self.kd
        self.d_term_fil = lpfilter1([d_term_last, self.d_term], [d_term_filtered_last])
        
        # only integrate when not saturated (prevent windup)
        if -self.CONST_PWM_LIMIT < self.last_pwm < self.CONST_PWM_LIMIT:
            self.i_term = self.i_term + (self.ki * self.dist * dt)
    
    update_slider.__annotations__ = {'x_des': int, 'return': None}


    def update_motor(self):
        motor_current_last = self.motor_current			
        motor_current_filtered_last = self.motor_current_filtered
        self.motor_current = get_current()
        self.motor_current_filtered = \
        lpfilter1([motor_current_last, self.motor_current], [motor_current_filtered_last])
    
    update_motor.__annotations__ = {'return': None}


    def update_data(self, pwm_feedback_in):
        t_elapsed = time.time() - self.start_time

        # Update data, not needed at this moment, so commented out
        
        # if t_elapsed < 1:
        #     self.my_data[self.i,0] = t_elapsed
        #     self.my_data[self.i,1] = pwm_feedback_in
        #     self.my_data[self.i,2] = self.dist
        #     self.my_data[self.i,3] = self.current_position
        #     self.my_data[self.i,4] = int(self.p_term)
        #     self.my_data[self.i,5] = self.i_term
        #     self.my_data[self.i,6] = int(self.d_term)
        #     self.my_data[self.i,7] = self.d_term_fil
        #     self.my_data[self.i,8] = self.motor_current
        #     self.my_data[self.i,9] = self.motor_current_filtered
        
        #     self.i = self.i+1
        
        return t_elapsed

    update_data.__annotations__ = {'pwm_feedback_in': int, 'return': None}


    def log_data(self):
        # print "Operation Complete"
        self.my_data = self.my_data[:self.i]
        # print("Done recording. Data stored in my_data.csv")
        column_names = ['time','pwm','error','position',\
        'K_p','K_i','K_d','dTermFiltered','motor_current',\
        'motor_current_filtered']
        with open("drive_slider_data.csv", "w") as output:
            writer = csv.writer(output,delimiter=',')
            writer.writerow(column_names)
            writer = csv.writer(output, lineterminator='\n',quotechar='|')
            writer.writerows(my_data[1:self.i,0:10])
        
        # break ===> error: 'break' not properly in loop
        # consider raising exception
    
    log_data.__annotations__ = {'return': None}


    def reset_data(self):
        self.start_time = self.curr_time
        self.motor_current = 0
        self.motor_current_filtered = 0
        self.last_pwm = 100
        # PID terms
        self.p_term = 0
        self.d_term = 0
        self.d_term_fil = 0
        self.i_term = 0
        self.e_d = 0
        # Target Distance
        self.dist = 0.0
        self.dist_last = self.dist 

    reset_data.__annotations__ = {'return': None}

    
    def drive_slider(self, x_des):
        self.update_slider(x_des)

        #Set PWM frequency
        pwm_feedback = int(self.p_term + self.i_term + self.d_term_fil)
        self.last_pwm = pwm_feedback

        # limit pwm feedback to 80% duty cycle
        if pwm_feedback > self.CONST_PWM_LIMIT:
            pwm_feedback = self.CONST_PWM_LIMIT
        elif pwm_feedback < -self.CONST_PWM_LIMIT:
            pwm_feedback = -self.CONST_PWM_LIMIT
        elif -7 <= pwm_feedback < -2:
            pwm_feedback = -7
        elif -2 <= pwm_feedback < 2: 
            pwm_feedback = 0
        elif 2 <= pwm_feedback < 7:
            pwm_feedback = 7        

        pwm = abs(pwm_feedback)

        # update motor current, not needed, so commented out
        # self.update_motor()

        # store some data
        t_elapsed = self.update_data(pwm_feedback)

        # figure out direction    
        if pwm_feedback <= 0:
            wp.digitalWrite(self.dir_pin, 0)    
        elif pwm_feedback > 0:
            wp.digitalWrite(self.dir_pin, 1)
        
        wp.pwmWrite(self.pwm_pin, pwm)

        if pwm > 70:
            print(pwm)

        # update error term
        self.dist_last = self.dist
        self.last_time = self.current_time
        self.last_position = self.current_position

        if abs(self.dist) < self.CONST_REACH_TARGET_THRESHOLD or t_elapsed > self.CONST_T_THRESHOLD:
            self.reset_data()
            wp.pwmWrite(pwm_pin, 0)

            #Only print it when needed
            if not self.position_reached:
                self.position_reached = True
                print('reached target position!', x_des, self.current_position)
            
            # Export Data, not needed at this moment, so commented out
            # self.log_data()
        
        return self.current_position
    
    drive_slider.__annotations__ = {'x_des': int, 'return': float}


    def position_reached_status(self):
        return self.position_reached
    
    position_reached_status.__annotations__ = {'return': bool}


    def set_position_reached(self, bool_in):
        self.position_reached = bool_in
    
    set_position_reached.__annotations__ = {'bool_in': bool, 'return': None}



def swing_phase_detection_main_program(encoder,LED,standing_angle): #light
    ''' Sprite: Swing Phase Detection Main Program 
    
        Need to pass in the actual encoder and LED object in the Run_Me program
    '''

    scale = 12578.0 # encoder conversion scale (counts to mm)

    # Ankle Encoder needs time to warm up before it will work properly
    print('Ankle Encoder and Dial Warming up\n')
    t_start = time.time()
    current_angle_ankle = SingleAngle_I2C()
    #current_angle_dial = SingleAngle_I2C_Dial()
    current_angle_dial=0
    
    while time.time()-t_start<2.0:
        current_angle_ankle = SingleAngle_I2C()
        #current_angle_dial = SingleAngle_I2C_Dial()

    # This calibration just creates an offset that sets the unloaded position of 
    # the device to the equilibrium 0 degree angle
    print('Calibrating Encoder Offset\n')
    calib_offset = SingleAngle_I2C()
    current_angle_ankle = SingleAngle_I2C()-calib_offset


    #current_angle_dial = SingleAngle_I2C_Dial()
    current_angle_dial = 0 #place Holder until we have the dial
    current_position_mm = round((encoder.readCounter() / scale),2)

    #Initial Gyro Shank Pitch
    gyro_data = mpu2.get_gyro_data() 
    gyro_pitch = gyro_data['x']

    #Initial Shank Forward Acceleration
    #accel_data = mpu2.get_accel_data()                   
    #accel_forward = accel_data['z']
    accel_forward = 0 #Placeholder when accelerometer is not used

    #Initial Barometer Leg Height
    #height = bme.readBME280Altitude()*100
    height = 0 #Placeholder when barometer is not used


    #Creates an instance of SwingPhaseDetector, StiffnessSelector and SliderDriver
    sp_detector = SwingPhaseDetector(init_time_in = time.time(),\
     calib_offset_in = calib_offset, init_ang_in = current_angle_ankle, height_in = height, standing_angle_in = standing_angle, accel_in = accel_forward, gyro_in = gyro_pitch)
    
    stiff_selector = StiffnessSelector(current_angle_dial, current_position_mm,time.time())

    sd = SliderDriver()

    # Flags for printing
    print_swing_phase = True
    print_stance_phase = True
    print_standing = True

    #Loop for data acquisition
    while True:
        try:
        	#Ankle Angle
            curr_ang_in = SingleAngle_I2C()

            #Time
            curr_time = time.time()

            #Shank Forward Acceleration
            #accel_data = mpu2.get_accel_data()                   
            #accel_forward = accel_data['z']
            #accel_forward = 0

            #Gyro Shank Pitch
            gyro_data = mpu2.get_gyro_data() 
            gyro_pitch = gyro_data['x']
            
            #Barometer Leg Height
            #height = bme.readBME280Altitude()*100
            height = 0

            sp_detector.update_ankle_ang(curr_ang_in = curr_ang_in, curr_time_in = curr_time)
            #sp_detector.update_shank_accel(accel_forward = accel_forward, curr_time_in = curr_time)
            sp_detector.update_gyro_pitch(gyro_pitch = gyro_pitch, curr_time_in = curr_time)
            #sp_detector.update_barometer(height = height, curr_time_in = curr_time)
           
            if sp_detector.state_machine():

                # commented out LED commands, somehow it messed up when
                # the dial and encoder are plugged in at the same time

                LED.fadetoRGB(5,0,255) # Fade to Blue


                #CODE IS COMMENTED OUT BECAUSE WE ARE JUST TRYING TO TEST SWING DETECTOR

                # Get desired position in mm
                desired_position_mm = \
                stiff_selector.set_stiffness(SingleAngle_I2C_Dial(),curr_time)

                if stiff_selector.new_stiffness_status():
                    current_position = sd.drive_slider(int(desired_position_mm*scale))
                    #for debugging purposes, print it all the time
                    #print('Swing Phase', int(desired_position_mm*scale), current_position)
                    if print_swing_phase:
                        print('Swing Phase', int(desired_position_mm*scale), current_position)
                        print_swing_phase = False
                        print_stance_phase = True
                        sd.set_position_reached(False)
                    stiff_selector.set_curr_pos(current_position/scale)
                else:
                    if print_swing_phase:
                        print('Swing Phase', round(desired_position_mm,2))
                        print_swing_phase = False
                        print_stance_phase = True
                        print_standing = True
                        sd.set_position_reached(False)
            else:
                if sp_detector.state == 'stance':
	                LED.fadetoRGB(255,0,0) # Red
	                wp.pwmWrite(pwm_pin,0)
	                if print_stance_phase:
	                    print('Stance Phase')
	                    print_stance_phase = False
	                    print_swing_phase = True
	                    print_standing = True
	                    sd.set_position_reached(False)

                if sp_detector.state == 'standing':
	                LED.fadetoRGB(0,255,0) # Green
	                if print_standing:
	                    print('Standing')
	                    print_stance_phase = True
	                    print_standing = False
	                    print_swing_phase = True
	                    sd.set_position_reached(False)

        
        except KeyboardInterrupt:
            print('Stop!\n')    
            sp_detector.write_out()
            wp.pwmWrite(pwm_pin, 0)
            break

if __name__ == "__main__":
	main()

