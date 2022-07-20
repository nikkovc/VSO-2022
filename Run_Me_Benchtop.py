
import smbus, time, math, random, threading, numpy, csv
import mpu6050_lib2
import BLNKM as LED
from LS7366R import LS7366R
from Functions_Benchtop import *
import traceback
import wiringpi as wp



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
        last_position = encoder.readCounter()
        sample_rate = 0.01
        keep_going = True
        wp.pwmWrite(pwm_pin, 65) #This sets the PWM frequency with which to jam it into the hard stop...
        time.sleep(0.3)

        while keep_going:
        #for x in range(20):
            last_position = encoder.readCounter()
            time.sleep(sample_rate)
            current_position = encoder.readCounter()
            motor_current = get_current()
            print "Motor current: %.2f A" % motor_current
            error = current_position - last_position
            print "current_position", current_position, '\n'
            if -400 <= error <= 400:
                keep_going = False
        wp.pwmWrite(pwm_pin, 0)

        encoder.clearCounter()
        current_position = int(round(encoder.readCounter() / scale))
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
    to_do = str(raw_input('What do you want to do? (dial, stiffness, get slider position, get ankle angle, zero ankle, read dial, set-zero, quit) '))
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
    if to_do == 'get slider position':
        LED.fadetoRGB(102,0,204) # Purple #Checking Data/No Action
        current_position = int(round(encoder.readCounter() / scale))
        print "The current slider position is %s mm" % current_position
        current_position_perc = int(round(encoder.readCounter() / scale_perc))
        print "The current slider position is %s %%" % current_position_perc
        battery_V = check_battery()
        if battery_V < 9.0:
            print "BATTERY VOLTAGE TOO LOW!  Charge your battery."
            LED.fadetoRGB(255,0,0)
            quit()

#======================================================================================================
    if to_do == 'get ankle angle':
        for i in xrange(200):
            # LED.fadetoRGB(102,0,204) # Purple #Checking Data/No Action
            current_angle = SingleAngle_I2C()-calib_offset
            print 'Angle = ', current_angle
            # print 'stiffness =', current_position
            time.sleep(0.1)


#======================================================================================================
    if to_do == 'read dial':
        address_dial = 0x42
        for i in xrange(100):
            # LED.fadetoRGB(102,0,204) # Purple #Checking Data/No Action
            dial_angle = SingleAngle_I2C_Dial()-calib_offset
            print 'dial = ', dial_angle
            time.sleep(0.1)

#=========================================================================================================
    if to_do == 'set_zero':
        while True:
            user_input = raw_input('What should I think the current position is?: ')
            if RepresentsInt(user_input):
                current_position = eval(user_input)
                desired_position = current_position
                print 'okay, I think the current position is ', current_position
                break
            elif user_input == 'back':
                break
            else:
                print 'failed... try an integer! or type back'

#======================================================================================================
    if to_do == 'record':
        #New file
        file_name = raw_input('Name the file: ') + '.csv'
        with open(file_name, "wb") as myfile:
            writer = csv.writer(myfile, delimiter=',')
        user_input = raw_input('for how many seconds?: ')
        experiment_duration = eval(user_input)  #Seconds
        sampling_frequency = 30 #Hz. rough!
        AddDataPoint(file_name,['Stiffness', current_position]) #Add the new stiffness to the csv file
        AddDataPoint(file_name,['Time', 'Angle'])
        start_time = time.time()
        t_elapsed = 0
        while t_elapsed < experiment_duration:
            time.sleep(1/sampling_frequency)
            current_angle = SingleAngle_I2C()-calib_offset
            t_elapsed = time.time()-start_time
            AddDataPoint(file_name,[t_elapsed, current_angle])

#======================================================================================================
    # if to_do == 'dial-stiffness':
    #     LED.fadetoRGB(5,0,255) # Fade to Blue
    #     # check_battery_or_quit()
    #     while True:
    #         try:
    #             user_input = str(raw_input('Test another position? (y/n)'))
    #             if user_input == 'y':
    #                 user_input = eval(raw_input('Desired position in mm (0 to 56): '))
    #                 current_angle = SingleAngle_I2C()-calib_offset
    #                 if abs(current_angle)<500:  # <---------------- Needs to be changed to +/- 1 once ankle encoder is added ENCedit
    #                     if 0 <= user_input <= 56:
    #                         desired_position_mm = user_input
    #                     elif 180 <= user_input <= 1060:
    #                         desired_position_mm = round(ConvertStiffnessToPosition(user_input),1)
    #                     else:
    #                         print "Out of range. Please enter a value between 0 and 56 or a stiffness between 180 and 1060"
    #                         break
    #                     desired_position_encoder = desired_position_mm * scale
    #                     sliderPosition(desired_position_encoder)
    #                     current_position_mm = round(encoder.readCounter()/scale,2)
    #                     print 'Current position as mm =', current_position_mm, 'and as k =', ConvertPositionToStiffness(current_position_mm)

    #                 else:
    #                     print 'Ankle is flexed too much!'

    #             if user_input == 'n':
    #                 wp.pwmWrite(pwm_pin, 0)
    #                 encoder.close()
    #                 break

    #         except KeyboardInterrupt:
    #             wp.pwmWrite(pwm_pin, 0)
    #             encoder.close()
    #             quit()

#======================================================================================================
    # if to_do == 'stiffness old':
    #     LED.fadetoRGB(5,0,255) # Fade to Blue
    #     # check_battery_or_quit()
    #     while True:
    #         try:
    #             user_input = str(raw_input('Test another position? (y/n)'))
    #             if user_input == 'y':
    #                 user_input = eval(raw_input('Desired position in mm (0 to 56): '))
    #                 current_angle = SingleAngle_I2C()-calib_offset
    #                 if abs(current_angle)<500:  # <---------------- Needs to be changed to +/- 1 once ankle encoder is added ENCedit
    #                     if 0 <= user_input <= 56:
    #                         desired_position_mm = user_input
    #                     elif 180 <= user_input <= 1060:
    #                         desired_position_mm = round(ConvertStiffnessToPosition(user_input),1)
    #                     else:
    #                         print "Out of range. Please enter a value between 0 and 56 or a stiffness between 180 and 1060"
    #                         break
    #                     desired_position_encoder = desired_position_mm * scale
    #                     sliderPosition(desired_position_encoder)
    #                     current_position_mm = round(encoder.readCounter()/scale,2)
    #                     print 'Current position as mm =', current_position_mm, 'and as k =', ConvertPositionToStiffness(current_position_mm)

    #                 else:
    #                     print 'Ankle is flexed too much!'

    #             if user_input == 'n':
    #                 wp.pwmWrite(pwm_pin, 0)
    #                 encoder.close()
    #                 break

    #         except KeyboardInterrupt:
    #             wp.pwmWrite(pwm_pin, 0)
    #             encoder.close()
    #             quit()

#======================================================================================================
    if to_do == 'stiffness':
        is_walking = raw_input('Is someone Walking on the ankle? y or n: ')
        while True:
            if is_walking == 'y':
                user_input = raw_input('Desired Position (0 to 56 [mm] OR 180 to 1060 [Nm/rad]): ')
                if user_input == 'back':
                    break
                else:
                    user_input = eval(user_input)
                    if 0 <= user_input <= 1300:
                        if 0 <= user_input <= 56:
                            desired_position = user_input
                        elif 200<= user_input <= 1300:
                            desired_position = round(ConvertStiffnessToPosition(user_input),1)
                        last_angles = [0,0,0,0,0,0]
                        in_swing = 0
                        last_in_swing = 1
                        new_step = 0
                        for i in xrange(300):
                            time.sleep(0.01)
                            current_angle = SingleAngle_I2C()
                            in_swing, last_angles = InSwingDetection(last_angles, current_angle)
                            new_step, last_in_swing = NewStepDetection(last_in_swing, in_swing) #new_step says whether a new step has just started

                            if new_step == True and current_position != desired_position:
                                sliderPosition(desired_position*scale, to_do, calib_offset)
                                current_position_mm = round(encoder.readCounter()/scale,2)
                                print 'The new slider position is:', current_position_mm, ' of stiffness:', round(ConvertPositionToStiffness(current_position_mm),1)
                                break
                        else:
                            print('Swing was not detected...')
                    else:
                        print('idk whats happening')
            else:
                user_input = raw_input('Desired Position (0 to 56 [mm] OR 180 to 1060 [Nm/rad]): ')
                if user_input == 'back':
                    break
                else:
                    user_input = eval(user_input)
                    if 0 <= user_input <= 1300:
                        if 0 <= user_input <= 56:
                            desired_position = user_input
                        elif 100<= user_input <= 1300:
                            desired_position = round(ConvertStiffnessToPosition(user_input),1)
                        current_angle = SingleAngle_I2C()
                        for x in xrange(100): #Try for 100 counts to see if the ankle is neutral in order to move the motor
                            if -1000 <= current_angle < 1000:
                                print calib_offset
                                sliderPosition(desired_position*scale, to_do, calib_offset)
                                current_position_mm = round(encoder.readCounter()/scale,2)
                                print 'The new slider position is:', current_position_mm, ' of stiffness:', round(ConvertPositionToStiffness(current_position_mm),1)
                                break
                            time.sleep(0.02)
                        else: #this executes if the for loop is not broken out of
                            print '\n ERROR: the ankle stayed flexed :( \n'

                    else:
                        print 'Out of range. Try again'



#======================================================================================================
    if to_do == 'dial':
        LED.fadetoRGB(255,0,5) # Fade to Red
        # check_battery_or_quit() #Only compatible with boards 7-9
        current_position_mm = round(encoder.readCounter()/scale,2)
        first_dial_angle = SingleAngle_I2C_Dial() #We want to zero it if necessary
        last_dial_angle = 0
        number_turns = 0
        last_in_swing = True
        last_angles = [0,0,0]
        in_swing = 0
        new_step = 0
        starting_position = current_position_mm
        record_yn = str(raw_input('do you want to record? y/n:'))
        if record_yn == 'y':
            file_name = raw_input('Name the file: ') + '.csv'
            with open(file_name, "wb") as myfile:
                writer = csv.writer(myfile, delimiter=',')
            AddDataPoint(file_name,['Time', 'Slider Position (mm)', 'Stiffness (Nm/rad)', 'Dial Position (deg)', 'Motor Encoder (cts)'])

            # AddDataPoint(file_name,['Time', 'Slider Position (mm)', 'Stiffness (Nm/rad)', 'in swing', 'last in swing'])

        print('starting_position: ', starting_position)
        t0 = time.time()

        while True:
            try:
                time.sleep(0.05)
                current_angle = SingleAngle_I2C()-calib_offset
                encoder_value = SingleAngle_I2C_Dial()
                desired_position_mm = round(encoder_value/360*56, 1) + starting_position #This is the desired position in %
                print desired_position_mm

                if 0 <= desired_position_mm <= 56: #If user_input is within stroke
                    desired_position_encoder = desired_position_mm*scale
                elif desired_position_mm > 56:
                	desired_position_encoder = 56*scale
                elif desired_position_mm < 0:
                	desired_position_encoder = 0*scale


            # print('new step detected')
                if abs(desired_position_mm - current_position_mm) > 0.3: #This removes jittery movements caused by encoder noise. -Max 7/17/18
                    sliderPosition(desired_position_encoder, to_do, calib_offset)
                    current_position_mm = round(encoder.readCounter()/scale, 2) #Round to two decimal places
                    # print 'Desired Position (mm): ', desired_position_mm, 'Current position:', current_position_mm
                    print 'k =', "%02.1f" %(ConvertPositionToStiffness(current_position_mm))
                    # print "Motor current: %.2f A" % motor_current

                if record_yn == 'y':
                    AddDataPoint(file_name,[time.time()-t0, current_position_mm, ConvertPositionToStiffness(current_position_mm), encoder_value, encoder.readCounter()])
                    # AddDataPoint(file_name,[time.time()-t0, current_position_mm, in_swing, last_in_swing])


            except KeyboardInterrupt:
                print('Break!')
                print 'The current slider position in mm =: ', current_position_mm, 'which is: ', ConvertPositionToStiffness(current_position_mm),' Nm/rad'
                break


LED.fadetoRGB(102,0,204) # Purple #Checking Data/No Action
