import mpu6050_lib2
from Functions_VSOV1 import *
import traceback

# Set up wiringpi pins to be GPIO
wp.wiringPiSetupGpio()

# Assign motor driver pins and set modes
pwm_pin = 12
dir_pin = 16
wp.pinMode(dir_pin, 1)
wp.pinMode(pwm_pin, 2)

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

t_start = time.time()

#======================================================================================================                         
while True:
    try: 
        user_input = str(raw_input('Test another position? (y/n)'))
        if user_input == 'y':
            user_input = raw_input('Desired position in % travel (0 to 100): ')
            current_position = encoder.readCounter() / scale_perc
            user_input = eval(user_input)

            #Assign User Input to the Motor
            #ANKLE ENCODER (Ankle Angle)
            #current_angle = SingleAngle_I2C()-calib_offset
            #print math.fabs(current_angle)
            #if math.fabs(current_angle)<1.5:                     
            if 0 <= user_input <= 110: #If user_input is within stroke
                desired_position = user_input * scale_perc
                sliderPosition(desired_position)
                current_position = int(round(encoder.readCounter()/scale_perc))
                print 'Current position = %s %%' % current_position                             
            else:
                print "Out of range. Please enter a value between 0 and 100."
                print 'user_input = ', user_input 
            #else:
                #print 'Ankle is flexed too much!' 
        if user_input == 'n':
            wp.pwmWrite(pwm_pin, 0)
            encoder.close()
            quit()
            break

    except KeyboardInterrupt:
        wp.pwmWrite(pwm_pin, 0)
        encoder.close()
        quit()

