print "RPi_server.py server side"

##############
#
ON_RPi = True
if ON_RPi:
    print("*** on RPi ****")
    import sys
    sys.path.insert(0, "/home/pi")
    from kzpy3.utils import *
    import RPi.GPIO as GPIO
    SERVO_IN = 38
    MOTOR_IN = 40
    out_pins = [SERVO_IN,MOTOR_IN]
    def gpio_setup():
        print('gpio_setup')
        GPIO.setmode(GPIO.BOARD)
        for p in out_pins:
            GPIO.setup(p,GPIO.OUT)
    gpio_setup() 
    pwm_motor = GPIO.PWM(40,50)
    pwm_servo = GPIO.PWM(38,50)
    pwm_motor.start(0)
    pwm_servo.start(0)
else:
    print("*** not RPi ****")
    from kzpy3.utils import *
#
##############

control_path = '/home/pi/camera_control.txt'
STANDBY = 'STANDBY'
CAPTURE = 'CAPTURE'
QUIT = 'QUIT'
list_of_strings_to_txt_file(control_path,[STANDBY])



##############
#
import socket
host = '0.0.0.0'
port = 5000
serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.bind((host, port))
serversocket.listen(5) # become a server socket, maximum 5 connections
connection, address = serversocket.accept()
#
##############

# http://stackoverflow.com/questions/17386487/python-detect-when-a-socket-disconnects-for-any-reason
# http://stackoverflow.com/questions/667640/how-to-tell-if-a-connection-is-dead-in-python
try:
    while True:
        buf = connection.recv(64)
        if len(buf) != "":
            if buf == 'q':
                try:
                    GPIO.cleanup()
                    list_of_strings_to_txt_file(control_path,[QUIT])
                except:
                    print("*** not RPi ****")
                time.sleep(0.1)
                serversocket.close()
                print('\nCleaning up.')
                break
            elif buf == 'c':
                cmd = CAPTURE
                print cmd
                list_of_strings_to_txt_file(control_path,[cmd])
            elif buf == 'x':
                cmd = STANDBY
                print cmd
                list_of_strings_to_txt_file(control_path,[cmd])
            elif buf == 'z':
                cmd = QUIT
                print cmd
                list_of_strings_to_txt_file(control_path,[cmd])
            else:
                try:
                    t = eval(buf)
                except:
                    t = False
                if t:
                    servo_ds = t[0]
                    motor_ds = t[1]
                    print(d2s(servo_ds,motor_ds))
                    if ON_RPi:
                        pwm_servo.ChangeDutyCycle(servo_ds)
                        if motor_ds >= 0:
                            pwm_motor.ChangeDutyCycle(motor_ds)
                else:
                    pass
        else:
            print("*** No Data received from socket ***")
            try:
                GPIO.cleanup()
                list_of_strings_to_txt_file(control_path,[QUIT])
            except:
                print("*** not RPi ****")
            time.sleep(0.1)
            serversocket.close()
            print('\nCleaning up.')
            break


except KeyboardInterrupt:        
    serversocket.close()
    if ON_RPi:
        GPIO.cleanup()
        list_of_strings_to_txt_file(control_path,[QUIT])
    print('cleaned up.')

