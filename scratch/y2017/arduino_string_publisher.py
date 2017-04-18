import std_msgs.msg
import rospy 
from kzpy3.utils import *

baudrate=115200
timeout=0.25

steer_in_pub = rospy.Publisher('/bair_car/steer_in', std_msgs.msg.Int32, queue_size=5) 
motor_in_pub = rospy.Publisher('/bair_car/motor_in', std_msgs.msg.Int32, queue_size=5) 
button_in_pub = rospy.Publisher('/bair_car/button_in', std_msgs.msg.Int32, queue_size=5) 
rospy.init_node('arduino_output_publisher') 

"""
sudo chmod 666 /dev/ttyACM*

"""

def get_arduino_serial_connections(baudrate, timeout):
    sers = []
    ACM_ports = [os.path.join('/dev', p) for p in os.listdir('/dev') if 'ttyACM' in p]
    for ACM_port in ACM_ports:
        try:
            sers.append(serial.Serial(ACM_port, baudrate=baudrate, timeout=timeout))
            print('Opened {0}'.format(ACM_port))
        except:
            pass
    return sers


serial_connections = get_arduino_serial_connections(baudrate, timeout)

mse = 'mse'

while not rospy.is_shutdown():
    for ser in serial_connections:
        try:
            ser_str = ser.readline()
            #print ser_str
            exec('ser_tuple = list({0})'.format(ser_str))
            if ser_tuple[0] == 'mse':
                button_in_pub.publish(ser_tuple[1])
                steer_in_pub.publish(ser_tuple[2])
                motor_in_pub.publish(ser_tuple[3])
        except:
            pass




