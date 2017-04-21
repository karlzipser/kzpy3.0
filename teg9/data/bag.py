
from kzpy3.vis import *
import rospy
import rosbag
import cv2
from cv_bridge import CvBridge,CvBridgeError
bridge = CvBridge()

image_topics = ['left_image','right_image']
single_value_topics = ['steer','state','motor','encoder','GPS2_lat']
vector3_topics = ['acc','gyro','gyro_heading']#,'gps']
all_topics = image_topics + single_value_topics + vector3_topics
camera_sides = ['left','right']

A = {}

def preprocess(path,A):
    timer = Timer(0)
    
    for topic in all_topics:
        A[topic] = []

    try:
        cprint(path,'yellow')

        bag = rosbag.Bag(path)

        for topic in single_value_topics:
            for m in bag.read_messages(topics=['/bair_car/'+topic]):
                t = round(m.timestamp.to_time(),3) # millisecond resolution
                if not isinstance(m[1].data,(int,long,float)):
                    print("if not isinstance(m[1].data,(int,long,float)):")
                    print(d2s("m[1].data = ",m[1].data))
                    assert(False)
                #A[topic][t] = m[1].data
                A[topic].append([t,m[1].data])
        
        for topic in vector3_topics:
            if topic != 'gps':
                for m in bag.read_messages(topics=['/bair_car/'+topic]):
                    t = round(m.timestamp.to_time(),3)
                    if not isinstance(m[1].x,(int,long,float)):
                        print("if not isinstance(m[1].x,(int,long,float)):")
                        print(d2s("m[1].x = ",m[1].x))
                        assert(False)
                    if not isinstance(m[1].y,(int,long,float)):
                        print("if not isinstance(m[1].y,(int,long,float)):")
                        print(d2s("m[1].y = ",m[1].y))
                        assert(False)
                    if not isinstance(m[1].z,(int,long,float)):
                        print("if not isinstance(m[1].x,(int,long,float)):")
                        print(d2s("m[1].z = ",m[1].z)) 
                        assert(False)
                    A[topic].append([t,[m[1].x,m[1].y,m[1].z]])

        try:
            topic = 'gps'
            for m in bag.read_messages(topics=['/bair_car/'+topic]):
                t = round(m.timestamp.to_time(),3)
                A[topic].append([t,[m[1].latitude,m[1].longitude,m[1].altitude]])
        except:
            print 'gps problem'

        color_mode = "rgb8"

        for s in camera_sides:
            for m in bag.read_messages(topics=['/bair_car/zed/'+s+'/image_rect_color']):
                t = round(m.timestamp.to_time(),3)
                A[s+'_image'].append([t,bridge.imgmsg_to_cv2(m[1],color_mode)])
        
    except Exception as e:
        print e.message, e.args


    print(d2s('Done in',timer.time(),'seconds'))
    
    return A

"""
import kzpy3.teg9.data.bag as bag
A=bag.preprocess('/media/karlzipser/ExtraDrive3/from_Mr_Yellow/Mr_Yellow_Fern_14April2017/direct_Fern_aruco_1_14Apr17_22h19m52s_Mr_Yellow/bair_car_2017-04-14-22-23-05_6.bag' )

for t in ts:
    ...:     Hs.append(A['gyro_heading'][t][0])
    ...:     Ts.append(t)

"""




def graph(A):
    state = array(A['state'])
    steer = array(A['steer'])
    motor = array(A['motor'])
    gyro = array(A['gyro'])
    figure('graph')
    clf()
    plot(steer[:,0],steer[:,1])
    plot(motor[:,0],motor[:,1])
    plot(state[:,0],10*state[:,1])
    figure('IMU')
    plot(gyro[0,:])
    

