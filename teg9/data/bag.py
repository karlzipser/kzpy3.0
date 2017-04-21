
from kzpy3.vis import *
import rospy
import rosbag
import cv2
from cv_bridge import CvBridge,CvBridgeError
bridge = CvBridge()

image_topics = ['left_image','right_image']
single_value_topics = ['steer','state','motor','encoder','GPS2_lat']
vector3_topics = ['acc','gyro','gyro_heading']#,'gps']
camera_sides = ['left','right']

A = {}

def preprocess(path,A):
    timer = Timer(0)
    
    for topic in image_topics + single_value_topics:
        A[topic] = []
    for topic in vector3_topics:
        A[topic+'_x'] = []
        A[topic+'_y'] = []
        A[topic+'_z'] = []

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
                    A[topic+'_x'].append([t,m[1].x])
                    A[topic+'_y'].append([t,m[1].y])
                    A[topic+'_z'].append([t,m[1].z])

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
    
    A['state'] = array(A['state'])
    A['steer'] = array(A['steer'])
    A['motor'] = array(A['motor'])
    A['gyro_x'] = array(A['gyro_x'])
    A['gyro_y'] = array(A['gyro_y'])
    A['gyro_z'] = array(A['gyro_z'])
    A['acc_x'] = array(A['acc_x'])
    A['acc_y'] = array(A['acc_y'])
    A['acc_z'] = array(A['acc_z'])

    return A

"""
import kzpy3.teg9.data.bag as bag
A=bag.preprocess('/media/karlzipser/ExtraDrive3/from_Mr_Yellow/Mr_Yellow_Fern_14April2017/direct_Fern_aruco_1_14Apr17_22h19m52s_Mr_Yellow/bair_car_2017-04-14-22-23-05_6.bag' )

for t in ts:
    ...:     Hs.append(A['gyro_heading'][t][0])
    ...:     Ts.append(t)

"""




def graph(A):


    figure('MSE')
    clf()
    plot(A['state'][:,0]-A['state'][0,0],A['state'][:,1])
    plot(A['motor'][:,0]-A['motor'][0,0],A['motor'][:,1])
    plot(A['steer'][:,0]-A['steer'][0,0],A['steer'][:,1])

    
    figure('gyro')
    clf()
    plot(A['gyro_x'][:,0]-A['gyro_x'][0,0],A['gyro_x'][:,1])
    plot(A['gyro_y'][:,0]-A['gyro_y'][0,0],A['gyro_y'][:,1])
    plot(A['gyro_z'][:,0]-A['gyro_z'][0,0],A['gyro_z'][:,1])

    figure('acc')
    clf()
    plot(A['acc_x'][:,0]-A['acc_x'][0,0],A['acc_x'][:,1])
    plot(A['acc_y'][:,0]-A['acc_y'][0,0],A['acc_y'][:,1])
    plot(A['acc_z'][:,0]-A['acc_z'][0,0],A['acc_z'][:,1])

    for i in range(300):
        xlim(i/10.,i/10.+3)
        pause(0.05)



def animate(A):
    for a in A['left_image']:
        mi_or_cv2(a[1],cv=True,delay=30,title='animate')