
from kzpy3.vis import *
import rospy
import rosbag
import cv2
from cv_bridge import CvBridge,CvBridgeError
import threading

bridge = CvBridge()

image_topics = ['left_image','right_image']
single_value_topics = ['steer','state','motor','encoder','GPS2_lat']
vector3_topics = ['acc','gyro','gyro_heading']#,'gps']
camera_sides = ['left','right']



def multi_preprocess(A,bag_folder_path,bagfile_range=[]):

    bag_files = sorted(gg(opj(bag_folder_path,'*.bag')))
    if len(bagfile_range) > 0:
        bag_files = bag_files[bagfile_range[0]:(bagfile_range[1]+1)]
    
    threading.Thread(target=multi_preprocess_thread,args=[A,bag_files]).start()


def multi_preprocess_thread(A,bag_files):
    for b in bag_files:
        if A['STOP_LOADER_THREAD']:
            A['STOP_LOADER_THREAD'] = False
            print('Stopping multi_preprocess_thread.')
            break
        preprocess(A,b)


def get_new_A():
    A = {}
    A['current_img_index'] = 0
    A['STOP_LOADER_THREAD'] = False
    A['STOP_ANIMATOR_THREAD'] = False
    A['d_indx'] = 1.0
    return A


def preprocess(A,path):
    timer = Timer(0)
    

    for topic in image_topics + single_value_topics:
        if topic not in A:

            A[topic] = []
    for topic in vector3_topics:
        if topic+'_x' not in A:
            A[topic+'_x'] = []
            A[topic+'_y'] = []
            A[topic+'_z'] = []

    if True:#try:
        cprint('Loading bagfile '+path,'yellow')

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
        """
        try:
            topic = 'gps'
            for m in bag.read_messages(topics=['/bair_car/'+topic]):
                t = round(m.timestamp.to_time(),3)
                A[topic].append([t,[m[1].latitude,m[1].longitude,m[1].altitude]])
        except:
            print 'gps problem'
        """
        color_mode = "rgb8"

        for s in camera_sides:
            for m in bag.read_messages(topics=['/bair_car/zed/'+s+'/image_rect_color']):
                t = round(m.timestamp.to_time(),3)
                A[s+'_image'].append([t,bridge.imgmsg_to_cv2(m[1],color_mode)])
        
    
    #except Exception as e:
    #    print e.message, e.args


    print(d2s('Done in',timer.time(),'seconds'))
    """
    A['state'] = array(A['state'])
    A['steer'] = array(A['steer'])
    A['motor'] = array(A['motor'])
    A['gyro_x'] = array(A['gyro_x'])
    A['gyro_y'] = array(A['gyro_y'])
    A['gyro_z'] = array(A['gyro_z'])
    A['acc_x'] = array(A['acc_x'])
    A['acc_y'] = array(A['acc_y'])
    A['acc_z'] = array(A['acc_z'])
    """


"""
import kzpy3.teg9.data.bag as bag
reload(bag)
A = {}
A['current_img_index'] = 0
bag.preprocess(A,'/media/karlzipser/ExtraDrive3/from_Mr_Yellow/Mr_Yellow_Fern_14April2017/direct_Fern_aruco_1_14Apr17_22h19m52s_Mr_Yellow/bair_car_2017-04-14-22-23-05_6.bag' )

bag.animate(A)
"""




def graph(A):


    figure('MSE')
    clf()
    
    steer = array(A['steer'])
    plot(steer[:,0],steer[:,1])
    
    motor = array(A['motor'])
    plot(motor[:,0],motor[:,1])
    """
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
    """
    while True:
        indx = int(A['current_img_index'])
        if indx < 0:
            indx = 0
        elif indx >= len(A['left_image']):
            indx = len(A['left_image'])-1
        t = A['left_image'][indx][0]
        xlim(t-3,t)
        pause(0.05)


def animator_thread(A):

    while A['STOP_ANIMATOR_THREAD'] == False:

        indx = int(A['current_img_index'])
        if indx < 0:
            indx = 0
        elif indx >= len(A['left_image']):
            indx = len(A['left_image'])-1

        a = A['left_image'][indx]
        mi_or_cv2(a[1],cv=True,delay=30,title='animate')

        A['current_img_index'] += A['d_indx']


def start_animation(A):
    threading.Thread(target=animator_thread,args=[A]).start()

def animate(A):
    while A['current_img_index'] < len(A['left_image']):
        a = A['left_image'][A['current_img_index']]
        mi_or_cv2(a[1],cv=True,delay=30,title='animate')
        A['current_img_index'] += 1

"""
import kzpy3.teg9.data.bag as bag
reload(bag)
A=bag.get_new_A()
bag_folder_path='/media/karlzipser/ExtraDrive1/bair_car_data_10/caffe_direct_Smyth_tape_16Jan17_16h52m24s_Mr_Silver'
bag.multi_preprocess(A,bag_folder_path,[0,1])

bag.start_animation(A)   
bag.graph(A)
"""

