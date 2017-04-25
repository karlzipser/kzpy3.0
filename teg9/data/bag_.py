
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

import kzpy3.data_analysis.aruco_annotator as ann


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



def get_new_A(_=None):
    A = {}
    A['current_img_index'] = 0
    A['STOP_LOADER_THREAD'] = False
    A['STOP_ANIMATOR_THREAD'] = False
    A['STOP_GRAPH_THREAD'] = False
    A['d_indx'] = 1.0
    A['t_previous'] = 0
    A['left_deltas'] = []
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
                if A['t_previous'] > 0:            
                    if s == 'left':
                        A['left_deltas'].append([t,t-A['t_previous']])
                A['t_previous'] = t
                
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


hist_timer = Timer(5)

def graph_thread(A):

    figure('MSE')
    clf()
    indx = int(A['current_img_index'])
    if indx < 0:
        indx = 0
    elif indx >= len(A['left_image']):
        indx = len(A['left_image'])-1
    while True:
        while A['STOP_GRAPH_THREAD'] == False:   
            if len(A['left_image']) < 30:
                #print("""'if len(A['left_image']) < 30:'""")
                pause(0.1)
                continue
            A_len = len(A['left_image'])
            clf()
                       
            steer = array(A['steer'])
            plot(steer[:,0]-steer[0,0],steer[:,1])
            t = A['left_image'][indx][0]
            xlim(t-20-steer[0,0],t-steer[0,0])
            motor = array(A['motor'])
            state = array(A['state'])*10
            plot(motor[:,0]-motor[0,0],motor[:,1])
            plot(state[:,0]-state[0,0],state[:,1])
            ylim(-1,100)
            left_deltas = array(A['left_deltas'])
            states = array(A['state'])[:,1]

            if hist_timer.check():
                figure('left_deltas')
                hist(left_deltas[:,1])
                figure('states')
                hist(states)
                hist_timer.reset()

            figure('MSE')

            #plot(left_deltas[:,0]-left_deltas[0,0],100*left_deltas[:,1])

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
            pause(0.001)
            while A_len == len(A['left_image']):
                indx = int(A['current_img_index'])
                if indx < 0:
                    indx = 0
                elif indx >= len(A['left_image']):
                    indx = len(A['left_image'])-1
                t = A['left_image'][indx][0]
                xlim(t-20-steer[0,0],t-steer[0,0])
                pause(0.2)
        pause(0.2)






def animator_thread(A):
    while True:
        while A['STOP_ANIMATOR_THREAD'] == False:
            if len(A['left_image']) < 30:
                time.sleep(0.1)
                continue

            indx = int(A['current_img_index'])
            if indx < 0:
                indx = 0
            elif indx >= len(A['left_image']):
                indx = len(A['left_image'])-1

            a = A['left_image'][indx]
            img = a[1]#ann.get_aruco_image(a[1],filled=True)
            mi_or_cv2(img,cv=True,delay=30,title='animate')

            A['current_img_index'] += A['d_indx']
            if A['current_img_index'] >= len(A['left_image']):
                A['current_img_index'] = len(A['left_image'])-1
        time.sleep(0.2)

    



def start_graph(A):
    A['STOP_GRAPH_THREAD'] = False
    
def stop_graph(A):
    A['STOP_GRAPH_THREAD'] = True
def start_animation(A):
    A['STOP_ANIMATOR_THREAD'] = False
    
def stop_animation(A):
    A['STOP_ANIMATOR_THREAD'] = True
def stop_loader(A):
    A['STOP_LOADER_THREAD'] = True

def menu(A):
    menu_functions = ['exit_menu','start_animation','start_graph','stop_animation','stop_graph','stop_loader','get_new_A']
    while True:
        for i in range(len(menu_functions)):
            print(d2n(i,') ',menu_functions[i]))
        try:
            choice = input('> ')
            if type(choice) == int:
                if choice == 0:
                    return
                if choice >-1 and choice < len(menu_functions):
                    exec_str = d2n(menu_functions[choice],'(A)')
                    exec(exec_str)
        except:
            pass


A = {}
A = get_new_A(A)
A['d_indx']=4.0


#import kzpy3.teg9.data.bag as bag
#reload(bag)

#bag_folder_path='/media/karlzipser/ExtraDrive1/bair_car_data_10/caffe_direct_Smyth_tape_16Jan17_16h52m24s_Mr_Silver'
#bag_folder_path='/media/karlzipser/ExtraDrive3/from_Mr_Blue/Mr_Blue_Fern_14April2017/processsed/direct_Fern_aruco_1_14Apr17_13h45m40s_Mr_Blue'

bag_folder_path = sys.argv[1]
multi_preprocess(A,bag_folder_path)#,[1,3])

threading.Thread(target=animator_thread,args=[A]).start()
threading.Thread(target=graph_thread,args=[A]).start()

#menu(A)


"""
import kzpy3.teg9.data.bag as bag
#reload(bag)
A = bag.get_new_A()
bag_folder_path='/media/karlzipser/ExtraDrive1/bair_car_data_10/caffe_direct_Smyth_tape_16Jan17_16h52m24s_Mr_Silver'
bag_folder_path='/media/karlzipser/ExtraDrive1/bair_car_data_10/direct_Smyth_tape_25Jan17_19h24m14s_Mr_Yellow'
bag.multi_preprocess(A,bag_folder_path,[1,3])

bag.menu(A)
"""



