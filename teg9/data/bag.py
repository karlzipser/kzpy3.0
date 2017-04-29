
from kzpy3.vis import *
import rospy
import rosbag
import cv2
from cv_bridge import CvBridge,CvBridgeError
import threading
import kzpy3.data_analysis.aruco_annotator as ann

bridge = CvBridge()

image_topics = ['left_image','right_image']
single_value_topics = ['steer','state','motor','encoder','GPS2_lat']
vector3_topics = ['acc','gyro','gyro_heading']#,'gps']
camera_sides = ['left','right']



def multi_preprocess(A,bag_folder_path,bagfile_range=[]):
    bag_files = sorted(gg(opj(bag_folder_path,'*.bag')))
    if len(bagfile_range) > 0:
        bag_files = bag_files[bagfile_range[0]:bagfile_range[1]]
    threading.Thread(target=multi_preprocess_thread,args=[A,bag_files]).start()


def multi_preprocess_thread(A,bag_files):
    for b in bag_files:
        if A['STOP_LOADER_THREAD']:
            A['STOP_LOADER_THREAD'] = False
            print('Stopping multi_preprocess_thread.')
            break
        preprocess(A,b)


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


def graph_thread(A):
    figure('MSE')
    indx = int(A['current_img_index'])
    while True:
        while A['STOP_GRAPH_THREAD'] == False:   
            if len(A['left_image']) < 3*30:
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
            plot(motor[:,0]-motor[0,0],motor[:,1])
            acc_x = array(A['acc_x'])
            acc_y = array(A['acc_y'])
            acc_z = array(A['acc_z'])
            plot(acc_x[:,0]-acc_x[0,0],acc_x[:,1])
            plot(acc_y[:,0]-acc_y[0,0],acc_y[:,1])
            plot(acc_z[:,0]-acc_z[0,0],acc_z[:,1])
            ylim(-15,105)
            if False: #hist_timer.check():
                figure('left_deltas')
                left_deltas = array(A['left_deltas'])
                hist(left_deltas[:,1])
                hist_timer.reset()
                figure('MSE')

            pause(0.001)
            while A_len == len(A['left_image']):
                indx = int(A['current_img_index'])
                t = A['left_image'][indx][0]
                xlim(t-20-steer[0,0],t-steer[0,0])
                pause(0.2)
        pause(0.2)


def animator_thread(A):
    #lock = threading.Lock()
    while True:
        while A['STOP_ANIMATOR_THREAD'] == False:
            #lock.acquire()
            if len(A['left_image']) < 2*30:
                time.sleep(0.1)
                continue
            A['current_img_index'] += A['d_indx']
            if A['current_img_index'] >= len(A['left_image']):
                A['current_img_index'] = len(A['left_image'])-1
            elif A['current_img_index'] < 0:
                A['current_img_index'] = 0
            indx = int(A['current_img_index'])
            img = A['left_image'][indx]
            #lock.release()
            #aruco_img = ann.get_aruco_image(img[1],filled=False,color=(255,0,0))
            mi_or_cv2(img[1],cv=True,delay=30,title='animate')
        time.sleep(0.2)

def d_index_up(A):
    lock = threading.Lock()
    lock.acquire()
    A['d_indx'] += 0.2
    lock.release()
    print(d2s("A['d_indx'] =",A['d_indx']))
def d_index_down(A):
    lock = threading.Lock()
    lock.acquire()
    A['d_indx'] -= 0.2
    lock.release()
    print(d2s("A['d_indx'] =",A['d_indx']))

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


def get_new_A(_=None):
    A = {}
    A['STOP_LOADER_THREAD'] = False
    A['STOP_ANIMATOR_THREAD'] = False
    A['STOP_GRAPH_THREAD'] = False
    A['d_indx'] = 1.0
    A['current_img_index'] = -A['d_indx']
    A['t_previous'] = 0
    A['left_deltas'] = []
    return A

def menu(A):
    menu_functions = ['exit_menu','start_animation','start_graph','stop_animation','stop_graph','stop_loader','get_new_A','d_index_up','d_index_down']
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



if __name__ == '__main__':
    bag_folder_path = sys.argv[1]
    bagfile_range=[int(sys.argv[2]),int(sys.argv[3])]
    hist_timer = Timer(10)
    A = {}
    A = get_new_A(A)
    multi_preprocess(A,bag_folder_path,bagfile_range)
    threading.Thread(target=animator_thread,args=[A]).start()
    threading.Thread(target=graph_thread,args=[A]).start()
    menu(A)





