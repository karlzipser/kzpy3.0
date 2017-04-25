'''aruco_steer,aruco_motor,aruco_only = aruco_code.do_aruco(left_list[-1],steer,motor)'''
#from Map import Map
from Video_Marker import Video_Marker
import cv2
marker = Video_Marker()




def do_aruco(cv_image,steering_cmd,motor_cmd,ar_params, crop=False):
    
    follow_behaviour = True
    
    safe_motor, safe_steer, evasion_needed = marker.add_evasion_behaviour(cv_image,steering_cmd,motor_cmd,ar_params,follow_behaviour,crop) 

    #cv2.imshow('test',cv_image)
    #cv2.moveWindow('test',0,500)

    #print("Hey: " + str(safe_motor) + " , " + str(safe_steer))
    
    return safe_steer,safe_motor,evasion_needed

if __name__ == '__main__':
    '''
    This code is purely to check if the code works standalone
    '''
    capture_device = cv2.VideoCapture(2)
    capture_device.set(cv2.CAP_PROP_FRAME_WIDTH, 1344)
    capture_device.set(cv2.CAP_PROP_FRAME_HEIGHT, 376)
    paused_video = False
    crop = True
    while True:
            if not paused_video:
                ret, image = capture_device.read()
                if image is None:
                    print("Error reading image! Wrong number of camera?")
                     
                do_aruco(image,1.0,1.0,None,crop)
                cv2.imshow('frame',image)
                key = cv2.waitKey(1000/30) & 0xFF
                
                if key == ord('q'):
                    break
                if key == ord(' '):
                    paused_video = not paused_video
                