"""

ipython --pylab osx kzpy3/scratch/keypress_view_RPi.py; reset

ssh pi@192.168.43.20

"""

from kzpy3.vis import *
import numpy as np
import matplotlib.pyplot as plt
from  matplotlib.animation import FuncAnimation

import paramiko
#paramiko.util.log_to_file(opjD('paramiko.log'))
hup = txt_file_to_list_of_strings('/Users/karlzipser/pw_RPi.txt')# '/Users/karlzipser/pw_MacbookPro.txt')
host = hup[0]
port = 22
transport = paramiko.Transport((host, port))
password = hup[2]
username = hup[1]
transport.connect(username = username, password = password)
sftp = paramiko.SFTPClient.from_transport(transport)



distal_command_file_path = '/home/pi/Desktop/distal_command.txt'
local_command_file_path = '/Users/karlzipser/Desktop/local_command.txt'


fig = plt.figure(figsize=(7,7))
ax = fig.add_axes([0, 0, 1, 1], frameon=False)
ax.set_xlim(0,1), ax.set_xticks([])
ax.set_ylim(0,1), ax.set_yticks([])

last_cmd ='no command yet'

def update(frame_number):
    global last_cmd
    #print last_cmd
    #start_time = time.time()


    try:
        #cmd_lst = txt_file_to_list_of_strings('/Users/karlzipser/Desktop/distal_command.txt')    
        #if cmd_lst[0] != last_cmd:
        #    last_cmd = cmd_lst[0]
        #    print(last_cmd)

        #c_new = os.path.getctime(opjD('image1.jpg'))
        #if c_new == c:
        #   pass #print('waiting...')
        #else:
        #c = c_new
        #unix(d2n('cp ',opjD('image1.jpg '),' /Users/karlzipser/Desktop/RPi_images/',c_new,'.',ctr,'.jpg'),False)
        #ctr += 1
        #print ctr
        img = imread(opjD('image1.jpg'))
        #print(shape(img))
        if shape(img)[2] == 3:
            plt.clf()
            mi(img)
        else:
            print('Empty frame.')


    except KeyboardInterrupt:
        print('Quitting now.')
        print('\nCleaning up.')
        sftp.close()
        transport.close()
        print('Done.')
        sys.exit(1)
    except:
        pass
    

def onclick(event):
    print 'button=%d, x=%d, y=%d, xdata=%f, ydata=%f'%(
        event.button, event.x, event.y, event.xdata, event.ydata)

list_of_strings_to_txt_file(local_command_file_path,['no command yet...'])
sftp.put(local_command_file_path, distal_command_file_path)

def on_key(event):
    list_of_strings_to_txt_file(local_command_file_path,[d2s(event.key,time.time())])
    sftp.put(local_command_file_path, distal_command_file_path)

    if event.key == 'left':
        pass
    	#print('GO LEFT!!')
    elif event.key == 'right':
        pass
    	#print('GO RIGHT!!')
    elif event.key == 'q':
    	plt.clf()
    	plt.close()
    	fig.canvas.mpl_disconnect(cid)
    	print('quit...')
        #print '\033[2J'
    	sys.exit(1)
    else:
        pass
    	#print('you pressed', event.key, event.xdata, event.ydata)


cid = fig.canvas.mpl_connect('key_press_event', on_key)

animation = FuncAnimation(fig, update, interval=10)
plt.show()

print("""
    Start this before starting command listener.
    To make command, put mouse on display window and press command keys...
        [<-] left
        [->] right
        [up arrow] straight
        [space bar] motor
        [q]  quit
""")
a=input('...')
while True:
	pass
#fig.canvas.mpl_disconnect(cid)