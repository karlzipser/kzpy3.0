from kzpy3.teg9.train_with_hdf5_utils import *

"""
*** kzpy3/teg7/train_with_hdf5.py ***

This script trains a model on hdf5 model car data. It expects to find the hdf5 folder here:

	~/Desktop/bair_car_data/hdf5

It also needs:

	~/kzpy3/caf7/z2_color/solver.prototxt
	~/kzpy3/caf7/z2_color/train_val.prototxt

and expects to find a folder:

	~/Desktop/z2_color

for saving weights.

Can run from python or command line:

	$ python kzpy3/teg7/train_with_hdf5.py

The script regularly displays left camera data, steer data (red) and
network steer output (green) [49=straight, 0=right, 99=left], and loss
averaged over 10000 iterations.

"""


USE_GPU = True
if USE_GPU:
	caffe.set_device(0)
	caffe.set_mode_gpu()

if True:
	load_run_codes()

if True:
	pb = ProgressBar(len(Segment_Data['run_codes']))
	ctr = 0
	print("doing run_into_Segment_Data...")
	for n in Segment_Data['run_codes'].keys():
		ctr+=1
		pb.animate(ctr)
		run_into_Segment_Data(n)
	pb.animate(len(Segment_Data['run_codes']))

if True:
	print('loading low_steer... (takes awhile)')
	low_steer = load_obj(opj(bair_car_data_path,'hdf5/segment_metadata/low_steer'))
	print('\nloading high_steer... (takes awhile)')
	high_steer = load_obj(opj(bair_car_data_path,'hdf5/segment_metadata/high_steer'))

	len_high_steer = len(high_steer)
	len_low_steer = len(low_steer)
	ctr_low = -1 # These counter keep track of position in segment lists, and when to reshuffle.
	ctr_high = -1

if True:
	solver = setup_solver(opjh('kzpy3/caf7/z2_color/solver.prototxt'))
	weights_file_path = opjh('kzpy3/caf5/z2_color/z2_color.caffemodel')
	solver.net.copy_from(weights_file_path)
	cprint('Loaded weights from '+weights_file_path)
	N_FRAMES = 2 # how many timesteps with images.
	N_STEPS = 10 # how many timestamps with non-image data
	ignore=['mostly_caffe',reject_run,left,out1_in2] # runs with these labels are ignored
	require_one=['mostly_human'] # at least one of this type of run lable is required
	print_timer = Timer(5)
	loss10000 = []
	loss = []
	rate_timer_interval = 10.
	rate_timer = Timer(rate_timer_interval)
	rate_ctr = 0
	figure('steer',figsize=(3,2))
	figure('loss',figsize=(3,2))
	while True:
		if ctr_low >= len_low_steer:
			ctr_low = -1
		if ctr_high >= len_high_steer:
			ctr_high = -1
		if ctr_low == -1:
			random.shuffle(low_steer) # shuffle data before using (again)
			ctr_low = 0
		if ctr_high == -1:
			random.shuffle(high_steer)
			ctr_high = 0
			
		if random.random() <= len(low_steer)/(len(high_steer)+len(low_steer)+0.0): # with some probability choose a low_steer element
			choice = low_steer[ctr_low]
			ctr_low += 1
		else:
			choice = high_steer[ctr_high]
			ctr_high += 1
		run_code = choice[3]
		seg_num = choice[0]
		offset = choice[1]
		data = get_data(run_code,seg_num,offset,N_STEPS,offset+0,N_FRAMES,ignore=ignore,require_one=require_one)
		if data == None:
			#print('if data == None:')
			continue
		if data['states'][0] != 1: ##################################################### !!!!!!!!!!!!
			continue
			print data['states'][0]
		############## load data into solver #####################
		#
		ctr = 0
		for c in range(3):
			for camera in ('left','right'):
				for t in range(N_FRAMES):
					solver.net.blobs['ZED_data_pool2'].data[0,ctr,:,:] = data[camera][t][:,:,c]
					ctr += 1
		Racing = 0
		Caf = 0
		Follow = 0
		Direct = 0
		Play = 0
		Furtive = 0
		if data['labels']['racing']:
			Racing = 1.0
		if data['states'][0] == 6:
			Caf = 1.0
		if data['labels']['follow']:
			Follow = 1.0
		if data['labels']['direct']:
			Direct = 1.0
		if data['labels']['play']:
			Play = 1.0
		if data['labels']['furtive']:
			Furtive = 1.0
		solver.net.blobs['metadata'].data[0,0,:,:] = Racing
		solver.net.blobs['metadata'].data[0,1,:,:] = Caf
		solver.net.blobs['metadata'].data[0,2,:,:] = Follow
		solver.net.blobs['metadata'].data[0,3,:,:] = Direct
		solver.net.blobs['metadata'].data[0,4,:,:] = Play
		solver.net.blobs['metadata'].data[0,5,:,:] = Furtive
		solver.net.blobs['steer_motor_target_data'].data[0,:N_STEPS] = data['steer'][-N_STEPS:]/99.
		solver.net.blobs['steer_motor_target_data'].data[0,N_STEPS:] = data['motor'][-N_STEPS:]/99.
		#
		##########################################################
		solver.step(1) # The training step. Everything below is for display.
		rate_ctr += 1
		if rate_timer.check():
			print(d2s('rate =',dp(rate_ctr/rate_timer_interval,2),'Hz'))
			rate_timer.reset()
			rate_ctr = 0
		a = solver.net.blobs['steer_motor_target_data'].data[0,:] - solver.net.blobs['ip2'].data[0,:]
		loss.append(np.sqrt(a * a).mean())
		if len(loss) >= 10000:
			loss10000.append(array(loss[-10000:]).mean())
			loss = []
			figure('loss');clf()
			lm = min(len(loss10000),100)
			plot(loss10000[-lm:])
			print(d2s('loss10000 =',loss10000[-1]))
		if print_timer.check():
			print(solver.net.blobs['metadata'].data[0,:,5,5])
			cprint(array_to_int_list(solver.net.blobs['steer_motor_target_data'].data[0,:][:]),'green','on_red')
			cprint(array_to_int_list(solver.net.blobs['ip2'].data[0,:][:]),'red','on_green')
			figure('steer')
			clf()
			xlen = len(solver.net.blobs['ip2'].data[0,:][:])/2-1
			ylim(-5,105);xlim(0,xlen)
			t = solver.net.blobs['steer_motor_target_data'].data[0,:]*100.
			o = solver.net.blobs['ip2'].data[0,:]*100.
			plot(zeros(xlen+1)+49,'k');plot(o,'g'); plot(t,'r'); plt.title(data['name']);pause(0.001)
			mi_or_cv2_animate(data['left'])
			print_timer.reset()

