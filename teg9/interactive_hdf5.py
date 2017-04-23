from kzpy3.teg7.data.preprocess_bag_data import *
from kzpy3.teg7.data.Bag_File import *
from kzpy3.misc.progress import *
from kzpy3.vis import *


i_variables = ['state','steer','motor','run_','runs','run_labels','meta_path','rgb_1to4_path','B_','left_images','right_images','unsaved_labels']

i_labels = ['out1_in2','direct','home','furtive','play','racing','multicar','campus','night','Smyth','left','notes','local','Tilden','reject_run','reject_intervals','snow','follow','only_states_1_and_6_good']
not_direct_modes = ['out1_in2','left','furtive','play','racing','follow']

i_functions = ['function_close_all_windows','function_set_plot_time_range','function_set_label','function_current_run','function_help','function_set_paths','function_list_runs','function_set_run','function_visualize_run','function_animate','function_run_loop']
for q in i_variables + i_functions + i_labels:
	exec(d2n(q,' = ',"\'",q,"\'")) # I use leading underscore because this facilitates auto completion in ipython

i_label_abbreviations = {out1_in2:'o1i2', direct:'D' ,home:'H',furtive:'Fu',play:'P',racing:'R',multicar:'M',campus:'C',night:'Ni',Smyth:'Smy',left:'Lf',notes:'N',local:'L',Tilden:'T',reject_run:'X',reject_intervals:'Xi',snow:'S',follow:'F',only_states_1_and_6_good:'1_6'}

I = {}






def function_load_hdf5(path):
	F = h5py.File(path)
	labels = {}
	Lb = F['labels']
	for k in Lb.keys():
		if Lb[k][0]:
			labels[k] = True
		else:
			labels[k] = False
	S = F['segments']
	return (labels,S)




def load_animate_hdf5(path,start_at_time=0):
	start_at(start_at_time)
	l,s=function_load_hdf5(path)
	img = False
	for h in range(len(s)):
		if type(img) != bool:
			img *= 0
			img += 128
			mi_or_cv2(img)
		pause(0.5)
		n = str(h)
		for i in range(len(s[n]['left'])):
			img = s[n]['left'][i]
			#print s[n][state][i]
			bar_color = [0,0,0]
			
			if s[n][state][i] == 1:
				bar_color = [0,0,255]
			elif s[n][state][i] == 6:
				bar_color = [255,0,0]
			elif s[n][state][i] == 5:
				bar_color = [255,255,0]
			elif s[n][state][i] == 7:
				bar_color = [255,0,255]
			else:
				bar_color = [0,0,0]
			if i < 2:
				smooth_steer = s[n][steer][i]
			else:
				smooth_steer = (s[n][steer][i] + 0.5*s[n][steer][i-1] + 0.25*s[n][steer][i-2])/1.75
			#print smooth_steer
			apply_rect_to_img(img,smooth_steer,0,99,bar_color,bar_color,0.9,0.1,center=True,reverse=True,horizontal=True)
			apply_rect_to_img(img,s[n][motor][i],0,99,bar_color,bar_color,0.9,0.1,center=True,reverse=True,horizontal=False)
			mi_or_cv2(img)
A5 = load_animate_hdf5


"""
N = 30
label_segment_dic = {}
for i in range(len(run_codes)):
	run_name = run_codes[i].replace('.pkl','')
	hdf5_path = opjD('bair_car_data/hdf5')
	labels,segments = function_load_hdf5(opj(hdf5_path,'runs',run_name+'.hdf5')))
	label_segment_dic[run_name] = (labels,segments)

run_name = high_steer[0][0]
seg = high_steer[0][0]
elm = high_steer[0][0]
labels,segments = label_segment_dic[run_name]
label_dic = make_label_dic(labels[seg])
segment = segments[seg][elm:(elm+N)]
"""






Segment_Data = {}
hdf5_runs_path = opjD('bair_car_data/hdf5/runs')
hdf5_segment_metadata_path = opjD('bair_car_data/hdf5/segment_metadata')



def load_hdf5_steer_hist(path,dst_path):
	print path
	unix('mkdir -p '+dst_path)
	low_steer = []
	high_steer = []
	l,s=function_load_hdf5(path)
	pb = ProgressBar(len(s))
	for h in range(len(s)):
		pb.animate(h)
		#print h
		n = str(h)
		for i in range(len(s[n]['left'])):
			if i < 2:
				smooth_steer = s[n][steer][i]
			else:
				smooth_steer = (s[n][steer][i] + 0.5*s[n][steer][i-1] + 0.25*s[n][steer][i-2])/1.75
			if smooth_steer < 43 or smooth_steer > 55:
				high_steer.append([h,i,int(round(smooth_steer))])
			else:
				low_steer.append([h,i,int(round(smooth_steer))])
	pb.animate(h)
	assert(len(high_steer)>0)
	assert(len(low_steer)>0)

	save_obj(high_steer,opj(dst_path,fname(path).replace('hdf5','high_steer.pkl')))
	save_obj(low_steer,opj(dst_path,fname(path).replace('hdf5','low_steer.pkl')))


def load_run_codes():
	run_codes = load_obj(opj(hdf5_segment_metadata_path,'run_codes.pkl'))
	Segment_Data['run_codes'] = run_codes
	Segment_Data['runs'] = {}
	for n in run_codes.keys():
		run_name = run_codes[n]
		Segment_Data['runs'][run_name] = {}
		Segment_Data['runs'][run_name]['run_code'] = n

def run_into_Segment_Data(run_code_num):
	run_name = Segment_Data['run_codes'][run_code_num]
	assert(run_name in Segment_Data['runs'])
	labels,segments = function_load_hdf5(opj(hdf5_runs_path,run_name+'.hdf5'))
	high_steer = load_obj(opj(hdf5_segment_metadata_path,run_name+'.high_steer.pkl'))
	low_steer = load_obj(opj(hdf5_segment_metadata_path,run_name+'.low_steer.pkl'))
	state_hist_list = load_obj(opj(hdf5_segment_metadata_path,run_name+'.state_hist_list.pkl'))
	Segment_Data['runs'][run_name]['labels'] = labels
	Segment_Data['runs'][run_name]['segments'] = segments
	Segment_Data['runs'][run_name]['high_steer'] = high_steer
	Segment_Data['runs'][run_name]['low_steer'] = low_steer
	Segment_Data['runs'][run_name]['state_hist_list'] = state_hist_list
	return run_name


def animate_segment(run_code_num,seg_num):
	run_name = Segment_Data['run_codes'][run_code_num]
	left_images = Segment_Data['runs'][run_name]['segments'][str(seg_num)]['left'][:]
	steers = Segment_Data['runs'][run_name]['segments'][str(seg_num)]['steer'][:]
	motors = Segment_Data['runs'][run_name]['segments'][str(seg_num)]['motor'][:]
	states = Segment_Data['runs'][run_name]['segments'][str(seg_num)]['state'][:]
	for i in range(shape(left_images)[0]):
		bar_color = [0,0,0]
		if states[i] == 1:
			bar_color = [0,0,255]
		elif states[i] == 6:
			bar_color = [255,0,0]
		elif states[i] == 5:
			bar_color = [255,255,0]
		elif states[i] == 7:
			bar_color = [255,0,255]
		else:
			bar_color = [0,0,0]
		if i < 2:
			smooth_steer = steers[i]
		else:
			smooth_steer = (steers[i] + 0.5*steers[i-1] + 0.25*steers[i-2])/1.75
		img = left_images[i,:,:,:]
		apply_rect_to_img(img,smooth_steer,0,99,bar_color,bar_color,0.9,0.1,center=True,reverse=True,horizontal=True)
		apply_rect_to_img(img,motors[i],0,99,bar_color,bar_color,0.9,0.1,center=True,reverse=True,horizontal=False)
		mi_or_cv2(img)


def get_data(run_code_num,seg_num,offset,slen,img_offset,img_slen,ignore=[left,out1_in2],require_one=[],smooth_steer=True):
	run_name = Segment_Data['run_codes'][run_code_num]
	labels = Segment_Data['runs'][run_name]['labels']
	for ig in ignore:
		if labels[ig]:
			return None
	require_one_okay = True
	if len(require_one) > 0:
		require_one_okay = False
		for ro in require_one:
			if labels[ro]:
				require_one_okay = True
	if not require_one_okay:		
		return None
	a = offset
	b = offset + slen
	ia = img_offset
	ib = img_offset + img_slen
	seg_num_str = str(seg_num)
	if not (b-a <= len(Segment_Data['runs'][run_name]['segments'][seg_num_str]['steer'][:])):
		return None
	if not (ib-ia <= len(Segment_Data['runs'][run_name]['segments'][seg_num_str]['steer'][:])):
		return None
	steers = Segment_Data['runs'][run_name]['segments'][seg_num_str]['steer'][a:b]
	if len(steers)!=slen:
		return None
	motors = Segment_Data['runs'][run_name]['segments'][seg_num_str]['motor'][a:b]
	if len(motors)!=slen:
		return None
	states = Segment_Data['runs'][run_name]['segments'][str(seg_num)]['state'][a:b]
	if len(states)!=slen:
		return None
	left_images = Segment_Data['runs'][run_name]['segments'][seg_num_str]['left'][ia:ib]
	right_images = Segment_Data['runs'][run_name]['segments'][seg_num_str]['right'][ia:ib]
	if smooth_steer:
		for i in range(2,len(steers)):
			steers[i] = (3/6.)*steers[i] + (2/6.)*steers[i-1] + (1/6.)*steers[i-2]
	data = {}
	data['name'] = run_name
	data['steer'] = steers
	data['motor']  = motors
	data['states'] = states
	data['left'] = left_images
	data['right'] = left_images
	data['labels'] = labels
	return data

#########################################


load_run_codes()

if True:
	pb = ProgressBar(len(Segment_Data['run_codes']))
	ctr = 0
	for n in Segment_Data['run_codes'].keys():
		ctr+=1
		pb.animate(ctr)
		run_into_Segment_Data(n)
	pb.animate(len(Segment_Data['run_codes']))


if False:
	data=get_data(0,0,100,100,100,102)
	figure('steer')
	clf()
	plot(data['steer']); pause(0.001)
	mi_or_cv2_animate(data['left'],img_title=data['name'])
	data['name']
	n = 100
	data=get_data(0,0,n,10,n,2);
	mi_or_cv2_animate(data['left'])
	print data['steer']
	print data['motor']





print('loading low_steer')
low_steer = load_obj(opjD('bair_car_data/hdf5/segment_metadata/low_steer'))
print('loading high_steer')
high_steer = load_obj(opjD('bair_car_data/hdf5/segment_metadata/high_steer'))

len_high_steer = len(high_steer)
len_low_steer = len(low_steer)
ctr_low = -1
ctr_high = -1






##################################################
import caffe
USE_GPU = True
if USE_GPU:
	caffe.set_device(1)
	caffe.set_mode_gpu()

def print_solver(solver):
	print("")
	for l in [(k, v[0].data.shape) for k, v in solver.net.params.items()]:
		print(l)
	print("")
	for l in [(k, v.data.shape) for k, v in solver.net.blobs.items()]:
		if 'split' not in l[0]:
			print(l)

def setup_solver(solver_file_path):
	solver = caffe.SGDSolver(solver_file_path)
	print_solver(solver)
	return solver

def _array_to_int_list(a):
	l = []
	for d in a:
		l.append(int(d*100))
	return l





def val_to_category(value,cat_min,cat_max,num_bins):
	s = zeros(num_bins)
	i = int((value / (1.*cat_max-cat_min)) * num_bins)
	if i < 0:
		i = 0
	if i > num_bins-1:
		i = num_bins-1
	return i



solver = setup_solver(opjh('kzpy3/caf7/z2_color/solver.prototxt'))
weights_file_path = opjh('Desktop/z2_color/z2_color_scratch_iter_27900000.caffemodel')
solver.net.copy_from(weights_file_path)
cprint('Loaded weights from '+weights_file_path)

##############################
#
N_FRAMES = 2
N_STEPS = 10
print_timer = Timer(5)
loss10000 = []
loss = []
rate_timer_interval = 10.
rate_timer = Timer(rate_timer_interval)
rate_ctr = 0
ignore=[reject_run,left,out1_in2]
require_one=[]
while True:

	if ctr_low >= len_low_steer:
		ctr_low = -1
	if ctr_high >= len_high_steer:
		ctr_high = -1
	if ctr_low == -1:
		random.shuffle(low_steer)
		ctr_low = 0
	if ctr_high == -1:
		random.shuffle(high_steer)
		ctr_high = 0
		
	if random.random() > 0.5:
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
		continue

	#####################################
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
	################################

	solver.step(1)

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
		cprint(_array_to_int_list(solver.net.blobs['steer_motor_target_data'].data[0,:][:]),'green','on_red')
		cprint(_array_to_int_list(solver.net.blobs['ip2'].data[0,:][:]),'red','on_green')
		figure('steer')
		clf()
		xlen = len(solver.net.blobs['ip2'].data[0,:][:])/2-1
		ylim(-5,105);xlim(0,xlen)
		t = solver.net.blobs['steer_motor_target_data'].data[0,:]*100.
		o = solver.net.blobs['ip2'].data[0,:]*100.
		plot(zeros(xlen+1)+49,'k');plot(o,'g'); plot(t,'r'); plt.title(data['name']);pause(0.001)
		mi_or_cv2_animate(data['left'])
		print_timer.reset()

