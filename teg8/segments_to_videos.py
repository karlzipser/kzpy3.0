from kzpy3.teg8.train_with_hdf5_utils import *




def frames_to_video_with_ffmpeg(input_dir,output_path,img_range=()):
	if input_dir[-1] == '/':
		input_dir = input_dir[:-1] # the trailing / messes up the name.
	_,fnames = dir_as_dic_and_list(input_dir)
	frames_folder = input_dir.split('/')[-1]
	unix('mkdir -p '+'/'.join(output_path.split('/')[:-1]))
	unix_str = ' -i '+input_dir+'/%d.png -pix_fmt yuv420p -r 30 -b:v 14000k '+output_path
	success = False
	try:
		print('Trying avconv.')
		unix('avconv'+unix_str)
		success = True
	except Exception as e:
		print "'avconv did not work.' ***************************************"
		print e.message, e.args
		print "***************************************"
	if not success:
		try:
			print('Trying ffmpeg.')
			unix('ffmpeg'+unix_str)
			success = True
		except Exception as e:
			print "'ffmeg did not work.' ***************************************"
			print e.message, e.args
			print "***************************************"
	if success:
		print('frames_to_video_with_ffmpeg() had success with ' + frames_folder)


run_frames_folders = sgg(opjD('bair_car_data/run_frames/*'))
for rff in run_frames_folders:
	print('\t'+rff)
	runs = sgg(opj(rff,'*'))
	for r in runs:
		print('\t\t'+r)
		print('\t\t'+r.replace('/run_frames/','/run_videos/')+'.mp4')
		input_dir = r
		output_path = r.replace('/run_frames/','/run_videos/')+'.mp4'
		frames_to_video_with_ffmpeg(input_dir,output_path)




def animate_segment(run_code_num,seg_num,do_save=False):
	"""
	def animate_segment(run_code_num,seg_num):
	Animate a data segment
	"""
	run_name = Segment_Data['run_codes'][run_code_num]
	left_images = Segment_Data['runs'][run_name]['segments'][str(seg_num)]['left'][:]
	steers = Segment_Data['runs'][run_name]['segments'][str(seg_num)]['steer'][:]
	motors = Segment_Data['runs'][run_name]['segments'][str(seg_num)]['motor'][:]
	states = Segment_Data['runs'][run_name]['segments'][str(seg_num)]['state'][:]
	created_save_folder = False
		 
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
			smooth_steer = steers[i] #(steers[i] + 0.5*steers[i-1] + 0.25*steers[i-2])/1.75
		img = left_images[i,:,:,:]
		apply_rect_to_img(img,smooth_steer,0,99,bar_color,bar_color,0.9,0.1,center=True,reverse=True,horizontal=True)
		apply_rect_to_img(img,motors[i],0,99,bar_color,bar_color,0.9,0.1,center=True,reverse=True,horizontal=False)
		delay = 30
		if do_save:
			delay = 1
			if created_save_folder == False:
				unix('mkdir -p '+opjD('bair_car_data/run_frames_teal',run_name+'_'+str(s)),False)
			imsave(opjD('bair_car_data/run_frames_teal',run_name+'_'+str(s),str(i)+'.png'),img)
		mi_or_cv2(img,title='snow')

if False:
	if len(Segment_Data) < 2:
		load_run_codes()
		pb = ProgressBar(len(Segment_Data['run_codes']))
		ctr = 0
		print("doing run_into_Segment_Data...")
		for n in Segment_Data['run_codes'].keys():
			ctr+=1
			pb.animate(ctr)
			run_into_Segment_Data(n)
		pb.animate(len(Segment_Data['run_codes']))

if False:
	completed_runs = sgg(opjD('bair_car_data/run_frames/*'))
	for i in range(len(completed_runs)):
		completed_runs[i] = fname(completed_runs[i])

if False:
	long_state_6_seg = 0
	for rc in Segment_Data['run_codes']:
		r = Segment_Data['run_codes'][rc]
		if False: #r in completed_runs:
			cprint('*** '+r+' done ***')
			continue
		if not Segment_Data['runs'][r]['labels']['flip']:

			for s in Segment_Data['runs'][r]['segments']:
				state_6_ctr = 0
				states = Segment_Data['runs'][r]['segments'][s]['state'][:]
				for st in states:
					if st == 6:
						state_6_ctr += 1
					else:
						break
				if state_6_ctr > 60*30:
					long_state_6_seg += 1
					print (r,long_state_6_seg,state_6_ctr)
					animate_segment(rc,s,do_save=True)


if False:
	long_state_6_seg = 0
	for rc in Segment_Data['run_codes']:
		r = Segment_Data['run_codes'][rc]
		if False: #r in completed_runs:
			cprint('*** '+r+' done ***')
			continue
		if not Segment_Data['runs'][r]['labels']['flip']:
			for s in Segment_Data['runs'][r]['segments']:
				state_6_ctr = 0
				low_motor_ctr = 0
				states = Segment_Data['runs'][r]['segments'][s]['state'][:]
				motors = Segment_Data['runs'][r]['segments'][s]['motor'][:]
				for i in range(len(states)):
					st = states[i]
					mo = motors[i]
					if st == 6:
						state_6_ctr += 1
					else:
						break
					if i > 30 and mo < 55:
						low_motor_ctr += 1
				if state_6_ctr > 10*30 and low_motor_ctr > 10:
					long_state_6_seg += 1
					print (r,long_state_6_seg,state_6_ctr)
					animate_segment(rc,s,do_save=True)

if False:
	long_state_6_seg = 0
	for rc in Segment_Data['run_codes']:
		r = Segment_Data['run_codes'][rc]
		if not 'Teal' in r:
			continue
		if not 'single' in r:
			continue
		if False: #r in completed_runs:
			cprint('*** '+r+' done ***')
			continue
		if not Segment_Data['runs'][r]['labels']['flip']:
			for s in Segment_Data['runs'][r]['segments']:
				state_6_ctr = 0
				low_motor_ctr = 0
				states = Segment_Data['runs'][r]['segments'][s]['state'][:]
				motors = Segment_Data['runs'][r]['segments'][s]['motor'][:]
				for i in range(len(states)):
					st = states[i]
					mo = motors[i]
					if st == 6:
						state_6_ctr += 1
					else:
						break
					if i > 30 and mo < 55:
						low_motor_ctr += 1
				if state_6_ctr > 10*30: # and low_motor_ctr > 10:
					long_state_6_seg += 1
					print (r,long_state_6_seg,state_6_ctr)
					animate_segment(rc,s,do_save=True)


