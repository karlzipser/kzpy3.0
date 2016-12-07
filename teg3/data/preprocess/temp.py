from kzpy3.teg3.data.preprocess.preprocess_bag_data import *

bfp = '/media/karlzipser/rosbags/direct_home_06Dec16_16h01m42s_Mr_Blue'

preprocess_bag_data(bfp,[]);

L=load_obj('/home/karlzipser/Desktop/direct_home_06Dec16_16h01m42s_Mr_Blue/.preprocessed2/left_image_bound_to_data.pkl')

if False:
	ts = sorted(L.keys())
	dts = [0]
	for i in range(1,len(ts)):
		dts.append(ts[i] - ts[i-1])
	figure(1)
	plot(ts,dts)
	figure(2)
	hist(dts)

import kzpy3.teg3.data.access.Bag_Folder as Bag_Folder
import kzpy3.teg3.data.access.Bag_File as Bag_File

BF = Bag_Folder.init('/home/karlzipser/Desktop/direct_home_06Dec16_16h01m42s_Mr_Blue',
	preprocessed_dir='.preprocessed2',
	left_image_bound_to_data_name='left_image_bound_to_data.pkl',
	NUM_STATE_ONE_STEPS=10)



#plot(BF['data']['raw_timestamps'],BF['data']['state_one_steps'],'bo-')
plot(BF['data']['good_start_timestamps'],zeros(len(BF['data']['good_start_timestamps']))+100,'go')
#plot(BF['data']['raw_timestamps'],2000*np.array(BF['data']['raw_timestamp_deltas']),'r')
#plot(BF['data']['raw_timestamps'],100*BF['data']['encoder'],'r')
#plot(BF['data']['raw_timestamps'],100*BF['data']['state'],'r')
#plot(BF['data']['raw_timestamps'],100*BF['data']['acc_z'],'r')

bf = BF['bag_file_num_dic'][15]  
bid = Bag_File.load_images(bf)
bag_left_timestamps = sorted(bid['left'].keys())
plot(bag_left_timestamps,2000+np.zeros(bag_left_timestamps),'go')

good_bag_timestamps = list(set(BF['data']['good_start_timestamps']) & set(bag_left_timestamps))
plot(good_bag_timestamps,2000+np.zeros(len(good_bag_timestamps)),'ro')


ts = sorted(bid['left'].keys())
for t in ts:
	mi(bid['left'][t],'left')
	#mi(bid['right'][BF['left_image_bound_to_data'][t]['right_image']],'right')
	plt.pause(0.0001)

plot(BF['data']['steer'],BF['data']['gyro_y'],'o')

"""
L=load_obj('/media/karlzipser/bair_car_data_7/rosbags14/direct_racing_Tilden_27Nov16_10h41m05s_Mr_Blue/.preprocessed2/left_image_bound_to_data.pkl')
ts = sorted(L.keys())
dts = [0]
for i in range(1,len(ts)):
	dts.append(ts[i] - ts[i-1])
figure(3)
plot(ts,dts)
figure(4)
hist(dts)

"""

pass



if False:
	bf = random.choice(BF['bag_file_num_dic'])
	bid = Bag_File.load_images(bf)
	bag_left_timestamps = sorted(bid['left'].keys())
	plot(bag_left_timestamps,2000+np.zeros(len(bag_left_timestamps)),'go')

	good_bag_timestamps = list(set(BF['data']['good_start_timestamps']) & set(bag_left_timestamps))
	plot(good_bag_timestamps,2000+np.zeros(len(good_bag_timestamps)),'ro')

	ts = sorted(bid['left'].keys())
	for t in ts:
		mi(bid['left'][t],'left')
		#mi(bid['right'][BF['left_image_bound_to_data'][t]['right_image']],'right')
		plt.pause(0.0001)


