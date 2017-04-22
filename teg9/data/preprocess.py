from kzpy3.vis import *
from kzpy3.teg7.data.preprocess_bag_data import *
from kzpy3.teg7.data.preprocess_Bag_Folders import *
from kzpy3.teg7.data.Bag_File import *
import shutil

# rsync -rav /home/karlzipser/Desktop/bair_car_data/ /media/karlzipser/bair_car_data_10/bair_car_data/

tb = '\t'

if False:
	backup_locations = []
	for i in [10]:#9,10]:
		backup_locations.append(opj('/media',username,'bair_car_data_'+str(i)))

#bag_folders_src_location = opj('/media',username,'rosbags')
bag_folders_src_location = '/media/karlzipser/ExtraDrive3/from_Mr_Blue/Mr_Blue_Fern_14April2017'
bag_folders_src = opj(bag_folders_src_location,'new' )
bag_folders_dst_rgb1to4_path = opjD('bair_car_data/rgb_1to4')
bag_folders_dst_meta_path = opjD('bair_car_data/meta_states_1_5_6_7_good')

runs = sgg(opj(bag_folders_src,'*'))

cprint('Preliminary check of '+bag_folders_src)
cprint("	checking bag file sizes and run durations")
for r in runs:
	bags = sgg(opj(r,'*.bag'))
	cprint(d2s(tb,fname(r),len(bags)))
	mtimes = []
	for b in bags:
		bag_size = os.path.getsize(b)
		mtimes.append(os.path.getmtime(b))
		#cprint(d2s(tb,tb,fname(b),bag_size))
		if bag_size < 0.99 * 1074813904:
			cprint(d2s('Bagfile',b,'has size',bag_size,'which is below full size.'),'red')
			assert(False)
	mtimes = sorted(mtimes)
	run_duration = mtimes[-1]-mtimes[0]
	print run_duration
	assert(run_duration/60./60. < 3.)
	cprint(d2s(r,'is okay'))



for r in runs:
	preprocess_bag_data(r)

bag_folders_transfer_meta(bag_folders_src,bag_folders_dst_meta_path)

bag_folders_save_images(bag_folders_src,bag_folders_dst_rgb1to4_path)

if False:
	preprocess_Bag_Folders(bag_folders_dst_meta_path,bag_folders_dst_rgb1to4_path,NUM_STATE_ONE_STEPS=30,graphics=False,accepted_states=[1,3,5,6,7])
if True:
	preprocess_Bag_Folders(bag_folders_dst_meta_path,bag_folders_dst_rgb1to4_path,NUM_STATE_ONE_STEPS=30,graphics=False,accepted_states=[1,3,5,6,7],pkl_name='Bag_Folder_60_state_one_steps.pkl')

"""
for bkp in backup_locations:
	for r in runs:
		unix(d2s('mkdir -p',(opj(bkp,'bair_car_data',fname(r)))))
		unix(d2s('scp -r',r,opj(bkp,'bair_car_data')))
		#shutil.copytree(r,opj(bkp,'bair_car_data',fname(r)))


preprocess_Bag_Folders(opjD('bair_car_data_new/meta_temp_location'),opjD('bair_car_data_new/rgb_1to4'),NUM_STATE_ONE_STEPS=30,graphics=False,accepted_states=[1,3,5,6,7])

"""
os.rename(bag_folders_src,opj(bag_folders_src_location,'processsed'))

