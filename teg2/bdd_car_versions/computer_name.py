# This should be place in ~ (home dir).
# This is used to specifiy caffe mode and data file name information

from kzpy3.utils import time_str

computer_name = 'Mr_Orange'
Direct = 1.
Follow = 0.
Play = 0.
Furtive = 0.
Caf = 1.0
Location = 'local'
GPS2_lat_orig = 37.881404 #-999.99
GPS2_long_orig = -122.2743327 #-999.99
GPS2_radius = 0.0004

if Direct == 1:
	task = 'direct'
elif Play == 1:
	task = 'play'
elif Follow == 1:
	task = 'follow'
elif Furtive == 1:
	task = 'furtive'
else:
	assert(False)


foldername = ''
if Follow == 1:
	foldername = 'follow_'

if Caf == 1:
	foldername = foldername + 'caffe_z2_'

foldername = foldername + task + '_'

foldername = foldername + Location + '_'

foldername = foldername + time_str() + '_'

foldername = foldername + computer_name





