from kzpy3.utils import *

hostname = "google.com" #example

ctr = 0

while True:
	time.sleep(1)
	response = os.system("ping -c 1 " + hostname)

	if response == 0:
		print hostname, 'is up!'
		ctr = 0
	else:
		print hostname, 'is down!'
		ctr += 1

	if ctr >= 2:
		print 'restarting net!'
		unix('/home/ubuntu/RESTART_NET.sh')
		pass