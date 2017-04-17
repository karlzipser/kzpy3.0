#use 1 for usb camera, 0 for internal macbook webcam
cap = cv2.VideoCapture(1)
cap.set(3, 672*2/4)
cap.set(4, 376/4)
cap.set(15,-8)
deltas = []
cap2 = cv2.VideoCapture(1)
cap2.set(3, 672*2) # 2560x720
cap2.set(4, 376)
cap2.set(11,30)

ctr = 0
t0 = time.time()
while(True):
	ctr += 1
	if True: #np.mod(ctr,10) == 0:
		ret2, frame2 = cap2.read()
		#cv2.imshow('frame2',frame2)
	else: 
		ret, frame = cap.read()
		cv2.imshow('frame',frame)
		#time.sleep(0.1)

	if cv2.waitKey(1) & 0xFF == ord('q'):
	    break
	cv2.imwrite(opjD('images',d2n(ctr,'.jpg')),frame)
	t1 = time.time()
	while t1-t0 < 0.030:
		t1 = time.time()
	deltas.append(t1-t0)
	t0 = t1

cap.release()
cap2.release()
cv2.destroyAllWindows()