
from kzpy3.vis import *
# run "kzpy3/teg2/gps_fence/first.py"
import math
def rotatePolygon(polygon,theta):
    """http://stackoverflow.com/questions/20023209/function-for-rotating-2d-objects
    Rotates the given polygon which consists of corners represented as (x,y),
    around the ORIGIN, clock-wise, theta degrees"""
    theta = math.radians(theta)
    rotatedPolygon = []
    for corner in polygon :
        rotatedPolygon.append(( corner[0]*math.cos(theta)-corner[1]*math.sin(theta) , corner[0]*math.sin(theta)+corner[1]*math.cos(theta)) )
    return rotatedPolygon


def rotatePoint(centerPoint,point,angle):
    """http://stackoverflow.com/questions/20023209/function-for-rotating-2d-objects
    Rotates a point around another centerPoint. Angle is in degrees.
    Rotation is counter-clockwise"""
    angle = math.radians(angle)
    temp_point = point[0]-centerPoint[0] , point[1]-centerPoint[1]
    temp_point = ( temp_point[0]*math.cos(angle)-temp_point[1]*math.sin(angle) , temp_point[0]*math.sin(angle)+temp_point[1]*math.cos(angle))
    temp_point = temp_point[0]+centerPoint[0] , temp_point[1]+centerPoint[1]
    return temp_point





def unit_vector(vector):
    """http://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python
    Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """http://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python
    Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

"""
http://stackoverflow.com/questions/31735499/calculate-angle-clockwise-between-two-points
"""
from math import acos
from math import sqrt
from math import pi

def length(v):
    return sqrt(v[0]**2+v[1]**2)
def dot_product(v,w):
   return v[0]*w[0]+v[1]*w[1]
def determinant(v,w):
   return v[0]*w[1]-v[1]*w[0]
def inner_angle(v,w):
   cosx=dot_product(v,w)/(length(v)*length(w))
   rad=acos(cosx) # in radians
   return rad*180/pi # returns degrees
def angle_clockwise(A, B):
    inner=inner_angle(A,B)
    det = determinant(A,B)
    if det<0: #this is a property of the det. If the det < 0 then B is clockwise of A
        return inner
    else: # if the det > 0 then A is immediately clockwise of B
        return 360-inner



"""
my_polygon = [(0,0),(1,0),(0,1)]
print rotatePolygon(my_polygon,90)
print rotatePoint((1,1),(2,2),45)
"""



def plt_square():
	plt.gca().set_aspect('equal', adjustable='box')
	plt.draw()




def distance(my_position,object_position):
	dist = np.sqrt((my_position[0]-object_position[0])**2+(my_position[1]-object_position[1])**2)
	return dist

def force_function(my_position,object_position):
	dist = distance(my_position,object_position)
	f = 0.001*1.0/((0.01*dist)**2.5)
	vec = np.array(my_position) - np.array(object_position)
	vec *= f
	#print dist,vec
	return vec

my_position = (np.random.random(2)-0.5)/4.0
my_heading = 90

pts = []
for i in range(20000):
	a = np.random.random()
	p = 2*np.random.random(2) - 1
	if distance(p,(0,0))>a**0.05:
		pts.append(p)
		if len(pts) > 150:
			break
for i in range(100):
	p = 2*np.random.random(2) - 1
	pts.append(p)

vec_total_prev = None
pts = np.array(pts)
rpts = pts.copy()
vec_total_prev = np.array((0,0))
my_position_prev = np.array((0,0))
GO_UP = False
plt.figure(1);plt_square()
while distance(my_position,(0,0)) < 1.415:

	plt.figure(1)
	plt.clf()
	
	#plt.subplot(1,2,1)
	#plt.plot(pts[:,0],pts[:,1],'.')
	#plt.plot(my_position[0],my_position[1],'o')
	#plt.xlim(-0.5,1.5)
	#plt.ylim(-0.5,1.5)

	
	"""
	rpts[:,0] -= my_position[0]
	rpts[:,1] -= my_position[1]
	rpts = rotatePolygon(rpts,my_heading)
	"""
	#rpts = np.array(rpts)
	#plt.subplot(1,2,2)

	#plt.plot(0,0,'o')
	plt.xlim(-1.5,1.5)
	plt.ylim(-1.5,1.5)



	vec_total = np.array([0.0,0.0])
	for i in range(len(rpts)):
		vec_total += 0.01 * force_function((my_position[0],my_position[1]),rpts[i,:])
	#D = distance(my_position,(0,0))
	#if D > 0.5:
	#	vec_total += -10*D*my_position
	vec_total += 1.0*vec_total_prev + 0.01*np.random.random(2)
	if distance(my_position,(1,1))>0.2 and GO_UP:
		vec_total +=25*np.array((1,1))
	elif distance(my_position,(1,1))<0.2 and GO_UP:
		GO_UP = False
		vec_total +=25*np.array((-1,-1))
	elif distance(my_position,(-1,-1))>0.2 and not GO_UP:
		vec_total +=25*np.array((-1,-1))
	else:
		GO_UP = True
		vec_total +=25*np.array((1,1))

	#plt.title(math.degrees(angle_clockwise(vec_total,[0,1])))
	 #[0,1]))
	if vec_total_prev == None:
		vec_total_prev = vec_total.copy()
	a = angle_clockwise(vec_total,[0,1])#,vec_total_prev)
	plt.title(a)

	rpts_rot = []
	for p in rpts:
		rpts_rot.append(rotatePoint([0,0],p,-a))
	rpts_rot = np.array(rpts_rot)
	#plt.plot(rpts_rot[:,0],rpts_rot[:,1],'.')
	#plt.plot([my_position[0],my_position[0]+vec_total[0]/10000.],[my_position[1],my_position[1]+vec_total[1]/10000.],'r-')
	mp_rot2 = np.array(rotatePoint([0,0],my_position+vec_total/10000.,-a))
	mp_rot = np.array(rotatePoint([0,0],my_position,-a))
	plt.plot(rpts_rot[:,0]-mp_rot[0],rpts_rot[:,1]-mp_rot[1],'.')
	plt.plot(mp_rot[0]-mp_rot[0],mp_rot[1]-mp_rot[1],'go')
	plt.plot(mp_rot2[0]-mp_rot[0],mp_rot2[1]-mp_rot[1],'rx')


	
	my_position += 1.0/distance(vec_total,(0,0)) * vec_total * 0.001
	if distance(my_position_prev,my_position) < 0.001:
		my_position+=1.0/distance(vec_total_prev,(0,0)) * vec_total_prev * 0.001
	vec_total_prev = vec_total.copy()
	my_position_prev = my_position.copy()
	plt.pause(0.000133)

raw_input('enter to quit')

