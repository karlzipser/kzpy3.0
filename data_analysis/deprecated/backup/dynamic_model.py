import collections
'''
Create datatype for position, speed into heading and heading (x,y,v,psi)
'''
ModelAnswer = collections.namedtuple('ModelAnswer', ['x', 'y', 'v', 'psi'])

lr = 12
lf = 13




def getXYFor(x_0, y_0, t_0, v, psi, t, a, sigmF):
    """
    Calculates the future position in meter, according to the model.
    Inputs: position at current time x_0, y_0
            current time t_0
            current speed in direction of movement v
            point in time to calculate t
            applied acceleration a
            steering angle over the front tire sigmF
    Note: The method is written for the model cars as used by Karl Zipser and Sascha Hornauer
    and the position of the center of gravity chosen respectively. Change lr and lf if it is elsewhere.
    """
    dt = t-t_0

    beta = np.arctan((lr/lf+lr)*np.tan(sigmF))

    dpsi = (v/lr)*np.sin(beta)
    psi = psi + dpsi*dt
    
    dx = v * np.cos(psi+beta)
    #print("dt " + str(dt))
    #print("dx " + str(dx))
    x = x_0 + dx*dt

    dy = v * np.sin(psi+beta)
    y = y_0 + dy*dt

    dv = a
    v = v + dv*dt
    
    answer = ModelAnswer(x,y,v,psi)
    # Model implemented after http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf
    return answer

def getDistanceInPixel(offsetXInMeter,offsetYInMeter,posXInMeter,posYInMeter,pixelInOneMeterX,pixelInOneMeterY):
    #return [(posXInMeter-offsetXInMeter)*pixelInOneMeterX,(posYInMeter-offsetYInMeter)*pixelInOneMeterY]
    return [posXInMeter*pixelInOneMeterX,posYInMeter*pixelInOneMeterY]
    
