
import numpy as np
# This function returns minimum jerk foot trajectories in the form of a 2D
# evenly spaced numpy array
# Inputs:
#           initPos     initial position 3 element  array-like
#           finalPos    final position 3 element  array-like
#           nPoints     number of points that the swing trajectory will span
#           h           Peak (midpoint) Z height of the swing trajectory 
#                       relative to the initial z height
#           zdotTD      Vertical touchdown velocity normalized to length of 
#                       swing. To get this from real desired velocity, divide
#                       by the swing phase length
# Outputs:
#           out         this is a [nPoints x 4] numpy 2d array 
def min_jerk_foot_trajectories( initPos, finalPos, nPoints, h, zdotTD):

    T = 1.0
    t = np.linspace(0.0, T, nPoints)
    t1 = t[0:int(np.floor(nPoints/2))]
    t2 = t[int(np.floor(nPoints/2)):]

    xF = finalPos[0] - initPos[0]
    x = initPos[0]     \
        +(5*xF)/(2*(T**2)) * (t**2) \
        -(5*xF)/(2*(T**4)) * (t**4) \
        +xF/(T**5) * (t**5)

    yF = finalPos[1] - initPos[1]
    y = initPos[1]  \
        +(5*yF)/(2*(T**2)) * (t**2)  \
        -(5*yF)/(2*(T**4)) * (t**4)  \
        +yF/(T**5) * (t**5)

    zF = finalPos[2] - initPos[2]
    hrel = h

    z1 = initPos[2] \
        +(40*hrel + (20*zF)/3 - T*zdotTD)/(4*T**2) * (t1**2)  \
         -(40*hrel + 20*zF - 3*T*zdotTD)/T**4 * (t1**4)  \
         +(4*(24*hrel + 20*zF - 3*T*zdotTD))/(3*T**5) * (t1**5)

    z2 = initPos[2] + 2*hrel - (28*zF)/3 + (7*T*zdotTD)/4  \
        -(120*hrel - 460*zF + 87*T*zdotTD)/(6*T) * (t2)  \
        +(1080*hrel - 2860*zF + 549*T*zdotTD)/(12*T**2) * (t2**2) \
        -(480*hrel - 1040*zF + 204*T*zdotTD)/(3*T**3) * (t2**3) \
        +(360*hrel - 700*zF + 141*T*zdotTD)/(3*T**4) * (t2**4)  \
        -(4*(24*hrel - 44*zF + 9*T*zdotTD))/(3*T**5) * (t2**5)

    print(t.shape)
    print(t1.shape)
    print(t2.shape)
    print(x.shape)
    print(y.shape)
    print(z1.shape)
    print(z2.shape)

    out = np.transpose([t, x, y, np.concatenate((z1,z2)) ])

    return out



if __name__ == '__main__':

    
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    initPos =[0.2,-0.1,0]

    finalPos = [1,0.1,0]

    h = 0.1

    zdotTD = -0.5

    data = min_jerk_foot_trajectories(initPos, finalPos, 100, h, zdotTD)

    print(data.shape)


    plt.plot(data[:,0], data[:,1])
    plt.plot(data[:,0], data[:,2])
    plt.plot(data[:,0], data[:,3])
    plt.show()

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(data[:,1],data[:,2],data[:,3])
    ax.axis('equal')
    ax.set_xlim(0,1)
    ax.set_ylim(0,1)
    ax.set_zlim(0,1)
    plt.show()
    