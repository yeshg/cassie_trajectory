
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

    x = initPos[0] \
        + (initPos[0] - finalPos[0])*( \
            15* t**4 - 6* t**5 - 10* t**3 )

    dx = (initPos[0] - finalPos[0])*( \
            15*4* t**3 - 6*5* t**4 - 10*3* t**2 )

    y = initPos[1] \
        + (initPos[1] - finalPos[1])*( \
            15* t**4 - 6* t**5 - 10* t**3 )

    dy = (initPos[1] - finalPos[1])*( \
            15*4* t**3 - 6*5* t**4 - 10*3* t**2 )

    hrel = h

    z1 = initPos[2] \
        + (16*hrel/3)*( 10* t1**3 - 25*t1**4 + 16* t1**5)

    dz1 = (16*hrel/3)*( 10*3* t1**2 - 25*4* t1**3 + 16*5* t1**4)

    z2 = initPos[2] \
        + (16*hrel/3)*( 10*(1-t2)**3 - 25*(1-t2)**4 + 16* (1-t2)**5)

    dz2 = (16*hrel/3)*( -10*3* (1-t2)**2 + 25*4* (1-t2)**3 - 16*5* (1-t2)**4)

    print(initPos)
    print(hrel)
    # print(t.shape)
    # print(t1.shape)
    # print(t2.shape)
    # print(x.shape)
    # print(y.shape)
    # print(z1.shape)
    # print(z2.shape)

    # import matplotlib.pyplot as plt

    # plt.subplot(2, 1, 1)
    # plt.plot(t,x)
    # plt.plot(t,y)
    # plt.plot(t,np.concatenate((z1,z2)))
    # plt.subplot(2, 1, 2)
    # plt.plot(t,dx)
    # plt.plot(t,dy)
    # plt.plot(t,np.concatenate((dz1,dz2)))
    # plt.show()
    # input()

    out = np.transpose([t, x, y, np.concatenate((z1,z2)), dx, dy, np.concatenate((dz1,dz2))])

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
    