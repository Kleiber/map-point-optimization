import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import sys
import argparse
import associate

if __name__=="__main__":
    groundtruth = associate.read_file_list("/home/kleiber/Desktop/VisionProject/Thirdparty/scale/groundtruth.txt")

    timestamp = groundtruth.keys()
    timestamp.sort()
    pointsXYZ = np.matrix([[float(value) for value in groundtruth[i][0:3]] for i in timestamp]).A

    x = []
    y = []
    z = []
    for i in range(len(timestamp)):
         x.append(pointsXYZ[i][0])
         y.append(pointsXYZ[i][1])
         z.append(pointsXYZ[i][2])

    cm = plt.cm.get_cmap('RdYlBu')
    mpl.rcParams['legend.fontsize'] = 10
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    ax.plot(x, y, z, label='groundtruth', c = 'b')

    p = ax.scatter(x[0], y[0], z[0], zdir='z', c='r')

    ax.legend()
    ax.set_xlabel('X[m]')
    ax.set_ylabel('Y[m]')
    ax.set_zlabel('Z[m]')


    #cm = plt.cm.get_cmap('RdYlBu')
    #xy = range(30)
    #z = xy
    #sc = plt.scatter(xy, xy, c=z, vmin=0, vmax=20, s=35, cmap=cm)
    #plt.colorbar(sc)


#from mpl_toolkits.mplot3d import Axes3D
#fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')

#xs,ys,zs = np.random.random(50),np.random.random(50),np.random.random(50)
#values = np.random.random(50)*10

#p = ax.scatter3D(xs, ys, zs=zs, c=values, cmap='hot')

#fig.colorbar(p, ax=ax)
    plt.show()
