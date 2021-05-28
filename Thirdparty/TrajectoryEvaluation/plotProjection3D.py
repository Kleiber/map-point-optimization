import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
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
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x, y, z, label='groundtruth', c = 'b')

    #x, y, z,v = (np.random.random((4,100))-0.5)*15
    #print v
    #c = np.abs(v)
    #cmhot = plt.get_cmap("hot")
    #cax = ax.scatter(x, y, z, v, s=50, c=c, cmap=cmhot)

    plt.show()
