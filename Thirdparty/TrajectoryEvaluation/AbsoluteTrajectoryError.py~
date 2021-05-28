import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy
import math
import sys
import argparse

""" Read dataset """
def readDataset(addressExperiments):
     file = open(addressExperiments)
     data = file.read()
     lines = data.split("\n")
     file.close()
     return lines

""" Read trajectory headers """
def readHeaderTrajectory(addressHeaderTrajectory):
     file = open(addressHeaderTrajectory)
     data = file.read()
     lines = data.split("\n")
     file.close()
     return lines

""" Drawing groundtruth """
def DrawTrajectory(ax, timestamp, groundtruth, Color, Label):
    timestamp.sort()
    x = []
    y = []
    z = []

    for i in range(len(timestamp)):
        x.append(groundtruth[i][0])
        y.append(groundtruth[i][1])
        z.append(groundtruth[i][2])
    
    ax.plot(x, y, z, c=Color, label=Label, linewidth=0.6)


""" Drawing keyframe trajectory """
def DrawKeyFrames(ax, timestamp, frame, error):
    timestamp.sort()
    x = []
    y = []
    z = []
    
    for i in range(len(timestamp)):
        x.append(frame[i][0])
        y.append(frame[i][1])
        z.append(frame[i][2])

    cm = plt.cm.get_cmap('autumn')
    sc = ax.scatter(x, y, z, c=error, cmap=cm)
    cb = plt.colorbar(sc)
    cb.set_label('error[m]', rotation= 0)

""" Drawing keyframe trajectory (inlier) """
def DrawKeyFramesInlier(ax, inlier, timestamp, frame, error, ColorMap, Label, s=40):
    inlier.sort()
    x = []
    y = []
    z = []
    indice = range(len(timestamp))
   
    for i, key in zip(indice, timestamp):
        if key in inlier:
           x.append(frame[i][0])
           y.append(frame[i][1])
           z.append(frame[i][2])

    cm = plt.cm.get_cmap(ColorMap)
    sc = ax.scatter(x, y, z, c=error, cmap=cm,  label= Label)

    cb = plt.colorbar(sc)
    cb.set_label(' ', rotation= 0)

""" Drawing keyframe trajectory (outlier) """
def DrawKeyFramesOutlier(ax, inlier, timestamp, frame, Color, Label):
    inlier.sort()
    x = []
    y = []
    z = []
    indice = range(len(timestamp))
   
    for i, key in zip(indice, timestamp):
        if key in inlier:
           continue
        else:
           x.append(frame[i][0])
           y.append(frame[i][1])
           z.append(frame[i][2])

    sc = ax.scatter(x, y, z, c = Color, label=Label, s=40)


""" Read file as a dictionary """
def ReadFile(filename):
    file = open(filename)
    data = file.read()
    
    lines = data.replace(","," ").replace("\t"," ").split("\n")

    list = [[v.strip() for v in line.split(" ") if v.strip() != ""] for line in lines if len(line) > 0 and line[0] != "#"]
    list = [(float(l[0]), l[1:]) for l in list if len(l) > 1]

    return dict(list)


""" Associate groundtruth with keyframe trajectory """
def Associate(groundtruth, frame, offset, max_difference):
    groundtruthKeys = groundtruth.keys()
    frameKeys = frame.keys()

    potentialMatches = [(abs(i - (j + offset)), i, j)
                        for i in groundtruthKeys
                        for j in frameKeys
                        if abs(i - (j + offset)) < max_difference]
 
    potentialMatches.sort()

    matches = []
    for diff, i, j in potentialMatches:
        if i in groundtruthKeys and j in frameKeys:
           groundtruthKeys.remove(i)
           frameKeys.remove(j)
           matches.append((i,j))

    matches.sort()

    return matches


""" Alignment matches beetwen groundtruth and keyframe trajectory using the method of Horn (closed-form) """
def Alignment(model,data):
    numpy.set_printoptions(precision=3,suppress=True)
    model_zerocentered = model - model.mean(1)
    data_zerocentered = data - data.mean(1)
    
    W = numpy.zeros( (3,3) )
    for column in range(model.shape[1]):
        W += numpy.outer(model_zerocentered[:,column],data_zerocentered[:,column])
    U,d,Vh = numpy.linalg.linalg.svd(W.transpose())
    S = numpy.matrix(numpy.identity( 3 ))
    if(numpy.linalg.det(U) * numpy.linalg.det(Vh)<0):
        S[2,2] = -1
    rot = U*S*Vh

    rotmodel = rot*model_zerocentered
    dots = 0.0
    norms = 0.0

    for column in range(data_zerocentered.shape[1]):
	dots += numpy.dot(data_zerocentered[:,column].transpose(),rotmodel[:,column])
        normi = numpy.linalg.norm(model_zerocentered[:,column])
        norms += normi*normi

    s = float(dots/norms)    

    trans = data.mean(1) - s*rot * model.mean(1)
    
    model_aligned = s*rot * model + trans
    alignment_error = model_aligned - data
    
    trans_error = numpy.sqrt(numpy.sum(numpy.multiply(alignment_error,alignment_error),0)).A[0]
        
    return s, rot,trans,trans_error


""" programa principal """
def run(groundtruthFile, frameFile, header, addressFigures, addressMeasurements):
    #Parametros para realizar la asociacion, el alineamiento, guardar y plotar 
    scale = 1.0
    offset = 0.0
    max_difference = 0.2

    dataset = header[5:header.find(".")] 

    information = addressMeasurements + "med_" + dataset + ".txt"
    plot3D = "plt_" + dataset + ".png"
    plotError = "error_" + dataset + ".png" #Absolute Trajectory Error (ATE)

    #leer los archivos
    groundtruth = ReadFile(groundtruthFile)
    frame = ReadFile(frameFile)

    #realizar el matching entre los tiempos de ambos archivos
    matches = Associate(groundtruth, frame, offset, max_difference) 
    if len(matches)<2: return 0

    #para cada match extraer su puntos xyz en ambos archivos
    groundtruthXYZ = numpy.matrix([[float(value) for value in groundtruth[f][0:3]] for f,s in matches]).transpose()
    frameXYZ = numpy.matrix([[float(value)*float(scale) for value in frame[s][0:3]] for f,s in matches]).transpose()

    #alinear ambos puntos
    s, R ,t , t_error = Alignment(frameXYZ, groundtruthXYZ)

    #alinear los puntos del matching en el frame
    frameXYZ_aligned = s * R * frameXYZ + t

    #extraer todos los puntos xyz del groundtruth
    groundtruth_timestamp = groundtruth.keys()
    groundtruth_timestamp.sort()
    groundtruthXYZ_full = numpy.matrix([[float(value) for value in groundtruth[i][0:3]] for i in groundtruth_timestamp]).transpose()

    #extraer todos los puntos xyz del keyframe trajectory y alinearlos
    frame_timestamp = frame.keys()
    frame_timestamp.sort()
    frameXYZ_full = numpy.matrix([[float(value)*float(scale) for value in frame[i][0:3]] for i in frame_timestamp]).transpose()
    frameXYZ_full_aligned = s * R * frameXYZ_full + t


    #calculamos el error para cada punto XYZ en el keyframe trajectory
    inlier = []
    for e,(f,s),(x1,y1,z1),(x2,y2,z2) in zip(t_error,matches,groundtruthXYZ.transpose().A,frameXYZ_aligned.transpose().A):
        inlier.append(s)

    #guardar informacion del error
    if information:
       file = open(information,"w")
       file.write("trajectorie_pose_pairs %d pairs\n"%(len(frame_timestamp)))
       file.write("compared_pose_pairs %d pairs\n"%(len(t_error)))
       file.write("absolute_translational_error.rmse %f m\n"%numpy.sqrt(numpy.dot(t_error,t_error) / len(t_error)))
       file.write("absolute_translational_error.mean %f m\n"%numpy.mean(t_error))
       file.write("absolute_translational_error.median %f m\n"%numpy.median(t_error))
       file.write("absolute_translational_error.std %f m\n"%numpy.std(t_error))
       file.write("absolute_translational_error.min %f m\n"%numpy.min(t_error))
       file.write("absolute_translational_error.max %f m\n"%numpy.max(t_error))
       file.close()

    #dibujar el error en base al tiempo
    if plotError:
       used = range(len(t_error))
       error = plt.figure(1)
       plt.ylim(0.0, 0.09)
       #plt.plot(used, t_error, c = 'r')
       plt.bar(used, t_error, color='r')
       plt.xlabel('time [s]', fontsize=18)
       plt.ylabel('translational error [m]', fontsize=16)
       plt.grid(True)

    #dibujar la trajectoria y los puntos keyframe
    if plot3D:
       fig = plt.figure(2)
       ax = fig.add_subplot(111, projection='3d')
       DrawTrajectory(ax, groundtruth_timestamp, groundtruthXYZ_full.transpose().A, 'y', 'groundtruth')
       DrawKeyFramesInlier(ax, inlier, frame_timestamp, frameXYZ_full_aligned.transpose().A, t_error, 'autumn', "inlier")
       DrawKeyFramesOutlier(ax, inlier, frame_timestamp, frameXYZ_full_aligned.transpose().A, 'b', "outlier")
       ax.legend()
       ax.set_xlabel('x[m]')
       ax.set_ylabel('y[m]')

    addressError = addressFigures + plotError
    addressPlot = addressFigures + plot3D

    error.savefig(addressError)
    fig.savefig(addressPlot)
    #plt.show()
    plt.close(error)
    plt.close(fig)

    return 1

""" configuracion para todos los dataset """
def runDatasetORB(address, experiments):
    addressExperiments = address + experiments
    dataset = readDataset(addressExperiments)

    for line in dataset:
        if len(line) > 0:
           print "run dataset : %s ... done!" % line
           addressDataset = address + line + "/"
           addressHeaderTrajectory = addressDataset + "test/ORB/Header/traj_headers_" + line +".txt";
           addressMeasurements = addressDataset + "test/ORB/Measurement/ORB_ate_"
           addressFigures = addressDataset + "test/PlotTrajectory/ORB_ate_"
           headerTrajectory = readHeaderTrajectory(addressHeaderTrajectory)
           groundtruthFile = addressDataset + "groundtruth.txt"
           test = -1
	   for header in headerTrajectory:
               test = test + 1
               if len(header) > 0:
                  frameFile = addressDataset + "test/ORB/Trajectory/" + header;
                  ok = run(groundtruthFile, frameFile, header, addressFigures, addressMeasurements)
    return 0

""" configuracion para todos los dataset """
def runDatasetFUS(address, experiments):
    addressExperiments = address + experiments
    dataset = readDataset(addressExperiments)

    for line in dataset:
        if len(line) > 0:
           print "run dataset : %s ... done!" % line
           addressDataset = address + line + "/"
           addressHeaderTrajectory = addressDataset + "test/FUS/Header/traj_headers_" + line +".txt";
           addressMeasurements = addressDataset + "test/FUS/Measurement/FUS_ate_"
           addressFigures = addressDataset + "test/PlotTrajectory/FUS_ate_"
           headerTrajectory = readHeaderTrajectory(addressHeaderTrajectory)
           groundtruthFile = addressDataset + "groundtruth.txt"
           test = -1
	   for header in headerTrajectory:
               test = test + 1
               if len(header) > 0:
                  frameFile = addressDataset + "test/FUS/Trajectory/" + header;
                  ok = run(groundtruthFile, frameFile, header, addressFigures, addressMeasurements)
    return 0


""" Main """
if __name__=="__main__":
    address = "/home/kleiber/Desktop/Experiments/"
    experiments = "Datasets.txt"
    runDatasetORB(address, experiments)
    #runDatasetFUS(address, experiments)



