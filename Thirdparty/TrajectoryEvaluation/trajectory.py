"""
This script computes the absolute trajectory error from the ground truth
trajectory and the estimated trajectory.
"""

import sys
import numpy
import argparse
import associate

def readDataset(addressExperiments):
     file = open(addressExperiments)
     data = file.read()
     lines = data.split("\n")

     return lines

def readHeaderTrajectory(addressHeaderTrajectory):
     file = open(addressHeaderTrajectory)
     data = file.read()
     lines = data.split("\n")

     return lines

def align(model,data):
    """Align two trajectories using the method of Horn (closed-form).
    
    Input:
    model -- first trajectory (3xn)
    data -- second trajectory (3xn)
    
    Output:
    rot -- rotation matrix (3x3)
    trans -- translation vector (3x1)
    trans_error -- translational error per point (1xn)
    
    """
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

    print "scale: %f " % s  
    
    trans = data.mean(1) - s*rot * model.mean(1)
    
    model_aligned = s*rot * model + trans
    alignment_error = model_aligned - data
    
    trans_error = numpy.sqrt(numpy.sum(numpy.multiply(alignment_error,alignment_error),0)).A[0]
        
    return rot,trans,trans_error, s

def plot_traj(ax,stamps,traj,style,color,label):
    """
    Plot a trajectory using matplotlib. 
    
    Input:
    ax -- the plot
    stamps -- time stamps (1xn)
    traj -- trajectory (3xn)
    style -- line style
    color -- line color
    label -- plot legend
    
    """
    stamps.sort()
    interval = numpy.median([s-t for s,t in zip(stamps[1:],stamps[:-1])])
    x = []
    y = []
    last = stamps[0]
    for i in range(len(stamps)):
        if stamps[i]-last < 2*interval:
            x.append(traj[i][0])
            y.append(traj[i][1])
        elif len(x)>0:
            ax.plot(x,y,style,color=color,label=label)
            label=""
            x=[]
            y=[]
        last= stamps[i]
    if len(x)>0:
        ax.plot(x,y,style,color=color,label=label)
            

if __name__=="__main__":

    offset = 0.0
    scale = 1.0
    max_difference = 0.02
    save = 0
    save_associations = 0
    plot = 1
    verbose = 1

    address = "/home/kleiber/Desktop/Experiments/"
    experiments = "Datasets.txt"

    addressExperiments = address + experiments
    dataset = readDataset(addressExperiments)

    for line in dataset:
         print "run dataset : %s ..." % line 
         addressDataset = address + line + "/"

         addressHeaderTrajectory = addressDataset + "test/ORB/Header/traj_headers_" + line +".txt";
         headerTrajectory = readHeaderTrajectory(addressHeaderTrajectory)

         addressGroundtruth = addressDataset + "groundtruth.txt"
         first_list = associate.read_file_list(addressGroundtruth)

         ID = -1

         for header in headerTrajectory:
              ID = ID + 1

              if len(header) > 0:
                  addressTrajectory = addressDataset + "test/ORB/Trajectory/" + header;
                  second_list = associate.read_file_list(addressTrajectory)

                  matches = associate.associate(first_list, second_list,float(offset),float(max_difference))

                  if len(matches)<2:
                      print "    Couldn't find matching timestamp : %s !" % header
                      continue

                  print "    trajectory : %s saved!" % header

                  first_xyz = numpy.matrix([[float(value) for value in first_list[a][0:3]] for a,b in matches]).transpose()
                  second_xyz = numpy.matrix([[float(value)*float(scale) for value in second_list[b][0:3]] for a,b in matches]).transpose()
                  rot,trans,trans_error,scale = align(second_xyz,first_xyz)

                  second_xyz_aligned = scale * rot * second_xyz + trans

                  first_stamps = first_list.keys()
                  first_stamps.sort()
                  first_xyz_full = numpy.matrix([[float(value) for value in first_list[b][0:3]] for b in first_stamps]).transpose()

                  second_stamps = second_list.keys()
                  second_stamps.sort()
                  second_xyz_full = numpy.matrix([[float(value)*float(scale) for value in second_list[b][0:3]] for b in second_stamps]).transpose()
                  second_xyz_full_aligned = scale * rot * second_xyz_full + trans

                  file_verbose ="med_" + line + "_" + str(ID) + ".txt"
                  file_associations = "ass_" + line +  "_" + str(ID)
                  file_save = "new_" + line +  "_" + str(ID)
                  file_plot = "plot_" + line +  "_" + str(ID)

                  address_verbose = addressDataset + "test/ORB/Info/" + file_verbose
                  address_save_associations = ""
                  adress_save = ""
                  adress_plot = addressDataset + "test/TrajectoryPlot/" + file_plot

                  if verbose:
                      file = open(address_verbose,"w")
                      file.write("compared_pose_pairs %d pairs\n"%(len(trans_error)))
                      file.write("absolute_translational_error.rmse %f m\n"%numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error)))
                      file.write("absolute_translational_error.mean %f m\n"%numpy.mean(trans_error))
                      file.write("absolute_translational_error.median %f m\n"%numpy.median(trans_error))
                      file.write("absolute_translational_error.std %f m\n"%numpy.std(trans_error))
                      file.write("absolute_translational_error.min %f m\n"%numpy.min(trans_error))
                      file.write("absolute_translational_error.max %f m\n"%numpy.max(trans_error))
                  else:
                      print "%f"%numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error))
        
                  if save_associations:
                      file = open(address_save_associations,"w")
                      file.write("\n".join(["%f %f %f %f %f %f %f %f"%(a,x1,y1,z1,b,x2,y2,z2) for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz.transpose().A,second_xyz_aligned.transpose().A)]))
                      file.close()

                  if save:
                      file = open(adress_save,"w")
                      file.write("\n".join(["%f "%stamp+" ".join(["%f"%d for d in line]) for stamp,line in zip(second_stamps,second_xyz_full_aligned.transpose().A)]))
                      file.close()

                  if plot:
                      import matplotlib
                      matplotlib.use('Agg')
                      import matplotlib.pyplot as plt
                      import matplotlib.pylab as pylab
                      from matplotlib.patches import Ellipse
                      fig = plt.figure()
                      ax = fig.add_subplot(111)
                      plot_traj(ax,first_stamps,first_xyz_full.transpose().A,'-',"black","ground truth")
                      plot_traj(ax,second_stamps,second_xyz_full_aligned.transpose().A,'-',"blue","estimated")

                      label="difference"
                      for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz.transpose().A,second_xyz_aligned.transpose().A):
                           ax.plot([x1,x2],[y1,y2],'-',color="red",label=label)
                           label=""
      
                      ax.legend()
                      ax.set_xlabel('x [m]')
                      ax.set_ylabel('y [m]')
                      plt.savefig(adress_plot,dpi=90)
                      plt.close()
        
