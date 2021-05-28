import argparse
import random
import numpy
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab

_EPS = numpy.finfo(float).eps * 4.0

""" convertir pose a una matrix de transformacion """
def transform44(l):
    t = l[1:4]
    q = numpy.array(l[4:8], dtype=numpy.float64, copy=True)
    nq = numpy.dot(q, q)
    if nq < _EPS:
        return numpy.array((
        (                1.0,                 0.0,                 0.0, t[0])
        (                0.0,                 1.0,                 0.0, t[1])
        (                0.0,                 0.0,                 1.0, t[2])
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=numpy.float64)
    q *= numpy.sqrt(2.0 / nq)
    q = numpy.outer(q, q)
    return numpy.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], t[0]),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], t[1]),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], t[2]),
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=numpy.float64)

""" leer la trayectoria y retornar un diccionario de matrices de transformacion o solo poses"""
def read_trajectory(filename, matrix=True):
    file = open(filename)
    data = file.read()
    lines = data.replace(","," ").replace("\t"," ").split("\n") 
    list = [[float(v.strip()) for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"]
    list_ok = []
    for i,l in enumerate(list):
        if l[4:8]==[0,0,0,0]:
            continue
        isnan = False
        for v in l:
            if numpy.isnan(v): 
                isnan = True
                break
        if isnan:
            sys.stderr.write("Warning: line %d of file '%s' has NaNs, skipping line\n"%(i,filename))
            continue
        list_ok.append(l)
    if matrix :
      traj = dict([(l[0],transform44(l[0:])) for l in list_ok])
    else:
      traj = dict([(l[0],l[1:8]) for l in list_ok])
    return traj

""" scalar la traslacion de una matrix de transfornmacion """
def scale(a,scalar):
    return numpy.array(
        [[a[0,0], a[0,1], a[0,2], a[0,3]*scalar],
         [a[1,0], a[1,1], a[1,2], a[1,3]*scalar],
         [a[2,0], a[2,1], a[2,2], a[2,3]*scalar],
         [a[3,0], a[3,1], a[3,2], a[3,3]]])

""" calcular la transformacion relativa 3D de a para b """
def ominus(a,b):
    return numpy.dot(numpy.linalg.inv(a),b)

""" distancia, norma de la componente de traslacion """
def compute_distance(transform):	
    return numpy.linalg.norm(transform[0:3,3])

""" angulo de rotacion de la matrix homogenea """
def compute_angle(transform):
    # an invitation to 3-d vision, p 27
    return numpy.arccos( min(1,max(-1, (numpy.trace(transform[0:3,0:3]) - 1)/2) ))

""" distancia traslacional a lo largo de la trayectoria """
def distances_along_trajectory(traj):
    keys = traj.keys()
    keys.sort()
    motion = [ominus(traj[keys[i+1]],traj[keys[i]]) for i in range(len(keys)-1)]
    distances = [0]
    sum = 0
    for t in motion:
        sum += compute_distance(t)
        distances.append(sum)
    return distances

""" rotacion angular a lo largo de la trayectoria """
def rotations_along_trajectory(traj,scale):
    keys = traj.keys()
    keys.sort()
    motion = [ominus(traj[keys[i+1]],traj[keys[i]]) for i in range(len(keys)-1)]
    distances = [0]
    sum = 0
    for t in motion:
        sum += compute_angle(t)*scale
        distances.append(sum)
    return distances

""" encontrar un valor en una lista , busca binaria de t en la lista L"""
def find_closest_index(L,t):
    beginning = 0
    difference = abs(L[0] - t)
    best = 0
    end = len(L)
    while beginning < end:
        middle = int((end+beginning)/2)
        if abs(L[middle] - t) < difference:
            difference = abs(L[middle] - t)
            best = middle
        if t == L[middle]:
            return middle
        elif L[middle] > t:
            end = middle
        else:
            beginning = middle + 1
    return best


""" Calcular la relative pose error entre dos trayectorias """
def evaluate_trajectory(traj_gt, traj_est, param_max_pairs=10000, param_fixed_delta=False, param_delta=1.00, param_delta_unit="s", param_offset=0.00, param_scale=1.00):

    stamps_gt = list(traj_gt.keys())
    stamps_est = list(traj_est.keys())
    stamps_gt.sort()
    stamps_est.sort()
    
    stamps_est_return = []
    for t_est in stamps_est:
        t_gt = stamps_gt[find_closest_index(stamps_gt,t_est + param_offset)]
        t_est_return = stamps_est[find_closest_index(stamps_est,t_gt - param_offset)]
        t_gt_return = stamps_gt[find_closest_index(stamps_gt,t_est_return + param_offset)]
        if not t_est_return in stamps_est_return:
            stamps_est_return.append(t_est_return)

    if(len(stamps_est_return)<2):
        raise Exception("Number of overlap in the timestamps is too small. Did you run the evaluation on the right files?")

    if param_delta_unit=="s":
        index_est = list(traj_est.keys())
        index_est.sort()
    elif param_delta_unit=="m":
        index_est = distances_along_trajectory(traj_est)
    elif param_delta_unit=="rad":
        index_est = rotations_along_trajectory(traj_est,1)
    elif param_delta_unit=="deg":
        index_est = rotations_along_trajectory(traj_est,180/numpy.pi)
    elif param_delta_unit=="f":
        index_est = range(len(traj_est))
    else:
        raise Exception("Unknown unit for delta: '%s'"%param_delta_unit)

    if not param_fixed_delta:
        if(param_max_pairs==0 or len(traj_est)<numpy.sqrt(param_max_pairs)):
            pairs = [(i,j) for i in range(len(traj_est)) for j in range(len(traj_est))]
        else:
            pairs = [(random.randint(0,len(traj_est)-1),random.randint(0,len(traj_est)-1)) for i in range(param_max_pairs)]
    else:
        pairs = []
        for i in range(len(traj_est)):
            j = find_closest_index(index_est,index_est[i] + param_delta)
            if j!=len(traj_est)-1: 
                pairs.append((i,j))
        if(param_max_pairs!=0 and len(pairs)>param_max_pairs):
            pairs = random.sample(pairs,param_max_pairs)
    
    gt_interval = numpy.median([s-t for s,t in zip(stamps_gt[1:],stamps_gt[:-1])])
    gt_max_time_difference = 2*gt_interval
    
    result = []
    for i,j in pairs:
        stamp_est_0 = stamps_est[i]
        stamp_est_1 = stamps_est[j]

        stamp_gt_0 = stamps_gt[ find_closest_index(stamps_gt,stamp_est_0 + param_offset) ]
        stamp_gt_1 = stamps_gt[ find_closest_index(stamps_gt,stamp_est_1 + param_offset) ]
        
        if(abs(stamp_gt_0 - (stamp_est_0 + param_offset)) > gt_max_time_difference  or
           abs(stamp_gt_1 - (stamp_est_1 + param_offset)) > gt_max_time_difference):
            continue
        
        error44 = ominus(  scale(
                           ominus( traj_est[stamp_est_1], traj_est[stamp_est_0] ),param_scale),
                           ominus( traj_gt[stamp_gt_1], traj_gt[stamp_gt_0] ) )
        
        trans = compute_distance(error44)
        rot = compute_angle(error44)
        
        result.append([stamp_est_0,stamp_est_1,stamp_gt_0,stamp_gt_1,trans,rot])
        
    if len(result)<2:
        raise Exception("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory!")
        
    return result

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

def run(groundtruth_file, estimated_file, header, addressFigures, addressMeasurements):
    random.seed(0)

    max_pairs = 10000
    fixed_delta = True
    delta = 1.0
    delta_unit = 's'
    offset = 0.0
    scale = 1.0

    dataset = header[5:header.find(".")] 

    information = addressMeasurements + "med_rpe_" + dataset + ".txt"
    plot3D = "plt_rpe_" + dataset + ".png"
    plotError = "arpe_" + dataset + ".png" #Absolute Trajectory Error (ATE)
    
    if plotError and not fixed_delta:
        sys.exit("The '--plot' option can only be used in combination with '--fixed_delta'")
    
    traj_gt = read_trajectory(groundtruth_file) #ok
    traj_est = read_trajectory(estimated_file) #ok
    
    result = evaluate_trajectory(traj_gt, traj_est, int(max_pairs), fixed_delta, float(delta), delta_unit, float(offset), float(scale))
    
    stamps = numpy.array(result)[:,0]
    trans_error = numpy.array(result)[:,4]
    rot_error = numpy.array(result)[:,5]
    
    if information:
        file = open(information,"w")
        file.write("compared_pose_pairs %d pairs\n"%(len(trans_error)))
        file.write("translational_error.rmse %f m\n"%numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error)))
        file.write("translational_error.mean %f m\n"%numpy.mean(trans_error))
        file.write("translational_error.median %f m\n"%numpy.median(trans_error))
        file.write("translational_error.std %f m\n"%numpy.std(trans_error))
        file.write("translational_error.min %f m\n"%numpy.min(trans_error))
        file.write("translational_error.max %f m\n"%numpy.max(trans_error))
        file.write("rotational_error.rmse %f deg\n"%(numpy.sqrt(numpy.dot(rot_error,rot_error) / len(rot_error)) * 180.0 / numpy.pi))
        file.write("rotational_error.mean %f deg\n"%(numpy.mean(rot_error) * 180.0 / numpy.pi))
        file.write("rotational_error.median %f deg\n"%numpy.median(rot_error))
        file.write("rotational_error.std %f deg\n"%(numpy.std(rot_error) * 180.0 / numpy.pi))
        file.write("rotational_error.min %f deg\n"%(numpy.min(rot_error) * 180.0 / numpy.pi))
        file.write("rotational_error.max %f deg\n"%(numpy.max(rot_error) * 180.0 / numpy.pi))
        file.close()
    else:
        print numpy.mean(trans_error)

    if plotError:    
        error = plt.figure()
        plt.bar(stamps - stamps[0],trans_error,color='b')
        plt.xlabel('time [s]', fontsize=18)
        plt.ylabel('translational error [m]', fontsize=16)
        plt.grid(True)

    addressError = addressFigures + plotError
    error.savefig(addressError)
    plt.close(error)

""" configuracion para todos los dataset """
def runDataset(address, experiments):
    addressExperiments = address + experiments
    dataset = readDataset(addressExperiments)

    for line in dataset:
        if len(line) > 0:
           print "run dataset : %s ... done!" % line
           addressDataset = address + line + "/"
           addressHeaderTrajectory = addressDataset + "test/FUS/Header/traj_headers_" + line +".txt";
           addressMeasurements = addressDataset + "test/FUS/Measurement/FUS_rpe_"
           addressFigures = addressDataset + "test/PlotTrajectory/FUS_"
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
    runDataset(address, experiments)

