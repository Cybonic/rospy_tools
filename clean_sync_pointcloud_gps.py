import os 
import sys
import yaml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.pyplot as plt
# from utils import *
from sklearn.neighbors import NearestNeighbors
from mpl_toolkits.mplot3d import Axes3D
from tqdm import tqdm
import argparse
#import loop_corr_config as config
from scipy.spatial import distance
#from oxford_lib.image import load_image
import cv2
import matplotlib as mpl
from matplotlib.backends.backend_agg import FigureCanvasAgg

PLOTTING_FLAG = True

TWIDTH = 300 
THIGHT = 240 

SYNC_OFFSET = 0

font                   = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10,50)
fontScale              = 2
fontColor              = (0,0,0)
lineType               = 2

fig, ax = plt.subplots()

def save_syn_file(file_path, idx):
    str_idx =  ' '.join([str(v) for v in idx])
    f = open(file_path,'w')
    f.write(str_idx + '\n')
    f.close()
    print("Saved to: "+ file_path)



def nearest_neighbors(target,array,n,algorithm ='abs'):
    if algorithm=='abs':
        v = np.abs(target-array)
        neighbors_idx = np.argsort(v)[:n]
    elif algorithm=='positive':
        array_idx = np.array(range(0,len(array)))
        v = array-target # Get only those values bigger than tkhe target value
        positive_idx = array_idx[v>0]
        v = np.abs((array[positive_idx]-target))
        sort_idx = np.argsort(v)
        neighbors_idx = positive_idx[sort_idx][:n]

    return array[neighbors_idx], neighbors_idx

def sync_modalities(base,second,kneighbors=3,debug=False):
    base_synch_idx=[]
    second_synch_idx=[]

    for i,(b) in tqdm(enumerate(base),"Synchronizing Timesamps"):
        values, idx = nearest_neighbors(b,second,kneighbors,'positive')
        if len(idx) > 0:
            second_synch_idx.append(idx[0])
            base_synch_idx.append(i)

        if debug==True:
            print(f'\n{i} -> {idx}')
            print(f'\n{b} -> {second[idx]}')
    
    return(np.array(base_synch_idx),np.array(second_synch_idx))


def frame_distance(xy):
    data_len = len(xy)
    delta = []
    for i in range(data_len-1):
        delta.append(np.sqrt(np.sum((xy[i]-xy[i+1])**2)))
    delta.append(0)
    return np.array(delta)

def load_pose_to_RAM(file):
    assert os.path.isfile(file)
    pose_array = []
    for line in tqdm(open(file), 'Loading to RAM'):
        values_str = line.split(' ')
        values = [float(v) for v in values_str]
        pose_array.append(values[0:3])
    return(np.array(pose_array))

def colorFader(c1,c2,mix=0): #fade (linear interpolate) from color c1 (at mix=0) to c2 (mix=1)
    c1=np.array(mpl.colors.to_rgb(c1))
    c2=np.array(mpl.colors.to_rgb(c2))
    return mpl.colors.to_hex((1-mix)*c1 + mix*c2)

def plot_on_map(pose,good_idx):
    """
    Input args:
     - name: (str) Name of image to be saved
     - pose: (numpy) nx3 array of poses
     - query:  query indice
     - similarity: (numpy) array of similarities, with the same order as pose
    
    Output arg:
     - Numpy array of the image
    """
    # https://stackoverflow.com/questions/25668828/how-to-create-colour-gradient-in-python
    fig, ax = plt.subplots()
    canvas = FigureCanvasAgg(fig)
    num_samples = pose.shape[0]

    all_idx= np.arange(len(pose))
    to_remove_idx = np.setxor1d(all_idx,good_idx)


    scale = np.ones(num_samples)*15
    c = np.array([mpl.colors.to_hex('Black')]*num_samples)
    
    c[to_remove_idx] = mpl.colors.to_hex('red')
    c[good_idx] = mpl.colors.to_hex('green')

    p = ax.scatter(pose[:,0],pose[:,1],s = scale, c = c)
    ax.set_aspect('equal')
    canvas.draw()
    buf = canvas.buffer_rgba()
    # convert to a NumPy array
    X = np.asarray(buf)
    return X

def main_clean_and_synch()

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Play back images from a given directory')
    parser.add_argument('--root', type=str, default='/media/tiago/vbig/dataset/FU_Odometry_Dataset/64LiDAR/20220505-120825')
    parser.add_argument('--plot',default  = True ,type = bool)
    parser.add_argument('--dist_thresh',default  = 0.01 ,type = float)
    parser.add_argument('--pose_file',default  = 'pose' ,choices = ['gps','odom'])
    
    args = parser.parse_args()

    dist_thresh = args.dist_thresh
    #dataset= args.dataset 
    root = args.root
    #seq = args.seq
    plotting_flag = args.plot

    if not os.path.isdir('temp'):
        os.makedirs('temp')

    print("="*20)
    #print("[INF] Dataset Name:  " + dataset)
    #print("[INF] Sequence Name: " + seq )
    print("[INF] Plotting Flag: " + str(plotting_flag) )
    # Build loop correspondess of query and reference  paths

    # Synchronization of images with poses
    assert os.path.isdir(root),'Path Does not Exist: ' + root 
    # pose_stamps_file = os.path.join(dir,'gps_times.txt') #pose_file 
    pose_stamps_file = os.path.join(root, f'pose_times.txt')#
    print("[INF] Loading: " + pose_stamps_file )
    pose_stamps = np.array([int(i) for i in open(pose_stamps_file,'r')])

    velo_stamps_file = os.path.join(root,'point_cloud_times.txt')
    print("[INF] Loading: " + velo_stamps_file )
    velo_stamps = np.array([int(i) for i in open(velo_stamps_file,'r')])
 
    pose_file = os.path.join(root,'pose.txt')
    print("[INF] Loading: " + pose_file )
    pose =load_pose_to_RAM(pose_file)

    # =================================================================
    # Sync point clouds and poses
    # ==================================================================
    velo_stamps_len = len(velo_stamps)
    pose_stamps_len = len(pose_stamps)

    print("=======================================")
    print("Velo Semaples: %d"%(velo_stamps_len))
    print("Pose Semaples: %d"%(pose_stamps_len))

    # the modality which as the lowest number of samples is the baseline
    if len(pose_stamps) < len(velo_stamps):
        base = np.array(pose_stamps)
        second = np.array(velo_stamps)
        base_modality = 'pose'
    else:
        base = np.array(velo_stamps)
        second = np.array(pose_stamps)
        base_modality = 'pointcloud'
    
    print("Base Modality: " + base_modality)

    sync_base_idx, sync_second_idx = sync_modalities(base,second,kneighbors=3,debug =plotting_flag)

    if len(pose_stamps) < len(velo_stamps):
        pose = pose[sync_base_idx]
    else:
        pose = pose[sync_second_idx]
    # =================================================================
    #  Remove data points based on some criterion
    # ==================================================================

    dist = frame_distance(pose)
    good_idx = dist>dist_thresh 
    
    print("Total data points %d"%(len(dist)))
    print("Data points Removed  %d"%(len(dist) - np.sum(good_idx)))

    x = plot_on_map(pose,good_idx)
    from PIL import Image
    pil_im = Image.fromarray(x)
    pil_im.save('temp/cleaning.png')

    sync_base_idx = sync_base_idx[good_idx]
    sync_second_idx = sync_second_idx[good_idx]
   
    #sync_base_idx, sync_second_idx = sync_modalities(base[sync_base_idx],second[sync_second_idx],kneighbors=3,debug =True)
    if sync_second_idx == 'pose': 
        save_syn_file(os.path.join(root,f'sync_pose_idx.txt'),sync_base_idx)
        save_syn_file(os.path.join(root,'sync_point_cloud_idx.txt'),sync_second_idx)
    else:
        save_syn_file(os.path.join(root,f'sync_pose_idx.txt'),sync_second_idx)
        save_syn_file(os.path.join(root,'sync_point_cloud_idx.txt'),sync_base_idx)
   