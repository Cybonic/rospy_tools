import os 
import sys
import yaml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.pyplot as plt
from utils.utils import *
from sklearn.neighbors import NearestNeighbors
from mpl_toolkits.mplot3d import Axes3D
from tqdm import tqdm
import argparse
import loop_corr_config as config
from scipy.spatial import distance
from oxford_lib.image import load_image 
from sklearn.neighbors import NearestNeighbors
import cv2
import laserscanvis 



PLOTTING_FLAG = True

TWIDTH = 300 
THIGHT = 240 

SYNC_OFFSET = 0

font                   = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10,50)
fontScale              = 2
fontColor              = (0,0,0)
lineType               = 2

#fig, ax = plt.subplots()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')



def process_img(img,w,h,txt= ''):
    img = cv2.putText(img,txt , bottomLeftCornerOfText, font, fontScale, fontColor,lineType)
    img = cv2.resize(img, (w, h))
    
    return(img)

def init_plot(x,y,s,c):
    p = ax.scatter(x,y,s = 20, c = 'g')
    return(p)

def update_plot(p,x,y,z,offset=20,zoom=10):

    global ax 
    xs,ys,zs = p._offsets3d 

    xs = np.append(xs,x)
    ys = np.append(ys,y)
    zs = np.append(zs,z)
    #x,y,z = newxy[:,0],newxy[:,1],newxy[:,2]
    p._offsets3d = (xs,ys,zs)

    # ax.add_collection3d(p)
    #p.set_offsets(newxy)

    #ax.axis([xmin - offset, xmax + offset, ymin -offset, ymax + offset])

    fig.canvas.draw()
    plt.show(block=False)
    plt.pause(0.001)

    return(p)


def generate_plot_img():
    buf = fig.canvas.tostring_rgb()
    ncols, nrows = fig.canvas.get_width_height()
    pose_img = np.frombuffer(buf, dtype=np.uint8).reshape(nrows, ncols, 3)
    return(pose_img)

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Play back images from a given directory')
    parser.add_argument('--root', type=str, default='')
    parser.add_argument('--dataset', default = 'oxford', type=str,help=' dataset root directory .')
    parser.add_argument('--seq',default  = '2014-05-06-13-09-52',type = str)
    parser.add_argument('--plot',default  = 1 ,type = int)

    args = parser.parse_args()

    dataset_name = args.dataset 
    root = args.root
    seq = args.seq
    plotting_flag = args.plot

    print("[INF] Dataset Name:  " + dataset_name )
    print("[INF] Sequence Name: " + seq )
    print("[INF] Plotting Flag: " + str(plotting_flag) )
    # Build loop correspondess of query and reference  paths

    # Synchronization of images with poses
    working_path = os.path.dirname(os.path.abspath(__file__))
    config_file = os.path.join(working_path,'dataset_config.yaml')
    # Load Yaml file with dataset specific directory structure
    dataset_structures = yaml.load(open(config_file),Loader=yaml.FullLoader)
    structures = dataset_structures[dataset_name]

    dataset_pointer = utility_mapper[dataset_name]
    dataset = dataset_pointer.dataset(structures['root'],seq)
    

    pclt = laserscanvis.pointcloud(dataset)
    laser = laserscanvis.LaserScanVis(pclt)
    laser.run()