

import argparse
import numpy as np
import os,sys 
# Get the current script's directory (script_directory)
script_dir = os.path.dirname(os.path.abspath(__file__))
# Calculate the parent directory of the script (project_directory)
project_dir = os.path.abspath(os.path.join(script_dir, ".."))
sys.path.append(project_dir)

from utils.viz import myplot



def plot_on_gif(pose: np.ndarray, dest_file:str, record_gif: bool =False, frame_jumps: int =1, init_frame: int  = 100):
    
    r"""
     Plots te path online or as gif. 

     Args:
       - pose [nx2] array  
       - dest_file [str] name of the destination file (only used if record_gif is one)
       - record_gif [bool] (default: False) Flag that defines if plot is saved as GIF
       - fame_jump [int] [Hz] Number frames are dropped at each jump
       - init_frame [int] Initial Frame

    """
    
    plot = myplot(delay = 0.01)
    plot.init_plot(pose[:,0],pose[:,1],c='black',s=10)
    plot.xlabel('m')
    plot.ylabel('m')

    if record_gif == True:
        plot.record_gif(dest_file)

    num_samples = pose.shape[0]
    for i in range(init_frame,num_samples,frame_jumps):
        color = np.array(['k']*i)
        color[i-1]='y'
        plot.update_plot(pose[:i,0],pose[:i,1], color = color , offset= 1, zoom=-1)



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = "Convert bag dataset to files!")
    parser.add_argument("--target_dir",default='/media/tiago/vbig/dataset/LBORO-UK/strawberry/june23/extracted')
    parser.add_argument("--file",default='poses.txt')
    args = parser.parse_args()

    target_file = os.path.join(args.target_dir,args.file)

    file = args.file
    print("RUNNING\n")
    target_dir = args.target_dir
    if not os.path.isdir(target_dir):
        print("target is not a directory")
        exit(0)

  
  
    fd = open(target_file,'r')
    lines = fd.readlines()

    line_int = []
    for line in lines:
        # map text to float transformation matrix
        coordinates = [float(value) for value in line.strip().split(' ')]
        coordinates = np.array(coordinates).reshape(4,4)
        # get only translation
        line_int.append(coordinates[:3,3])

    # convert to numpy array
    xyz = np.array(line_int)

    name = file.split('.')[0]
    gif_file  = os.path.join(args.target_dir,f'{name}.gif')
    plot_on_gif(xyz,gif_file,record_gif=True,frame_jumps=50)
    print("Saved file to: " + gif_file)
