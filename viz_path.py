

import argparse
import numpy as np
import os 
from viz import myplot


def load_to_RAM(file: str):
    r"""
    Loads path (in the file) to RAM memory
    
    """
    assert os.path.isfile(file)
    pose_array = []
    for line in open(file):
        values_str = line.split(' ')
        values = [float(v) for v in values_str]
        pose_array.append(values[0:3])
    return(np.array(pose_array))


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
    parser.add_argument("--pose_file",
                                     #default='/media/tiago/vbig/dataset/FU_Odometry_Dataset/64LiDAR/20190924-143337/pose.txt',
                                     default='20190924-144057/pose.txt',
                                    help = "")
    args = parser.parse_args()

    run1 = args.pose_file

    path1 = load_to_RAM(run1)
    plot_on_gif(path1,'test.gif',record_gif=False,frame_jumps=500)



