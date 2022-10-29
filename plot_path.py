

import argparse
import numpy as np
import matplotlib.pyplot as plt
import os 

fig, ax = plt.subplots()

def init_plot(x,y,s,c):
    p = ax.scatter(x,y,s = 20, c = 'g')
    return(p)

def update_plot(p,x,y,offset=20,zoom=10):
    global ax 
    p.set_offsets(np.c_[x,y])

    if len(x)<zoom:
        xmax,xmin= max(x),min(x)
        ymax,ymin = max(y),min(y)
    else: 
        xmax,xmin= max(x[-zoom:]),min(x[-zoom:])
        ymax,ymin = max(y[-zoom:]),min(y[-zoom:])

    ax.axis([xmin - offset, xmax + offset, ymin -offset, ymax + offset])
    ax.set_aspect('equal')
    fig.canvas.draw()
    plt.pause(0.001)
    return(p)


def dynamic_plot(x,y,step=10,s = 20, c = 'g'):
    p = init_plot(x[0],y[0],20,'b')
    leng = len(x)
    for i in range(0,leng,step):
        xi,yi = x[:i+1],y[:i+1]
        print(i)
        update_plot(p,xi,yi,offset=20,zoom=0)


def load_to_RAM(file):
    assert os.path.isfile(file)
    pose_array = []
    for line in open(file):
        values_str = line.split(' ')
        values = [float(v) for v in values_str]
        pose_array.append(values[0:3])
    return(np.array(pose_array))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = "Convert bag dataset to files!")

    parser.add_argument("--seq", #default='/home/tiago/Dropbox/research/datasets/orchard-uk/orchard1/pose.txt', 
                                     default='/home/tiago/Dropbox/research/datasets/FUBerlin/toyexample/vehicleA',
                                    help = "")
    args = parser.parse_args()

    #run1 = '/home/tiago/Dropbox/research/datasets/orchard-uk/rerecord_sparce/pose.txt'
    run1 = os.path.join(args.seq,'pose.txt')
    fig, ax = plt.subplots()

    path1 = load_to_RAM(run1)
    #path2 = load_to_RAM(run2)

    #assert len(path1) == len(path2)
    dynamic_plot(path1[:,0],path1[:,1])
    plt.show()


