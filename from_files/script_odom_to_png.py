import os
import argparse
import os
import numpy as np
import matplotlib.pyplot as plt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = "Convert bag dataset to files!")
    parser.add_argument("--target",default='/media/tiago/vbig/dataset/LBORO-UK/orchard-uk/june23/extracted')
    args = parser.parse_args()

    print("RUNNING\n")
    target = args.target
    if not os.path.isdir(target):
        print("target is not a directory")
        exit(0)

    parse_target_path = target.split('/')
    # read files from target dir
    files = os.listdir(target)

    if 'odom.txt' in files:
        file = os.path.join(target,'odom.txt')
        fd = open(file,'r')
        lines = fd.readlines()

        line_int = []
        for line in lines:
            # map text to float transformation matrix
            coordinates = [float(value) for value in line.strip().split(' ')]
            coordinates = np.array(coordinates).reshape(3,4)
            coordinates = np.append(coordinates,np.array([0,0,0,1])).reshape(4,4)
            line_int.append(coordinates[:3,3])

    # convert to numpy array
    line_int = np.array(line_int)
    
    # save trajectory to png
    gps_file  = os.path.join(target,'odom.png')
    
    # save numpy array to image png
    x = line_int[:,0]
    y = line_int[:,1]

    # Create a 2D path plot
    plt.figure(figsize=(8, 6))  # Optional: Set the figure size
    plt.plot(x, y, marker='o', linestyle='-', color='b', label='Path')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('2D Path Plot')
    plt.grid(True)
    plt.legend()
    plt.savefig(gps_file, dpi=300, bbox_inches='tight')  # Adjust the file format as needed

    print("Saved file to: " + gps_file)
    print("DONE\n")
   

    
