import os
import argparse
import os
import numpy as np
import matplotlib.pyplot as plt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = "Convert bag dataset to files!")
    parser.add_argument("--target_dir",default='/home/tiago/Dropbox/SHARE/DATASET/GEORGIA-FR/husky/temp')
    parser.add_argument("--file", default="poses_gps.txt")
    parser.add_argument("--kitti_format",default=True,type=bool,help="Expects that poses.txt file to be kitti format")
    args = parser.parse_args()

    print("RUNNING\n")
    target_dir = args.target_dir
    if not os.path.isdir(target_dir):
        print("target is not a directory")
        exit(0)

    parse_target_path = target_dir.split('/')
    # read files from target dir
    #files = os.listdir(target)
    pose_file = os.path.join(target_dir,args.file)
    assert os.path.isfile(pose_file), "File does not exist: " + pose_file
    
    if args.kitti_format :
        print("Kitti Format is On!")
        fd = open(pose_file,'r')
        lines = fd.readlines()

        line_int = []
        for line in lines:
            # map text to float transformation matrix
            coordinates = [float(value) for value in line.strip().split(' ')]

            if len(coordinates) == 16:
                coordinates = np.array(coordinates).reshape(4,4)
                #coordinates = np.append(coordinates,np.array([0,0,0,1])).reshape(4,4)
                line_int.append(coordinates[:3,3])
            elif len(coordinates) == 3:
                coordinates = np.array(coordinates)
                line_int.append(coordinates[:3])
            else:    
                print("Error: Wrong format in kitti file")
                exit(0)

    # convert to numpy array
    line_int = np.array(line_int)
    
    # save trajectory to png
    file_name = args.file.split('.')[0]
    gps_file  = os.path.join(target_dir,f'{file_name}.png')
    
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
    plt.axis('equal')  # Set equal scale length for both axes
    plt.legend()
    plt.savefig(gps_file, dpi=300, bbox_inches='tight')  # Adjust the file format as needed
    
    print("Saved file to: " + gps_file)
    print("DONE\n")
   

    
