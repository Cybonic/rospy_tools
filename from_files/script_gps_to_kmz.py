import os
import argparse
from simplekml import Kml # https://simplekml.readthedocs.io/en/latest/kml.html
import os




if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = "Convert bag dataset to files!")
    parser.add_argument("--target",default='/media/tiago/vbig/dataset/LBORO-UK/orchard-uk/june23/extracted')
    args = parser.parse_args()

    print("RUNNING\n")

    target = args.target
    parse_target_path = target.split('/')

    gps_file  = os.path.join(target,'gps.kmz') 
    kml = Kml(name='trajecotry')

    # read files from target dir
    files = os.listdir(target)

    if 'gps.txt' in files:
        file = os.path.join(target,'gps.txt')
        fd = open(file,'r')
        lines = fd.readlines()

        line_int = []
        for line in lines:
            coordinates = [float(value) for value in line.strip().split(' ')]
            kml.newpoint(name="", coords=[(coordinates[0],coordinates[1],coordinates[2])])  # A simple Point
    
    # Save to kmz
    kml.savekmz(gps_file, format=False)  # Saving as KMZ
        
    print("Saved file to: " + gps_file)
    print("DONE\n")
   

    
