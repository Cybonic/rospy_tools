
import os

def find_bags(target_folder):
    print(target_folder)
    
    bag_files= []
    print("Searching for Bags: \n")
    for bag_file in os.listdir(target_folder):
        
        path_bag_file = os.path.join(target_folder,bag_file)
        if not bag_file.endswith('.bag'):
            continue
        print(bag_file)
        bag_files.append(path_bag_file)
    
    return bag_files