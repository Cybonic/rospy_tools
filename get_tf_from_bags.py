import os
from sre_parse import FLAGS

import numpy as np
import argparse
import rosbag
from tqdm import tqdm
from utils.utils import find_bags
from utils.msgs_utils import NavSatFixLocal,NavMsgsOdometry,pose_np_to_str, transform_to_numpy
import ros_numpy
from tf import transformations

def extract_bag_data(bag_files,topic_dict,dst_dir):

    topic_list = topic_dict['tf']
    frame_list = topic_dict['frame']
    global_transform = np.eye(4)
    frame_idx = 0
    for bag_file in tqdm(bag_files,total=len(bag_files)):
        bag = rosbag.Bag(bag_file)
        
        for i,(topic, msg, t) in enumerate(bag.read_messages(topics=topic_list)):
            #if frame_idx>=len(frame_list)-1:
                #break
        
            for tf in msg.transforms:
                print(f"{tf.header.frame_id} -> {tf.child_frame_id}")
                if tf.header.frame_id == frame_list[0]: #and tf.child_frame_id == frame_list[frame_idx+1]:
                    print(tf.transform.rotation)
                    translation = transform_to_numpy(tf.transform)
                    global_transform = np.dot(global_transform,translation)
                    frame_idx +=1 
                #print(global_transform)
                   
        bag.close()
        
        print(global_transform)
        
        tr_t = global_transform.transpose()
        print("\n")
        print(tr_t)
        
        trans = transformations.translation_from_matrix(tr_t)
        quat = transformations.quaternion_from_matrix(tr_t)
        
        print(trans)
        print(quat)


def parse_bag_name(file):
    return file.split('/')[-1].split('.')[0]

def main_bag_to_file(bag_file,topic_to_read,dst_root):
    extract_bag_data(bag_file,topic_to_read,dst_root)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = "Convert bag dataset to files!")
    parser.add_argument("--bag",default='/media/tiago/vbig/dataset/FU_Odometry_Dataset/rawbags/64/20170303-140708.bag')
    parser.add_argument("--target_bag_dir",default='/media/tiago/vbig/dataset/GEOGIATECH-FR/onfoot')
    parser.add_argument("--pcl_topic",default='/rslidar_points')
    parser.add_argument("--pose_topic",default='/vectornav/GPS_INS')
    parser.add_argument("--dst_root",default='/media/tiago/vbig/dataset/GEOGIATECH-FR/onfoot')
    args = parser.parse_args()
    # /sensors/applanix/gps_odom
    
    topic_to_read = {'tf':'/tf','frame':['imu','laser','odom','base_link']}
    topic_to_read = {'tf':'/tf','frame':['odom','base_link']}
    for folder in os.listdir(args.target_bag_dir):
        target_dir = os.path.join(args.target_bag_dir,folder)
        if not os.path.isdir(target_dir):
            continue
        
        bags = find_bags(target_dir)
        bags = sorted(bags)
        dst_folder = os.path.join(args.dst_root,'extracted',folder)
        
        if not os.path.isdir(dst_folder):
            os.makedirs(dst_folder)
            
        main_bag_to_file(bags,topic_to_read,dst_folder)

    
