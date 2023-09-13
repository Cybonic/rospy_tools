import os
from sre_parse import FLAGS

import numpy as np
import argparse
import rosbag
from tqdm import tqdm
from utils.utils import find_bags
from utils.msgs_utils import transform_np_to_str,NavSatFixLocal,NavMsgsOdometry, vector_to_matrix
import ros_numpy

import tf as tf_ros
import math


def get_static_tf(bag_files,static_tf, file = 'static_tf.txt', verbose = False):

    fd = open(file,'w')
    fd.write("frame_id child_frame_id x y z yaw pitch roll (radians)\n")

    keys_tfs = list(static_tf.keys())
    
    for bag_file in tqdm(bag_files,total=len(bag_files)):
        bag = rosbag.Bag(bag_file)
        for i,(topic, msg, t) in enumerate(bag.read_messages(topics="/tf_static")):

            for key, static_tf_instance in static_tf.items():
                if msg.transforms[0].header.frame_id == static_tf_instance['frame_id'] and \
                    msg.transforms[0].child_frame_id == static_tf_instance['child_frame_id']:
                    keys_tfs.remove(key)
                    
                    # quaternion vector (x,y,z,w)
                    quaternion = [  msg.transforms[0].transform.rotation.x,
                                    msg.transforms[0].transform.rotation.y,
                                    msg.transforms[0].transform.rotation.z,
                                    msg.transforms[0].transform.rotation.w]
                    
                    # Convert quaternion to Euler angles (roll, pitch, yaw) radians
                    euler = tf_ros.transformations.euler_from_quaternion(quaternion)

                    line = [
                            msg.transforms[0].header.frame_id,
                            msg.transforms[0].child_frame_id,
                            msg.transforms[0].transform.translation.x,
                            msg.transforms[0].transform.translation.y,
                            msg.transforms[0].transform.translation.z,
                            euler[2], # yaw
                            euler[1], # pitch
                            euler[0], # roll
                            ]
                            
                    line_str = [str(elem) for elem in line]
                    fd.write(" ".join(line_str) + '\n')
                    
                    if verbose:
                        print("="*50)
                        print("Found static tf:")
                        print(line_str)
                        print("="*50)

        # Check if all static tfs were found
        if len(keys_tfs) == 0:
            # close file
            fd.close()
            break

        bag.close()







def extract_bag_data(bag_files,topic_dict,dst_dir,transform=None,verbose=False):
    

    topic_list = list(topic_dict.values())

    # Get keys from transform dict
    tf_key_list = []
    if transform != None:
        tf_key_list = list(transform.keys())


    # Create folder to save Point Clouds
    velodyne_dir =  os.path.join(dst_dir,'point_cloud')
    if not os.path.isdir(velodyne_dir):
        os.makedirs(velodyne_dir)
        print("Created folder: %s"%velodyne_dir)

    # Save point cloud timestamps
    velo_times_file = os.path.join(dst_dir,'point_cloud_times.txt')

    # Save pose
    pose_file = os.path.join(dst_dir,'gps.txt')

    # Save pose timestamps
    pose_times_file = os.path.join(dst_dir,'gps_times.txt')
    
    # open file descriptors
    ft  = open(pose_times_file,'w')
    fvt = open(velo_times_file,'w')
    fp  = open(pose_file,'w')
    fp.write("utm lat lon alt\n")

    tf_fds = {key:open(os.path.join(dst_dir,key+'.txt'),'w') for key in tf_key_list}
    
    # Deisplay topics to read
    if verbose:
        print("="*50)
        print("Topics to read:")
        print(topic_list)
        print("="*50)

    pose_counter     = 0
    velodyne_counter = 0 

    pcl_sync_sample = {'sync':0}
    pose_sync_sample = {'sync':0}
    imy_sync_sample = {'sync':0}
    delta_msecs_buffer = []
    odom_sync_sample = {'sync':0}
    
    sync = 0
    local_tf_key_list = tf_key_list.copy()
    
    for bag_file in tqdm(bag_files,total=len(bag_files)):
        bag = rosbag.Bag(bag_file)
        
        for topic, msg, t in tqdm(bag.read_messages(topics=topic_list)):
            #print(topic)
            if topic == topic_dict['gps'] and  'NavSatFix'  in msg._type:

                # Extract pose from NavSatFix message
                pose = NavSatFixLocal(msg)
                pose_sync_sample['pose'] = pose
                pose_sync_sample['t']    = t
                pose_sync_sample['sync'] = 1

            elif topic == topic_dict['point-cloud']:
                 
                # Extract point cloud from sensor_msgs/PointCloud2 message
                pc_np = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
                pcl_sync_sample['pcl']  = pc_np
                pcl_sync_sample['t']    = t
                pcl_sync_sample['sync'] = 1
                
            elif len(local_tf_key_list) > 0 and topic == topic_dict['tf']: 
                
                for tf in msg.transforms:
                    if verbose:
                        print(f"{tf.header.frame_id} -> {tf.child_frame_id}")

                    for key, tf_instance in transform.items():
                        
                        if tf.header.frame_id == tf_instance['frame_id'] and \
                            tf.child_frame_id == tf_instance['child_frame_id']:
                            local_tf_key_list.remove(key)
                            # convert tf to matrix
                            # translation vector (x,y,z)
                            translation = [tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z]
                            # quaternion vector (x,y,z,w)
                            quaternion  = [tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w]
                            # convert tf to transformation matrix
                            matrix = vector_to_matrix(translation,quaternion)
                            
                            odom_sync_sample[key] = {'t':t,'tf':matrix,'frame_id':tf.header.frame_id,'child_frame_id':tf.child_frame_id}
                            
                            odom_sync_sample['sync'] = 1
            
            elif 'imu' in topic and topic == topic_dict['imu']:
                from ros_numpy.geometry import quat_to_numpy
                from tf.transformations import quaternion_matrix
                
                rot = quaternion_matrix(quat_to_numpy(msg.orientation))
                imy_sync_sample['sync'] = 1
                imy_sync_sample['data'] = rot
                imy_sync_sample['t'] = t
                sync +=1

            # Check if all topics are sync                
            if pcl_sync_sample['sync'] and \
                odom_sync_sample['sync'] and \
                    pose_sync_sample['sync']: #and imy_sync_sample['sync']:
                
                local_tf_key_list = tf_key_list.copy()

                # Save point cloud
                scan = pcl_sync_sample['pcl'].copy()
                pc_np = np.stack((scan['x'],scan['y'],scan['z'],np.ones_like(scan['z'])),axis=1).transpose()
                scan['x'] = pc_np[0]
                scan['y'] = pc_np[1]
                scan['z'] = pc_np[2]
                
                # name of point cloud file
                name = os.path.join(velodyne_dir,'{0:07d}.bin'.format(velodyne_counter))
                pcl = open(name,'wb')
                scan.tofile(pcl)
                pcl.close()

                # Save PCL timestamp
                fvt.write(str(pcl_sync_sample['t']) + '\n')
                velodyne_counter+=1

                # Save GPS
                pose_array = transform_np_to_str(pose_sync_sample['pose'],precision=3)
                fp.write(pose_array + '\n')
                ft.write(str(pose_sync_sample['t']) + '\n')
                pose_counter +=1

                # Save tfs
                for label, fd in tf_fds.items():
                    tf = odom_sync_sample[label]['tf']
                    tf_array = transform_np_to_str(tf,precision=3)
                    fd.write(tf_array + '\n')

                # Reset sync
                pose_sync_sample['sync'] = 0
                pcl_sync_sample['sync']  = 0
                imy_sync_sample['sync']  = 0
                odom_sync_sample['sync'] = 0

                # Compute delta time
                delta = pcl_sync_sample['t'] - odom_sync_sample['odom']['t']
                delta = float(delta.secs + delta.nsecs)/1000000
                delta_msecs_buffer.append(delta)
        bag.close()
        
    ft.close()
    fp.close()
    fvt.close()

    # close tf file descriptors
    for label, fd in tf_fds.items():
        fd.close()

    if verbose:
        print("="*50)
        
    delta_msecs_buffer = np.array(delta_msecs_buffer)

    print("time_pcl - time_pose [ms] mean: %f std: %f"%(delta_msecs_buffer.mean(),delta_msecs_buffer.std()))
    print("time_pcl - time_pose [ms] max: %f min: %f"%(delta_msecs_buffer.max(),delta_msecs_buffer.min()))
    print(f'pose: {pose_counter}  velodyne: {velodyne_counter}')
    print("="*50)

def parse_bag_name(file):
    return file.split('/')[-1].split('.')[0]

def main_bag_to_file(bag_file,topic_to_read,dst_root,tf,static_tf):
   
    # Extract data
    extract_bag_data(bag_file,topic_to_read,dst_root,tf)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = "Convert bag dataset to files!")
    parser.add_argument("--bag",default='/media/tiago/vbig/dataset/FU_Odometry_Dataset/rawbags/64/20170303-140708.bag')
    parser.add_argument("--target_bag_dir",default='/media/tiago/vbig/dataset/LBORO-UK/strawberry/june23')
    parser.add_argument("--pcl_topic",default='/velodyne_points')
    parser.add_argument("--pose_topic",default='/antobot_gps')
    parser.add_argument("--dst_root",default='/media/tiago/vbig/dataset/LBORO-UK/strawberry/june23')
    parser.add_argument("--multibag",default=False)
    args = parser.parse_args()
    # /sensors/applanix/gps_odom
    

    topic_to_read = {'point-cloud':args.pcl_topic,
                     'gps':args.pose_topic,
                     'tf':'/tf'
                     }
    
    static_tf ={
        'velodyne':{
            'frame_id':'base_link',
            'child_frame_id':'velodyne'
            }
        }
    
    tf ={
        'odom':{
            'frame_id':'odom',
            'child_frame_id':'base_link'
            }
        }
    
    
    # read bags from folder
    if args.multibag == True:
        for folder in os.listdir(args.target_bag_dir):
            target_dir = os.path.join(args.target_bag_dir,folder)
            if not os.path.isdir(target_dir):
                continue
            bags = find_bags(target_dir)
            bags = sorted(bags)
            
    else:
        # Read single bag
        for elem in os.listdir(args.target_bag_dir):
            if elem.endswith('.bag'):
                bags = [os.path.join(args.target_bag_dir,elem)]

    # Create folder to save data
    dst_folder = os.path.join(args.dst_root,'extracted')       
    os.makedirs(dst_folder,exist_ok=True)
    


    # Read static tf
    file = os.path.join(dst_folder,'static_tf.txt')
    get_static_tf(bags,static_tf=static_tf,file=file,verbose=True)


    # Extract data
    main_bag_to_file(bags,topic_to_read,dst_folder,tf,static_tf)

    
