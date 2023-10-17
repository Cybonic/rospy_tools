import os
from sre_parse import FLAGS

import numpy as np
import argparse
import rosbag
from tqdm import tqdm
from utils.utils import find_bags
from utils.msgs_utils import transform_np_to_str,NavSatFixLocal,NavMsgsOdometry, vector_to_matrix
from utils.velo_utils import np_pts_to_pcd,parser,OUSTER_16_STRUCTURE
import ros_numpy

import tf as tf_ros
import math


ouster_parser = parser(OUSTER_16_STRUCTURE)

def get_static_tf(bag_files,static_tf, file = 'static_tf.txt', verbose = False):

    fd = open(file,'w')
    fd.write("frame_id child_frame_id x y z yaw pitch roll (radians)\n")

    keys_tfs = list(static_tf.keys())
    
    for bag_file in tqdm(bag_files,total=len(bag_files)):
        bag = rosbag.Bag(bag_file)
        for i,(topic, msg, t) in enumerate(bag.read_messages(topics=["/tf","/tf_static"])):
            #print(f"{msg.transforms[0].header.frame_id} -> {msg.transforms[0].child_frame_id}")
            for key, static_tf_instance in static_tf.items():
                
                if msg.transforms[0].header.frame_id == static_tf_instance['frame_id'] and \
                    msg.transforms[0].child_frame_id == static_tf_instance['child_frame_id']:
                    if len(keys_tfs) == 0:
                        break
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

    data_list = list(topic_dict.keys())

    # Get keys from transform dict
    tf_key_list = []
    if transform != None:
        tf_key_list = list(transform.keys())

    # Create a data structure to save data from topics
    # fied -> {data, t, sync, fd, fdt}
    # data -> data from topic
    # t -> timestamp
    # sync -> flag to check if topic is sync
    # fd -> file descriptor to save data
    # fdt -> file descriptor to save timestamp
    # cntr -> counter that tracks the number of data points
    topic_tracker = {field:{'data':{},'t':[],'sync':False,'fd':[],'fdt':[],'cntr':0} for field in data_list}
    
    if 'point-cloud' in data_list:
        # Create folder to save Point Clouds
        velodyne_dir =  os.path.join(dst_dir,'point_cloud')
        if not os.path.isdir(velodyne_dir):
            os.makedirs(velodyne_dir)
            print("Created folder: %s"%velodyne_dir)
        # Save point cloud timestamps    
        topic_tracker['point-cloud']['fdt'] = open(os.path.join(dst_dir,'point_cloud_timestamp.txt'),'w')
    
    for sensor in data_list:
        topic_tracker[sensor]['fd'] =  open(os.path.join(dst_dir,f'{sensor}.txt'),'w')
        topic_tracker[sensor]['fdt'] = open(os.path.join(dst_dir,f'{sensor}_timestamp.txt'),'w')

    # Save GPS data
    #if 'gps' in   data_list:
    #    topic_tracker['gps']['fd'] =  open(os.path.join(dst_dir,'gps.txt'),'w')
    #    topic_tracker['gps']['fdt'] = open(os.path.join(dst_dir,'gps_timestamp.txt'),'w')
    
    # Save Poses
    #if 'poses' in   data_list:
    #    topic_tracker['poses']['fd']  = open(os.path.join(dst_dir,'poses.txt'),'w')
    #    topic_tracker['poses']['fdt'] = open(os.path.join(dst_dir,'poses_timestamp.txt'),'w')
 
    # Deisplay topics to read
    #if verbose:
    #    print("="*50)
    #    print("Topics to read:")
    #    print(topic_list)
    #    print("="*50)


    delta_msecs_buffer = []

    local_tf_key_list = tf_key_list.copy()
    
    for bag_file in tqdm(bag_files,total=len(bag_files)):
        bag = rosbag.Bag(bag_file)
        
        for topic, msg, t in tqdm(bag.read_messages(topics=topic_list)):
            #print(topic)
            if topic in topic_list and  'NavSatFix'  in msg._type and topic == topic_dict['gps']:

                # Extract pose from NavSatFix message
                lat = msg.latitude
                lon = msg.longitude
                alt = msg.altitude
                pose =np.array([lon,lat,alt])
                topic_tracker['gps']['data'] = pose
                topic_tracker['gps']['t']    = t
                topic_tracker['gps']['sync'] = 1
                topic_tracker['gps']['cntr'] += 1

            elif topic in topic_list and topic == topic_dict['point-cloud']:
                # Extract point cloud from sensor_msgs/PointCloud2 message
                pc_np = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
                topic_tracker['point-cloud']['data']  = ouster_parser.velo_sensor(pc_np)
                topic_tracker['point-cloud']['t']    = t
                topic_tracker['point-cloud']['sync'] = 1
                topic_tracker['point-cloud']['cntr'] += 1

            elif 'pose' in topic_dict and topic in topic_list and topic == topic_dict['poses']:
                # Extract pose from 'nav_msgs/Odometry' message
                translation = [msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]
                # quaternion vector (x,y,z,w)
                quaternion  = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
                            # convert tf to transformation matrix
                matrix = vector_to_matrix(translation,quaternion)
                topic_tracker['poses']['data'] = matrix
                topic_tracker['poses']['sync'] = 1
                topic_tracker['poses']['cntr'] += 1
                topic_tracker['poses']['t'] = t


            elif len(local_tf_key_list) > 0 and topic in topic_list and topic == topic_dict['poses']: 
                
                for tf in msg.transforms:
                    if verbose:
                        print(f"{tf.header.frame_id} -> {tf.child_frame_id}")

                    for key, tf_instance in transform.items():
                        
                        if tf.header.frame_id == tf_instance['frame_id'] and \
                            tf.child_frame_id == tf_instance['child_frame_id']:
                            
                            if key in local_tf_key_list:
                                local_tf_key_list.remove(key)
                            # convert tf to matrix
                            # translation vector (x,y,z)
                            translation = [tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z]
                            # quaternion vector (x,y,z,w)
                            quaternion  = [tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w]
                            # convert tf to transformation matrix
                            matrix = vector_to_matrix(translation,quaternion)
                            topic_tracker['poses']['data'][key] = {'t':t,'tf':matrix,'frame_id':tf.header.frame_id,'child_frame_id':tf.child_frame_id}
                            
                        if len(local_tf_key_list) == 0:
                            topic_tracker['poses']['sync'] = 1
                            topic_tracker['poses']['cntr'] += 1
                            break
            

            elif 'imu' in data_list and topic_dict['imu'] == topic:
                from ros_numpy.geometry import quat_to_numpy
                from tf.transformations import quaternion_matrix
                
                rot = quaternion_matrix(quat_to_numpy(msg.orientation))
                topic_tracker['imu']['data'] = rot
                topic_tracker['imu']['sync'] = 1
                topic_tracker['imu']['cntr'] += 1
                topic_tracker['imu']['t'] = t
            
            all_sync = np.all([value['sync'] for key,value in topic_tracker.items()])
            # Check if all topics are sync                
            if all_sync:
                
                # Reset tf key list
                if len(tf_key_list)>0:
                    local_tf_key_list = tf_key_list.copy()

                # Save point clouds
                if 'point-cloud' in data_list:
                    scan = topic_tracker['point-cloud']['data'].copy()
                    
                    name = os.path.join(velodyne_dir,'{0:07d}.bin'.format(topic_tracker['point-cloud']['cntr']))
                    pcl = open(name,'wb')
                    scan.tofile(pcl)
                    pcl.close()
                    
                    timestamp = topic_tracker['point-cloud']['t'].secs + topic_tracker['point-cloud']['t'].nsecs
                    topic_tracker['point-cloud']['fdt'].write(str(timestamp) + '\n')
                    topic_tracker['point-cloud']['t'] = timestamp
                    topic_tracker['point-cloud']['sync'] = 0
                    topic_tracker['point-cloud']['data'] = {}

                # Save GPS
                if 'gps' in data_list:
                    pose = transform_np_to_str(topic_tracker['gps']['data'].copy())
        
                    timestamp = topic_tracker['gps']['t'].secs + topic_tracker['gps']['t'].nsecs
                    topic_tracker['gps']['fd'].write(pose + '\n')
                    topic_tracker['gps']['fdt'].write(str(timestamp) + '\n')
                    topic_tracker['gps']['t'] = timestamp
                    topic_tracker['gps']['sync'] = 0
                    topic_tracker['gps']['data'] = {}
                
                if 'imu' in data_list:
                    pose = transform_np_to_str(topic_tracker['imu']['data'].copy())
        
                    timestamp = topic_tracker['imu']['t'].secs + topic_tracker['imu']['t'].nsecs
                    topic_tracker['imu']['fd'].write(pose + '\n')
                    topic_tracker['imu']['fdt'].write(str(timestamp) + '\n')
                    topic_tracker['imu']['t'] = timestamp
                    topic_tracker['imu']['sync'] = 0
                    topic_tracker['imu']['data'] = {}

                # Save Poses
                if 'poses' in data_list:
                    # Save Global tf
                    tf_timestamp_list = []
                    global_tf = np.eye(4)
                    
                    if len(tf_key_list)>0:
                        # compute transformation matrix
                        for key in local_tf_key_list:
                            tf = topic_tracker['poses']['data'][key]['tf']
                            global_tf = np.dot(global_tf,tf) #= np.matmul(odom_sync_sample[key]['tf'],imy_sync_sample['data'])
                            tf_timestamp = topic_tracker['poses']['data'][key]['t']
                            tf_timestamp_list.append(tf_timestamp.secs + tf_timestamp.nsecs)
                    else:
                        global_tf = topic_tracker['poses']['data'].copy()
                        tf_timestamp_list.append(topic_tracker['poses']['t'].secs + topic_tracker['poses']['t'].nsecs)    
                    
                    # Save timestamp
                    pose_timestamp = np.mean(tf_timestamp_list,dtype=np.uint32)
                    topic_tracker['poses']['t'] = pose_timestamp
                    topic_tracker['poses']['fdt'].write(str(pose_timestamp) + '\n')

                    # Save global transformation matrix
                    tf_array = transform_np_to_str(global_tf,precision=3)
                    topic_tracker['poses']['fd'].write(tf_array + '\n')

                    # Reset data
                    topic_tracker['poses']['sync'] = 0
                    topic_tracker['poses']['data'] = {}

                # Compute delta time
                delta = topic_tracker['point-cloud']['t'] - topic_tracker[data_list[1]]['t']
                delta = float(delta)/1000000
                delta_msecs_buffer.append(delta)
        bag.close()
    
    
    # Close files
    for key,value in topic_tracker.items():
        if value['fd'] != []:
            value['fd'].close()
        if value['fdt'] != []:
            value['fdt'].close()

    return np.array(delta_msecs_buffer),topic_tracker['point-cloud']['cntr']

def parse_bag_name(file):
    return file.split('/')[-1].split('.')[0]



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = "Convert bag dataset to files!")
    parser.add_argument("--target_bag_dir",default='/home/tiago/Dropbox/SHARE/DATASET/uk/orchards/june23')
    parser.add_argument("--pcl_topic",default='/husky/velodyne_points')
    parser.add_argument("--pose_topic",default='/odom')
    parser.add_argument("--gps_topic",default='/mavros/global_position/raw/fix')
    parser.add_argument("--dst_root",default=None)
    parser.add_argument("--multibag",default=False)
    parser.add_argument("--from_file",default="topics2read.yaml")

    args = parser.parse_args()
    # /sensors/applanix/gps_odom

    if args.dst_root == None:
        args.dst_root = args.target_bag_dir

    bag_dir = os.path.join(args.target_bag_dir,'bagfiles')
    print("Reading bags from folder: %s"%bag_dir)
    topic_to_read = {}
    static_tf = {}
    tfs = None
    # Read topics to read  from yaml file
    if args.from_file != None:
        import yaml
        file = os.path.join(bag_dir,args.from_file)
        print("Reading topics from file: %s"%file)
        with open(file) as f:
            data = data = yaml.safe_load(f)
            
            if 'topics' in list(data.keys()):
                topic_to_read = data['topics']

            if 'static_tf' in list(data.keys()):
                static_tf = data['static_tf']
    else:
        # Read topics to read from args
        topic_to_read = {'point-cloud':args.pcl_topic,
                        'gps':args.gps_topic,
                        'poses':args.pose_topic
                        }
        
        static_tf ={
            'velodyne':{
                'frame_id':'base_link',
                'child_frame_id':'os_sensor'
                },
            'gps':{
                'frame_id':'base_link',
                'child_frame_id':'navsat_link'
                }
            }
    
        #tfs = None
        tfs ={
            'map':{
                'frame_id':'map',
                'child_frame_id':'odom'
                },
            'odom':{
                'frame_id':'odom',
                'child_frame_id':'base_link'
                },
            }
    
    # read bags from folder
    bags = []

        # Read single bag
    for elem in os.listdir(bag_dir):
        if elem.endswith('.bag'):
            bags.append(os.path.join(bag_dir,elem))
    
    bags = sorted(bags)

    # Create folder to save data
    dst_folder = os.path.join(args.dst_root,'extracted')       
    os.makedirs(dst_folder,exist_ok=True)

    # Read static tf
    print("Reading static tf...\n")
    file = os.path.join(dst_folder,'static_tf.txt')
    get_static_tf(bags,static_tf=static_tf,file=file,verbose=True)

    print("Extracting Topics from bags...\n")
    # Extract data  
    delta_msecs_buffer,counter =  extract_bag_data(bags,topic_to_read,dst_folder)
    #delta_msecs_buffer,counter = main_bag_to_file(bags,topic_to_read,dst_folder,tfs)

    info_fd = open(os.path.join(dst_folder,'extracted_info.txt'),'w')
    info_fd.write("GPS info structure -> utm data:  lat lon alt\n")
    info_fd.write("static tf structure -> frame_id child_frame_id x y z yaw pitch roll (radians)\n")
    info_fd.write("poses -> transformation matrix 4x4 a11,a12,a13,a21,a22,a23,a31,a32,a33,0,0,0,1\n")
    
    if tfs != None:
        for label, tf in tfs.items():
            info_fd.write("tf %s %s->%s\n"%(label,tf['frame_id'],tf['child_frame_id']))
   
    info_fd.write("timestamps [ms]\n")
    info_fd.write("time_pcl - time_pose [ms] mean: %f std: %f\n"%(delta_msecs_buffer.mean(),delta_msecs_buffer.std()))
    info_fd.write("time_pcl - time_pose [ms] max: %f min: %f\n"%(delta_msecs_buffer.max(),delta_msecs_buffer.min()))
    info_fd.write('data points counter : %d\n'%(counter))
    info_fd.close()
   

