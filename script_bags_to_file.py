import os
from sre_parse import FLAGS

import numpy as np
import argparse
import rosbag
from tqdm import tqdm
from utils.utils import find_bags
from utils.msgs_utils import transform_np_to_str,NavSatFixLocal,NavMsgsOdometry, vector_to_matrix
from utils.velo_utils import parser,OUSTER_16_STRUCTURE
import ros_numpy
from ros_numpy.geometry import quat_to_numpy
from tf.transformations import quaternion_matrix
import utm
import tf as tf_ros


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




def extract_poses_from_topic(bag_files,topic_dict):

    poses_array = []
    timestamp_array = []
    topic_list = [topic_dict]
    for bag_file in tqdm(bag_files,total=len(bag_files)):
        bag = rosbag.Bag(bag_file)
        
        for topic, msg, t in tqdm(bag.read_messages(topics=topic_list)):

            # Extract pose from 'nav_msgs/Odometry' message
            translation = [msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]
            # quaternion vector (x,y,z,w)
            quaternion  = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
            # convert tf to transformation matrix
            matrix = vector_to_matrix(translation,quaternion)
            poses_array.append(matrix)
            timestamp = t.secs + float(t.nsecs)/1e9
            timestamp_array.append(timestamp)
    
    return np.array(poses_array),np.array(timestamp_array)


def extract_gps_from_topic(bag_files,topic_dict):
    
    gps_array = []
    timestamp_array = []
    topic_list = list([topic_dict])
    for bag_file in tqdm(bag_files,total=len(bag_files)):
        bag = rosbag.Bag(bag_file)
        
        for topic, msg, t in tqdm(bag.read_messages(topics=topic_list)):
            # Extract pose from 'nav_msgs/Odometry' message
            # Extract pose from NavSatFix message
            lat = msg.latitude
            lon = msg.longitude
            alt = msg.altitude
            #pose =np.array(,dtype=np.float64)
            gps_array.append([lon,lat,alt])
            timestamp = t.secs + float(t.nsecs)/1e9
            timestamp_array.append(timestamp)
    
    return np.array(gps_array,dtype=np.float64),np.array(timestamp_array)


def extract_IMU_from_topic(bag_files,topic_dict):
    
    array = []
    timestamp_array = []
    topic_list = [topic_dict]
    for bag_file in tqdm(bag_files,total=len(bag_files)):
        bag = rosbag.Bag(bag_file)
        
        for topic, msg, t in tqdm(bag.read_messages(topics=topic_list)):
        
            rot = quaternion_matrix(quat_to_numpy(msg.orientation))
            array.append(rot)
            timestamp = t.secs + float(t.nsecs)/1e9
            timestamp_array.append(timestamp)
    
    return np.array(array,dtype=np.float64),np.array(timestamp_array)


def extract_ODOM_from_topic(bag_files,topic_dict):
    array = []
    timestamp_array = []
    topic_list = [topic_dict]
    for bag_file in tqdm(bag_files,total=len(bag_files)):
        bag = rosbag.Bag(bag_file)
        
        for topic, msg, t in tqdm(bag.read_messages(topics=topic_list)):
        
            rot = quaternion_matrix(quat_to_numpy(msg.orientation))
            array.append(rot)
            timestamp = t.secs + float(t.nsecs)/1e9
            timestamp_array.append(timestamp)


def extract_data_from_tfs(bags,transform,save_dir,verbose=True):
    
    for key, value in transform.items():
        print(f"Extracting {key} from bag files...")

        data,timesamp = extract_from_tf(bags,value,verbose=verbose)

        print(f"Extracted %d data points"%len(data))

        # Save data to files using the KITTI format
        fd =  open(os.path.join(save_dir,f'{key}.txt'),'w')
        fdt = open(os.path.join(save_dir,f'{key}_timestamp.txt'),'w')
        for i in range(len(data)):
            fd.write(transform_np_to_str(data[i]) + '\n')
            fdt.write(str(timesamp[i]) + '\n')
        
        print(f"Saved {key} data file to {save_dir}")

        fd.close()
        fdt.close()


def conv_gps_to_positions_KITTI_format(data:np.ndarray):
    # Convert lat lon alt to utm
    pose_array = data.copy()
    utm_pose = utm.from_latlon(data[:,1],data[:,0])
    pose_array[:,0] = utm_pose[0]
    pose_array[:,1] = utm_pose[1]
    # Center reference frame at the first gps position
 
    first_pose = pose_array[0,:]
    pose_array = pose_array - first_pose

    return pose_array


def save_positions_KITTI_format(data,timestamp,save_dir):
    """
    Save poses using the KITTI format
    
    Args:
        data (np.array): poses data
        timestamp (np.array): timestamp of each pose
        save_dir (str): directory to save the data
        
    """
    # Save data to files using the KITTI format
    fd =  open(os.path.join(save_dir,f'positions.txt'),'w')
    fdt = open(os.path.join(save_dir,f'positions_timestamp.txt'),'w')


    for i in range(len(data)):
        array_str_list = ' '.join([str(value) for value in data[i]])
        fd.write(array_str_list + '\n')
        fdt.write(str(timestamp[i]) + '\n')
    
    print(f"\nSaved {len(data)} POSITIONS to file at {save_dir}")

    fd.close()
    fdt.close()


def save_gps_KITTI_format(data,timestamp,file):
    """
    Save poses using the KITTI format
    
    Args:
        data (np.array): poses data
        timestamp (np.array): timestamp of each pose
        save_dir (str): directory to save the data
        
    """
    
    # Save data to files using the KITTI format
    fd =  open(file + '.txt','w')
    fdt = open(file + '_timestamp.txt','w')

    for i in range(len(data)):        
        array_str_list = ' '.join([str(value) for value in data[i]])
        fd.write(array_str_list + '\n')
        fdt.write(str(timestamp[i]) + '\n')

    fd.close()
    fdt.close()



def save_poses_KITTI_format(data,timestamp,file):
    """
    Save poses using the KITTI format
    
    Args:
        data (np.array): poses data
        timestamp (np.array): timestamp of each pose
        save_dir (str): directory to save the data
        
    """
    # Save data to files using the KITTI format
    fd =  open(file + '.txt','w')
    fdt = open(file + '_timestamp.txt','w')
    
    for i in range(len(data)):
        fd.write(transform_np_to_str(data[i]) + '\n')
        fdt.write(str(timestamp[i]) + '\n')

    fd.close()
    fdt.close()


def save_imu_KITTI_format(data,timestamp,file):
    """
    Save poses using the KITTI format
    
    Args:
        data (np.array): poses data
        timestamp (np.array): timestamp of each pose
        save_dir (str): directory to save the data
        
    """
    # Save data to files using the KITTI format
    fd =  open(file +'.txt','w')
    fdt = open(file + 'timestamp.txt','w')
    
    for i in range(len(data)):
        fd.write(transform_np_to_str(data[i]) + '\n')
        fdt.write(str(timestamp[i]) + '\n')
    
    print(f"\nSaved {len(data)}  to {file}")

    fd.close()
    fdt.close()



def save_pcd_KITTI_format(data,timestamp,save_dir):
    """
    Save point clouds using the KITTI format

    Args:
        data (np.array): point cloud data
        timestamp (np.array): timestamp of each point cloud
        save_dir (str): directory to save the data
    
    """

    pcd_dir =  save_dir
    os.makedirs(pcd_dir,exist_ok=True)
    fdt = open(save_dir +'timestamp.txt','w')

    for i in range(len(data)):
        name = os.path.join(pcd_dir,'{0:07d}.bin'.format(i))
        pcl = open(name,'wb')
        data[i].tofile(pcl)
        pcl.close()
        fdt.write(str(timestamp[i]) + '\n')
    fdt.close()
    
    print(f"\nSaved {len(data)} PCD to file at {save_dir}")



def extract_from_tf(bag_files,transform,verbose=True):

    assert transform != None, "Transform dict is empty!"
    
    tf_list = list(transform.keys())
    local_tf_list = tf_list.copy()

    data_buffer = []
    time_buffer = []
    
    timestamp = []
    local_tf = {key:[] for key in tf_list} 

    timestamp = {key:[] for key in tf_list}
    for bag_file in tqdm(bag_files,total=len(bag_files)):
        bag = rosbag.Bag(bag_file)
        
        for topic, msg, t in tqdm(bag.read_messages(topics="/tf")):
            # Read tf from bag            
            for tf in msg.transforms:
                if verbose:
                    print(f"\n{tf.header.frame_id} -> {tf.child_frame_id}\n")
                global_tf = np.eye(4)

                for key, tf_instance in transform.items():
                    
                    if tf.header.frame_id == tf_instance['frame_id'] and \
                        tf.child_frame_id == tf_instance['child_frame_id']:
                        
                        if verbose:
                            print(f"---> stored")

                        #if key in local_tf_list:
                        #    local_tf_list.remove(key)
                        # convert tf to matrix
                        # translation vector (x,y,z)
                        translation = [tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z]
                        # quaternion vector (x,y,z,w)
                        quaternion  = [tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w]
                        # convert tf to transformation matrix
                        matrix = vector_to_matrix(translation,quaternion)
                        data_buffer.append(matrix) # Storing tf using respective key (here order is not important) 
                        time_buffer.append(t.secs + float(t.nsecs)/1e9) # Storing tf using respective key (here order is not important) 
 
                
    return np.array(data_buffer),np.array(time_buffer)



def extract_point_cloud_from_topic(bag_files,topic_dict,verbose=False):

    data_buffer = []
    timesamp_buffer = []
    topic_list = list([topic_dict])
    for bag_file in tqdm(bag_files,total=len(bag_files)):
        bag = rosbag.Bag(bag_file)
        
        for topic, msg, t in tqdm(bag.read_messages(topics=topic_list)):
        
            pc_np = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
            data_buffer.append(ouster_parser.velo_sensor(pc_np))
            timesamp_buffer.append(t.secs + float(t.nsecs)/1e9)
    
    return data_buffer,np.array(timesamp_buffer)


def parse_bag_name(file):
    return file.split('/')[-1].split('.')[0]


def timestamp_match(query,reference,verbose=True):
    """
    Compute the timestamp match between query and reference

    Args:
        query (np.array): query timestamps
        reference (np.array): reference timestamps
        verbose (bool, optional): Print error statistics. Defaults to True.

    Returns:
        np.array: nearest indices of the reference array
    """
    from scipy.spatial.distance import cdist
    # point to point distance, where rows are the query points and columns are the reference points
    query = query.reshape(-1,1)
    reference = reference.reshape(-1,1)

    distances = cdist(query,reference, 'euclidean')
    
    nearest_indices = np.argmin(distances, axis=1)
    # Compute error
    if verbose:
        error = abs(query-reference[nearest_indices])
        print("Mean %f"%np.mean(error))
        print("STD %f"%np.std(error))
        print("MAX %f"%np.max(error))
        print("MIN %f"%np.min(error))
    

    return nearest_indices


def check_for_tokens_in_topics(field_names,tokens):

    indice = []
    for i,(field) in enumerate(field_names):
        for token in tokens:
            if str(field).startswith(token):
                indice.append(i)
            
            
    return np.array(indice)
    



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = "Convert bag dataset to files!")
    parser.add_argument("--target_bag_dir",default='/home/tiago/Dropbox/SHARE/DATASET/GEORGIA-FR/husky')
    parser.add_argument("--sync",default=False)
    parser.add_argument("--gps_to_position",default=False)
    parser.add_argument("--dst_root",default=None)
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

            if "tf" in list(data.keys()):
                tfs = data['tf']

    
    
    # read bags from folder
    bags = []

        # Read single bag
    for elem in os.listdir(bag_dir):
        if elem.endswith('.bag'):
            bags.append(os.path.join(bag_dir,elem))
    
    # Sort bags by name
    bags = sorted(bags)

    dest_dir = args.dst_root
    # Create folder to save data
    target_dir = os.path.join(dest_dir,'temp')
    os.makedirs(target_dir,exist_ok=True)

    # Read static tf
    if len(static_tf.keys()) > 0:
        print("Reading static tf...\n")
        file = os.path.join(target_dir,'static_tf.txt')
        get_static_tf(bags,static_tf=static_tf,file=file,verbose=True)


    # Read tf
    if tfs != None:
        print(f"\nExtracting POSES from tfs...")
        poses,poses_timestamp = extract_from_tf(bags,tfs['poses'],verbose=False)
        print("Extracted %d data points"%len(poses))
        #data_extracted['poses'] = True
    
    # =====================
    # Extract data from bags
    # =====================

    modality_token   = {'pcd':['point-cloud','point_cloud','velodyne_points'],
                        'poses':['poses','odom'],
                        'gps':['gps'],
                        'imu':['imu']}
    
    modality_extract_fn     = {'pcd':extract_point_cloud_from_topic,
                        'poses':extract_poses_from_topic,
                        'gps':extract_gps_from_topic,
                        'imu':extract_IMU_from_topic}
    

    modality_save_fn     = {'pcd':save_pcd_KITTI_format,
                        'poses':save_poses_KITTI_format,
                        'gps':save_gps_KITTI_format,
                        'imu':save_imu_KITTI_format}        

    field_names = list(topic_to_read.keys())
    topic_names = list(topic_to_read.values())
    modalities =['pcd','poses','gps','imu']
    data_extracted = {mod:[] for mod in modalities}#'pcd':False, 'poses':False,'gps':False,'point-cloud':False}

    print("Extracting Topics from bags...\n")
    # Extract point cloud from topic
    #check = np.array([[True,i]  for i,(field) in enumerate(field_names) if [str(field).startswith(token) for token in pcd_tokens['tokens'] ].count(True) > 0])
    modality = 'pcd'


    for modality in ['pcd','poses','gps','imu']:
        fn = modality_extract_fn[modality]
        tokens = modality_token[modality]
        print(f"Extracting {modality} from bag files...")
        print(f"Tokens: {tokens}")

        check = check_for_tokens_in_topics(field_names, tokens)
        for idx in check:
            print(f"\nExtracting {field_names[idx]} from bag files...")
            data,timestamp = fn(bags,topic_to_read[field_names[idx]])        
            data_extracted[modality].append({'data':data,'timestamp':timestamp,'field':field_names[idx]})
            print("Extracted %d data points from %s"%(len(data),topic_names[idx]))



    # =====================
    # Data Association
    # Save sync data to files using the KITTI format
    # =====================
    ref_timestamp = None
    modality = 'pcd'
    save_fn = modality_save_fn[modality]
    mod_data = data_extracted[modality]
    for values in mod_data:

        data = values['data']
        timestamp = values['timestamp']
        fieldname = values['field']
        target_file = os.path.join(target_dir,fieldname)
        ref_timestamp = timestamp.copy()
        save_fn(data,timestamp,target_file)
        del data_extracted[modality]

    # =====================
    for modality, mod_data in data_extracted.items():
        save_fn = modality_save_fn[modality]

        if len(mod_data) == 0:
            continue

        for values in mod_data:
            data = values['data']
            timestamp = values['timestamp']
            fieldname = values['field']
            target_file = os.path.join(target_dir,fieldname)
            if args.sync:
                print("\nSync {modality} data with point cloud...")
                nearest_indices = timestamp_match(ref_timestamp,timestamp)
                data = data[nearest_indices]
                timestamp = timestamp[nearest_indices]

            
            # save file
            save_fn(data,timestamp,target_file)
            print(f"\nSaved {len(data)} {modality} to {target_file} ... ")


    info_fd = open(os.path.join(target_dir,'extracted_info.txt'),'w')
    info_fd.write("GPS info structure -> utm data:  lat lon alt\n")
    info_fd.write(f"Synchonized Data (to point cloud): {args.sync}\n")
    info_fd.write("static tf structure -> frame_id child_frame_id x y z yaw pitch roll (radians)\n")
    info_fd.write("poses -> transformation matrix 4x4 a11,a12,a13,a21,a22,a23,a31,a32,a33,0,0,0,1\n")
    info_fd.close()
   

