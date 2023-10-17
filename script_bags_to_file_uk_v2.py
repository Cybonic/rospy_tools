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


def save_gps_KITTI_format(data,timestamp,save_dir):
    """
    Save poses using the KITTI format
    
    Args:
        data (np.array): poses data
        timestamp (np.array): timestamp of each pose
        save_dir (str): directory to save the data
        
    """
    # Save data to files using the KITTI format
    fd =  open(os.path.join(save_dir,f'gps.txt'),'w')
    fdt = open(os.path.join(save_dir,f'gps_timestamp.txt'),'w')
    
    for i in range(len(data)):
        array_str_list = ' '.join([str(value) for value in data[i]])
        fd.write(array_str_list + '\n')
        fdt.write(str(timestamp[i]) + '\n')
    
    print(f"\nSaved {len(data)} GPS to file at {save_dir}")

    fd.close()
    fdt.close()


def save_poses_KITTI_format(data,timestamp,save_dir):
    """
    Save poses using the KITTI format
    
    Args:
        data (np.array): poses data
        timestamp (np.array): timestamp of each pose
        save_dir (str): directory to save the data
        
    """
    # Save data to files using the KITTI format
    fd =  open(os.path.join(save_dir,f'poses.txt'),'w')
    fdt = open(os.path.join(save_dir,f'poses_timestamp.txt'),'w')
    
    for i in range(len(data)):
        fd.write(transform_np_to_str(data[i]) + '\n')
        fdt.write(str(timestamp[i]) + '\n')
    
    print(f"\nSaved {len(data)} POSES to file at {save_dir}")

    fd.close()
    fdt.close()


def save_imu_KITTI_format(data,timestamp,save_dir):
    """
    Save poses using the KITTI format
    
    Args:
        data (np.array): poses data
        timestamp (np.array): timestamp of each pose
        save_dir (str): directory to save the data
        
    """
    # Save data to files using the KITTI format
    fd =  open(os.path.join(save_dir,f'imu.txt'),'w')
    fdt = open(os.path.join(save_dir,f'imu_timestamp.txt'),'w')
    
    for i in range(len(data)):
        fd.write(transform_np_to_str(data[i]) + '\n')
        fdt.write(str(timestamp[i]) + '\n')
    
    print(f"\nSaved {len(data)} IMU to file at {save_dir}")

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

    pcd_dir =  os.path.join(save_dir,"point_cloud")
    os.makedirs(pcd_dir,exist_ok=True)
    fdt = open(os.path.join(save_dir,'point_cloud_timestamp.txt'),'w')

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
    local_tf = {} 
    for bag_file in tqdm(bag_files,total=len(bag_files)):
        bag = rosbag.Bag(bag_file)
        
        for topic, msg, t in tqdm(bag.read_messages(topics="/tf")):
            # Read tf from bag            
            for tf in msg.transforms:
                if verbose:
                    print(f"\n{tf.header.frame_id} -> {tf.child_frame_id}\n")

                for key, tf_instance in transform.items():
                    
                    if tf.header.frame_id == tf_instance['frame_id'] and \
                        tf.child_frame_id == tf_instance['child_frame_id']:
                        
                        if verbose:
                            print(f"---> stored")

                        if key in local_tf_list:
                            local_tf_list.remove(key)
                        # convert tf to matrix
                        # translation vector (x,y,z)
                        translation = [tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z]
                        # quaternion vector (x,y,z,w)
                        quaternion  = [tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w]
                        # convert tf to transformation matrix
                        matrix = vector_to_matrix(translation,quaternion)
                        local_tf[key] = matrix # Storing tf using respective key (here order is not important) 
                        timestamp.append(t.secs + float(t.nsecs)/1e9)

                        if len(local_tf_list) == 0:
                            # All tfs were found
                            break

            # Check if all tfs were found
            if len(local_tf_list) == 0:
                # compute transformation matrix
                # compute the global tf using the order provided in TF_LIST
                global_tf = np.eye(4)
                for key in tf_list:
                    global_tf = np.dot(global_tf,local_tf[key] )
                # Save global transformation matrix
                data_buffer.append(global_tf)
                # Save timestamp, mean of all timestamps
                time_buffer.append(np.mean(timestamp))
                # Reset buffers
                local_tf_list = tf_list.copy()
                timestamp = []
                local_tf = {} 
                
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

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = "Convert bag dataset to files!")
    parser.add_argument("--target_bag_dir",default='/home/tiago/Dropbox/SHARE/DATASET/uk/strawberry/june23')
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

            if "tf" in list(data.keys()):
                tfs = data['tf']

    
    # read bags from folder
    bags = []

        # Read single bag
    for elem in os.listdir(bag_dir):
        if elem.endswith('.bag'):
            bags.append(os.path.join(bag_dir,elem))
    
    bags = sorted(bags)

    dest_dir = args.dst_root
    # Create folder to save data

    target_dir = os.path.join(dest_dir,'temp')
    os.makedirs(target_dir,exist_ok=True)

    # Read static tf
    print("Reading static tf...\n")
    file = os.path.join(target_dir,'static_tf.txt')
    get_static_tf(bags,static_tf=static_tf,file=file,verbose=True)


    # =====================
    # Extract data from bags
    # =====================

    data_extracted = {'poses':False,'gps':False,'point-cloud':False}

    print("Extracting Topics from bags...\n")
    # Extract point cloud from topic
    if 'point-cloud' in topic_to_read:
        print(f"\nExtracting POINT CLOUD from bag files...")
        pcd_data,pcd_timesamp = extract_point_cloud_from_topic(bags,topic_to_read['point-cloud'],verbose=False)        
        print("Extracted %d data points"%len(pcd_data))
        data_extracted['point-cloud'] = True

    if 'poses' in topic_to_read:
        # Extract poses from topic
        print("\nExtracting POSES from topic...\n")
        poses,poses_timestamp = extract_poses_from_topic(bags,topic_to_read['poses'])
        print("Extracted %d data points"%len(poses))
        data_extracted['poses'] = True
    
    elif tfs != None and 'poses' in tfs:
        print(f"\nExtracting POSES from bag files...")
        poses,poses_timestamp = extract_from_tf(bags,tfs['poses'],verbose=False)
        print("Extracted %d data points"%len(poses))
        data_extracted['poses'] = True

    if 'gps' in topic_to_read:
        print("\nExtracting GPS from topic...\n")
        gps_data,gps_timestamp = extract_gps_from_topic(bags,topic_to_read['gps'])
        print("Extracted %d data points"%len(gps_data))
        data_extracted['gps'] = True
    
    if 'imu' in topic_to_read:
        print("\nExtracting IMU from topic...\n")
        imu_data,imu_timestamp = extract_IMU_from_topic(bags,topic_to_read['imu'])
        print("Extracted %d data points"%len(imu_data))
        data_extracted['imu'] = True

    # =====================
    # Data Association
    # Save sync data to files using the KITTI format
    # =====================
    save_pcd_KITTI_format(pcd_data,pcd_timesamp,target_dir)

    if data_extracted['poses']:
        print("\Sync POSES data with point cloud...")
        nearest_indices = timestamp_match(pcd_timesamp,poses_timestamp)

        poses = poses[nearest_indices]
        poses_timestamp = poses_timestamp[nearest_indices]

        save_poses_KITTI_format(poses,poses_timestamp,target_dir)

    if 'gps' in list(data_extracted.keys()) and data_extracted['gps']:
        print("\Sync GPS data with point cloud...")
        nearest_indices = timestamp_match(pcd_timesamp,gps_timestamp)

        gps_data      = gps_data[nearest_indices]
        gps_timestamp = gps_timestamp[nearest_indices]

        save_gps_KITTI_format(gps_data,gps_timestamp,target_dir)

    if 'imu' in list(data_extracted.keys()) and data_extracted['imu']:
        print("\Sync IMU data with point cloud...")
        nearest_indices = timestamp_match(pcd_timesamp,imu_timestamp)

        imu_data      = imu_data[nearest_indices]
        imu_timestamp = imu_timestamp[nearest_indices]

        save_imu_KITTI_format(imu_data,imu_timestamp,target_dir)



    info_fd = open(os.path.join(target_dir,'extracted_info.txt'),'w')
    info_fd.write("GPS info structure -> utm data:  lat lon alt\n")
    info_fd.write("static tf structure -> frame_id child_frame_id x y z yaw pitch roll (radians)\n")
    info_fd.write("poses -> transformation matrix 4x4 a11,a12,a13,a21,a22,a23,a31,a32,a33,0,0,0,1\n")
    info_fd.close()
   

