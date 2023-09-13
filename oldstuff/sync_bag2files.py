import os
from sre_parse import FLAGS
import cv2
import rospy
import progressbar

from datetime import datetime
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatFix
import sensor_msgs.point_cloud2 as pcl2
#from sensor_msgs import PointCloud
import sensor_msgs.point_cloud2
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform
from cv_bridge import CvBridge
import numpy as np
import argparse
import rosbag
from std_msgs.msg import Int32, String
import yaml
from tqdm import tqdm
import ros_numpy
from simplekml import Kml # https://simplekml.readthedocs.io/en/latest/kml.html
import copy
import math


def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


def NavMsgsOdometry(msgs):
    return GeometryMsg2str(msgs.pose)

def NavSatFix2local(nav):
    import utm
    lat = nav.latitude
    lon = nav.longitude
    alt = nav.altitude

    u = utm.from_latlon(lat, lon)
    return(np.array([u[0],u[1]]))

def GeometryMsg2str(masg):
    Pose = masg.pose
    theta = orientation2str(Pose.orientation)
    position = position2str(Pose.position)
    return(' '.join([position,theta]))

def orientation2list(msg):
    return(list(msg.x,msg.y,msg.z,msg.w))

def orientation2str(msg):
    return ' '.join([str(msg.x),str(msg.y),str(msg.z),str(msg.w)])   

def position2str(msg):
    return ' '.join([str(msg.x),str(msg.y),str(msg.z)])  

def extract_bag_data(bag_files,topic_dict,dst_root,dst_dir):
    
    dst_dir_root  = os.path.join(dst_root,dst_dir)
    velodyne_dir =  os.path.join(dst_dir_root,'point_cloud')
    
    if not os.path.isdir(velodyne_dir):
        os.makedirs(velodyne_dir)

    pose_file = os.path.join(dst_dir_root,'pose.txt')
    raw_gps_file = os.path.join(dst_dir_root,'raw_gps.kmz')
 
    pose_times_file = os.path.join(dst_dir_root,'pose_times.txt')
    raw_gps_times_file = os.path.join(dst_dir_root,'raw_gps_times.txt')
    velo_times_file = os.path.join(dst_dir_root,'point_cloud_times.txt')
    #velo_file = os.path.join(pose_dir,'velo.txt')

    ft  = open(pose_times_file,'w')
    ftrgps  = open(raw_gps_times_file,'w')
    fvt = open(velo_times_file,'w')
    fp  = open(pose_file,'w')
    #frgps  = open(raw_gps_file,'w')

    topic_list = list(topic_dict.values())
    

    pose_counter     = 0
    velodyne_counter = 0 


    
    #for bag_file in tqdm(bag_files):
    #with tqdm(total=100) as pbar:
    pcl_sync_sample = {'sync':0}
    pose_sync_sample = {'sync':0}
    delta_msecs_buffer = []
    kml = Kml(name=dst_dir)
    bag = rosbag.Bag(bag_files)

    for i,(topic, msg, t) in tqdm(enumerate(bag.read_messages(topics=topic_list)),'Reading from Topics'):
        if i == 0:
            t0 = copy.copy(t)
        
        if topic == topic_dict['pose']:
            
            if 'Odometry' in msg._type:
                pose = NavMsgsOdometry(msg)
            
            pose_sync_sample['pose'] = pose
            pose_sync_sample['t'] = t
            pose_sync_sample['sync'] = 1

        elif topic == topic_dict['point-cloud']:            
            pc_np = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
            pcl_sync_sample['pcl'] = pc_np
            pcl_sync_sample['t'] = t
            pcl_sync_sample['sync'] = 1

        if pcl_sync_sample['sync'] and pose_sync_sample['sync']:
            # Save sync pcl and pose
            name = os.path.join(velodyne_dir,'{0:07d}.bin'.format(velodyne_counter))
            pcl = open(name,'wb')
            pcl_sync_sample['pcl'].tofile(pcl)
            pcl.close()
            # Save PCL timestamp
            fvt.write(str(pcl_sync_sample['t']) + '\n')
            velodyne_counter+=1
            # Save Pose
            fp.write(pose_sync_sample['pose'] + '\n')
            # Save Pose timestamp
            ft.write(str(pose_sync_sample['t']) + '\n')
            pose_counter +=1
            
            pose_sync_sample['sync'] = 0
            pcl_sync_sample['sync'] = 0
            #print("\n")
            delta = pcl_sync_sample['t']-pose_sync_sample['t']
            delta = float(delta.secs + delta.nsecs)/1000000
            delta_msecs_buffer.append(delta)
            #print(delta)

    bag.close()
    ft.close()
    fp.close()
    fvt.close()
    
    print("="*50)
    delta_msecs_buffer = np.array(delta_msecs_buffer)

    print("time_pcl - time_pose [ms] %f %f"%(delta_msecs_buffer.mean(),delta_msecs_buffer.std()))
    print(f'pose: {pose_counter}  velodyne: {velodyne_counter}')
    print("="*50)

def parse_bag_name(file):
    return file.split('/')[-1].split('.')[0]

def main_bag_to_file(bag_file,topic_to_read,dst_root):
    print(bag_file)
    # topic_to_read = list(cfg['topics'].values())
    file = parse_bag_name(bag_file)
   
    extract_bag_data(bag_file,topic_to_read,dst_root,file)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = "Convert bag dataset to files!")
    parser.add_argument("--bag",default='/media/tiago/vbig/dataset/FU_Odometry_Dataset/rawbags/64/20170303-140708.bag')
    parser.add_argument("--target_bag_dir",default='/media/tiago/vbig/dataset/FU_Odometry_Dataset/rawbags/128/')
    parser.add_argument("--pcl_topic",default='/sensors/velodyne_points')
    parser.add_argument("--pose",default='/sensors/applanix/gps_odom')
    parser.add_argument("--dst_root",default='/media/tiago/vbig/dataset/FU_Odometry_Dataset/128LiDAR')
    args = parser.parse_args()
    # /sensors/applanix/gps_odom
    
    topic_to_read = {'point-cloud':args.pcl_topic,'pose':args.pose}
    
    for file in os.listdir(args.target_bag_dir):
        full_path_file = os.path.join(args.target_bag_dir,file)
        if not file.endswith('.bag'):
            continue 
        #bag_file = args.bag
        main_bag_to_file(full_path_file,topic_to_read,args.dst_root)

    
