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

def extract_bag_data(bag_files,topics,root):
    
    velodyne_dir =  os.path.join(root,'point_cloud')

    if not os.path.isdir(velodyne_dir):
        os.makedirs(velodyne_dir)

    pose_file = os.path.join(root,'pose.txt')
    raw_gps_file = os.path.join(root,'raw_gps.txt')
 
    pose_times_file = os.path.join(root,'pose_times.txt')
    raw_gps_times_file = os.path.join(root,'raw_gps_times.txt')
    velo_times_file = os.path.join(root,'point_cloud_times.txt')
    #velo_file = os.path.join(pose_dir,'velo.txt')

    ft  = open(pose_times_file,'w')
    ftrgps  = open(raw_gps_times_file,'w')
    fvt = open(velo_times_file,'w')
    fp  = open(pose_file,'w')
    frgps  = open(raw_gps_file,'w')

    topic_list = list(topics.values())
    

    first_flag       = 0
    pose_counter     = 0
    velodyne_counter = 0 
    raw_gps_counter  = 0
    gps_pose0        = 0 
    
    #for bag_file in tqdm(bag_files):
    #with tqdm(total=100) as pbar:

    bag = rosbag.Bag(bag_files)
    for i,(topic, msg, t) in tqdm(enumerate(bag.read_messages(topics=topic_list)),'Reading from Topics'):
        if i == 0:
            t0 = t
        
        delta = (t-t0)
        t0=t


        if 'pose' in topics and topic == topics['pose']:
            ft.write(str(t) + '\n')
            pose = GeometryMsg2str(msg.pose)
            fp.write(pose + '\n')
            pose_counter +=1
        
        elif 'raw_gps' in topics and  topic == topics['raw_gps']:
            ftrgps.write(str(t) + '\n')
            gps_pose = NavSatFix2local(msg) # 
            if raw_gps_counter == 0:
                gps_pose0 = gps_pose.copy()
            gps_pose -= gps_pose0
            #print(gps_pose)
            gps_pose_str = '{} {}'.format(gps_pose[0],gps_pose[1])
            frgps.write(gps_pose_str + '\n')
            raw_gps_counter+=1
        elif topic == topics['point-cloud']:
            
            if first_flag == 0:
                first_flag = 1
                i0 = i

            name = os.path.join(velodyne_dir,'{0:07d}.bin'.format(velodyne_counter))
            #msg.
            pcl = open(name,'wb')
            pc_np = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
            pc_np.tofile(pcl)
            pcl.close()

            fvt.write(str(t) + '\n')

            #print('{:30s}: {:10s} delta: {:10s}'.format(topic,str(t),str(delta)))
            velodyne_counter+=1

    bag.close()
    ft.close()
    fp.close()
    fvt.close()
    print("="*50)
    print(f'pose: {pose_counter}  velodyne: {velodyne_counter} gps: {raw_gps_counter}')
    print("="*50)




if __name__ == '__main__':
    
    
    parser = argparse.ArgumentParser(description = "Convert bag dataset to files!")
    parser.add_argument("--cfg", default = 'topics2read.yaml')
    parser.add_argument("--bag",default='~/Dropbox/research/datasets/orchard-uk/winter/winterOrchard.bag')
    args = parser.parse_args()


    cfg = args.cfg
    assert os.path.isfile(cfg)

    cfg = yaml.load(open(cfg))
    bag_file = args.bag
    print(bag_file)
    
    topic_to_read = list(cfg['topics'].values())

    extract_bag_data(bag_file,cfg['topics'],'')
