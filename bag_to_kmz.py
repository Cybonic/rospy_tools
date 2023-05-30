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

def save_gps_to_kmz(bag_files,topic_dict,dst_root,dst_dir,skip=30):
    
    dst_dir_root  = os.path.join(dst_root,dst_dir)
    raw_gps_file = os.path.join(dst_dir_root,'raw_gps.kmz')

    topic_list = list(topic_dict.values())
    raw_gps_counter  = 1

    
    kml = Kml(name=dst_dir)
    bag = rosbag.Bag(bag_files)
    for i,(topic, msg, t) in tqdm(enumerate(bag.read_messages(topics=topic_list)),'Reading from Topics'):
        if i == 0:
            t0 = t
        delta = (t-t0)
        t0=t
        if topic == topic_dict['gps']:
            raw_gps_counter +=1
            result = raw_gps_counter% skip 
            #result = 0
            if  result == 0:
                lat = msg.latitude
                lon = msg.longitude
                alt = msg.altitude
                kml.newpoint(name="", coords=[(lon,lat,alt)])  # A simple Point

    kml.savekmz(raw_gps_file, format=False)  # Saving as KMZ
    print("="*50)
    print(f' gps: {raw_gps_counter}')
    print("="*50)

def parse_bag_name(file):
    return file.split('/')[-1].split('.')[0]

def main_bag_to_file(bag_file,topic_to_read,dst_root):
    print(bag_file)
    # topic_to_read = list(cfg['topics'].values())
    file = parse_bag_name(bag_file)
   
    save_gps_to_kmz(bag_file,topic_to_read,dst_root,file)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = "Convert bag dataset to files!")
    parser.add_argument("--bag",default='/media/tiago/vbig/dataset/FU_Odometry_Dataset/rawbags/128/20190924-144057.bag')
    parser.add_argument("--target_bag_dir",default='/media/tiago/vbig/dataset/FU_Odometry_Dataset/rawbags/64/')
    parser.add_argument("--pcl_topic",default='/sensors/velodyne_points')
    parser.add_argument("--pose",default='/sensors/applanix/gps_odom')
    parser.add_argument("--dst_root",default='/media/tiago/vbig/dataset/FU_Odometry_Dataset/64LiDAR')
    parser.add_argument("--gps",default='/sensors/applanix/gps_fix')
    args = parser.parse_args()
    # /sensors/applanix/gps_odom
    
    topic_to_read = {'point-cloud':args.pcl_topic,'pose':args.pose,'gps':args.gps}
    for file in os.listdir(args.target_bag_dir):
        full_path_file = os.path.join(args.target_bag_dir,file)
        if not file.endswith('.bag'):
            continue 
        #bag_file = args.bag
        main_bag_to_file(full_path_file,topic_to_read,args.dst_root)

    
