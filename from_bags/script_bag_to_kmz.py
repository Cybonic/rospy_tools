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

def parseGeometryMsg(msgs):
    Pose = msgs.pose.pose
    theta = orientation2list(Pose.orientation)
    position = position2list(Pose.position)

    return{'p':position,'o':theta}

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
    return(list((msg.x,msg.y,msg.z,msg.w)))

def position2list(msg):
    return [msg.x,msg.y,msg.z]  

def orientation2str(msg):
    return ' '.join([str(msg.x),str(msg.y),str(msg.z),str(msg.w)])   

def position2str(msg):
    return ' '.join([str(msg.x),str(msg.y),str(msg.z)])  

def save_gps_to_kmz(bag_files,topic_dict,dst_root,dst_dir,skip=1):
    
    dst_dir_root  = os.path.join(dst_root,dst_dir)
    if not os.path.isdir(dst_dir_root):
        os.makedirs(dst_dir_root)

    raw_gps_file = os.path.join(dst_dir_root,'raw_gps.kmz')

    topic_list = list(topic_dict.values())
    raw_gps_counter  = 1
    counter = 1
    
    kml = Kml(name=dst_dir)
    bag = rosbag.Bag(bag_files)
    for i,(topic, msg, t) in tqdm(enumerate(bag.read_messages(topics=topic_list)),'Reading from Topics'):
        if i == 0:
            t0 = t
        delta = (t-t0)
        t0=t


        counter +=1

        if counter%skip !=0:
            continue

        if 'pose' in topic_dict and topic == topic_dict['pose']:
            
            if 'Odometry' in msg._type:
                data = parseGeometryMsg(msg)
                pose = data['p']
                kml.newpoint(name="", coords=[(pose[0],pose[1],pose[2])])  # A simple Point

        if 'gps' in topic_dict and topic == topic_dict['gps']:

            lat = msg.latitude
            lon = msg.longitude
            alt = msg.altitude
            kml.newpoint(name="", coords=[(lon,lat,alt)])  # A simple Point

    kml.savekmz(raw_gps_file, format=False)  # Saving as KMZ
    print("="*50)
    #print(f' gps: {counter}')
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
    parser.add_argument("--bag",default='/home/tiago/Dropbox/SHARE/DATASET/GEORGIA-FR/onfoot/200803/semantics_2020-08-03-14-43-08_3.bag')
    parser.add_argument("--target_bag_dir",default='/home/tiago/Dropbox/SHARE/DATASET/GEORGIA-FR/onfoot/200803/')
    parser.add_argument("--pcl_topic",default='/sensors/velodyne_points')
    parser.add_argument("--pose",default='/sensors/applanix/gps_odom')
    parser.add_argument("--dst_root",default='/home/tiago/Dropbox/SHARE/DATASET/GEORGIA-FR/onfoot/200803')
    parser.add_argument("--gps",default='/vectornav/GPS_INS')
    args = parser.parse_args()
    # /sensors/applanix/gps_odom
    print("RUNNING\n")

    topic_to_read = {'pose':args.pose}
    topic_to_read = {'gps':args.gps}

    full_path_file = []
    if args.target_bag_dir != None:
        print("DIR")
        for file in os.listdir(args.target_bag_dir):
            path_file = os.path.join(args.target_bag_dir,file)
            if not file.endswith('.bag'):
                continue
            full_path_file.append(path_file) 
    else:
        print("INFO")
        print(args.bag)
        full_path_file = [args.bag]

    for file in full_path_file:
        print("FILES")
        #print(file)
        main_bag_to_file(file,topic_to_read,args.dst_root)

    
