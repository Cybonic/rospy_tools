# /bin/python3 

import math
import numpy as np
#import utm
from ros_numpy.geometry import vector3_to_numpy, quat_to_numpy
from tf import transformations

def vector_to_matrix(translation,rotation):
    '''
    translation: [x,y,z]
    rotation: [x,y,z,w]
    '''
    return np.dot(
        transformations.translation_matrix(translation),
        transformations.quaternion_matrix(rotation)
    )
        
def transform_to_numpy(msg):
	return np.dot(
        transformations.translation_matrix(vector3_to_numpy(msg.translation)),
        transformations.quaternion_matrix(quat_to_numpy(msg.rotation))
    )
 
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

def NavSatFixLocal(nav):
    lat = nav.latitude # degrees
    lon = nav.longitude # degrees
    alt = nav.altitude # Positive is above the WGS 84 ellipsoid
    u = utm.from_latlon(lat, lon)
    return(np.array([u[0],u[1],alt]))

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

def transform_np_to_str(array: np.ndarray,precision = 3)-> None:
    '''
    Convert a numpy array to a string
    array: numpy array
    '''
    array_str_list = array.flatten().tolist()

    # Convert value to string   
    array_str_list = [str(round(value,precision)) for value in array_str_list]
    return ' '.join(array_str_list)
