import numpy as np
import open3d as o3d
import os,tqdm
OUSTER_16_STRUCTURE =  np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgba', '<u4'), ('intensity', '<f4')])
VELODYNE_64_STRUCTURE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4')])


def extract_roi(point_cloud, min_bound, max_bound):
    """
    Extracts a region of interest (ROI) from a point cloud.

    Args:
        point_cloud (np.array): Nx3 array of points.
        min_bound (np.array): 1x3 array of minimum x, y, z coordinates of the ROI.
        max_bound (np.array): 1x3 array of maximum x, y, z coordinates of the ROI.

    Returns:
        np.array: Nx3 array of points within the ROI.
    """
    # Use logical indexing to find points within the bounds
    within_bounds = np.all((point_cloud >= min_bound) & (point_cloud <= max_bound), axis=1)
    # Extract the points within the bounds
    roi_points = point_cloud[within_bounds]
    return roi_points

def extract_roi_from_pcds(pcds,min_bound,max_bound):
    """_summary_

    Args:
        pcds (numpy): [x,y,z]
        min_bound (list): [x,y,z]
        max_bound (list): [x,y,z]

    Returns:
        numpy: _description_
    """
    roi_point_clouds= []
    for pcd in tqdm.tqdm(pcds,"Extracting ROI"):
        roi_point_clouds.append(extract_roi(pcd, min_bound, max_bound))
    return roi_point_clouds

def downsample_pcd(pcd,n_points):
    points = np.asarray(pcd.points)
    sampled_points = points[np.random.choice(points.shape[0], 1000, replace=False)]
    pcd.points = o3d.utility.Vector3dVector(sampled_points)
    return pcd


def np_pts_to_pcd(np_pts):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_pts)
    return pcd          

class  parser():
    def __init__(self,dt):
        self.dt = []
        self.dt = dt
        self.n_fields = len(self.dt)

    def velo_sensor(self,scan):
        return(np.stack((scan['x'].flatten(),scan['y'].flatten(),scan['z'].flatten(),scan['intensity'].flatten()),axis=1))
    
    def velo_read(self,scan_path):
        
        scan = np.fromfile(scan_path, dtype=self.dt)
        
        return(np.stack((scan['x'],scan['y'],scan['z'],scan['intensity']),axis=1))




class load_from_files():
    def __init__(self,root):
        self.root = root
  
        dt =None
        if 'kitti' in root:
            dt = VELODYNE_64_STRUCTURE
            self.read_poses = self.read_poses_kitti
            self.lidar_dir  = os.path.join(root,"velodyne")
            self.poses_file = os.path.join(root,'poses.txt')
        elif 'on-foot' in root:
            dt = OUSTER_16_STRUCTURE
            self.read_poses = self.read_poses_onfoot
            self.lidar_dir  = os.path.join(root,"point_cloud")
            self.poses_file = os.path.join(root,'odometry.txt')
        else:
            assert dt != None
        self.lidar_parser = parser(dt)
        
        #assert os.path.isfile(self.poses_file), "Pose file is not recognized"
        
    def load_pcds(self,):

        files = np.sort(os.listdir(self.lidar_dir))
        full_pcd_path = [os.path.join(self.lidar_dir,file) for file in files]
        pcds = []
        for file in tqdm.tqdm(full_pcd_path,"Loading PCDs"):
            pcds.append(self.lidar_parser.velo_read(file)[:,:3])
            
        return pcds

    
    def read_poses_onfoot(self):
        assert os.path.isfile(self.poses_file)
        
        pose_array = []
        for line in tqdm.tqdm(open(self.poses_file,'r'), 'Loading to RAM'):
            line = line.replace('\n','')
            values_str = line.split(' ')
            values = np.array([float(v) for v in values_str])
            values = values.reshape((4,4))
            pose_array.append(values)
        pose_array = np.array(pose_array)
        return(pose_array)
    
    
    def read_poses_kitti(self):
        file = self.poses_file
        T_array = []
        t = np.zeros((4,4))
        t[0,2]=1
        t[1,0]=-1
        t[2,1]=-1
        t[3,3]=1
        for line in open(file):
            l = line.strip().split(' ')
            l_num = np.asarray([float(value) for value in l])
            T = np.eye(4)
            T[:3,:] = l_num.reshape((3,4))
            T = np.matmul(t,T)
            T_array.append(T)
        
        return np.stack(T_array,axis=0)