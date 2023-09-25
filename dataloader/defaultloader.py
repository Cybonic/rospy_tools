
import os,sys
import numpy as np
sys.path.append(os.sep.join(os.path.dirname(__file__).split(os.sep)[:-1]))
from dataloader.projections import SphericalProjection
from tqdm import tqdm

def get_files(target_dir):
    assert os.path.isdir(target_dir), "Target directory does not exit! " + target_dir
    files = np.array([f.split('.')[0] for f in os.listdir(target_dir)])
    idx = np.argsort(files)
    fullfiles = np.array([os.path.join(target_dir,f) for f in os.listdir(target_dir)])
    return(files[idx],fullfiles[idx])

def load_pose_to_RAM(file):
    assert os.path.isfile(file)
    pose_array = []
    for line in tqdm(open(file,'r'), 'Loading to RAM'):
        line = line.replace('\n','')
        values_str = line.split(' ')
        values = np.array([float(v) for v in values_str])
        if len(values)>3:
            position = values[[3,7,11]]
        else:
            position = values
        pose_array.append(position.tolist())
    pose_array = np.array(pose_array)
    return(pose_array)


class  velo_parser():
    def __init__(self):
        self.dt = []
        self.dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgba', '<u4'), ('intensity', '<f4')])
        self.n_fields = len(self.dt)

    def velo_read(self,scan_path):
        
        #scan = np.fromfile(scan_path, dtype=self.dt)
        scan = np.fromfile(scan_path)
        #return scan.reshape(-1,4)
        return(np.stack((scan['x'],scan['y'],scan['z'],scan['intensity']),axis=1))
    
class loader:
    def __init__(self,root,sequence,modality):
        self.root = root
        self.sequence = sequence
        self.modality = modality
        self.target_dir = os.path.join(root,sequence)
        _,self.pclfiles= get_files(os.path.join(self.target_dir,'point_cloud'))
        #self.poses = load_pose_to_RAM(os.path.join(self.target_dir,'odom.txt'))
    
    def __len__(self):
        return len(self.pclfiles)
    
    def __getitem__(self,idx):
        file = self.pclfiles[idx]
        data = self.modality.load(file)
        return data
        
        


if __name__=="__main__":
    root = "/media/tiago/vbig/dataset/GEOGIATECH-FR/onfoot/extracted"
    sequence = '200803'
    proj = SphericalProjection(512,512,parser=velo_parser())
    dataset = loader(root,sequence,proj)
    
    for img in dataset:
        print(img)
        