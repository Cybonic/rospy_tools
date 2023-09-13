#!/usr/bin/env python3

import yaml
from shutil import copyfile
import os
from tqdm import tqdm
import argparse
import numpy as np
import numpy as np
from torchvision import transforms as T
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
import imageio
from PIL import ImageOps



if __name__ == '__main__':

  parser = argparse.ArgumentParser("./infer.py")
  parser.add_argument(
    '--root',
    type=str,
    required=False,
    default="/media/tiago/vbig/dataset/GEOGIATECH-FR/onfoot/extracted",
    #default='/media/tiago/BIG',
    #default='predictions',
    help='Directory to get the trained model.'
  )
  parser.add_argument(
    '--dataset',
    type=str,
    required=False,
    default='orchard-uk',
    #default='kitti',
    help='Directory to get the trained model.'
  )

  parser.add_argument(
    '--sequence',
    type=str,
    required=False,
    #default='02',
    default='200803',
    help='Directory to get the trained model.'
  )

  parser.add_argument(
    '--max_points',
    type=str,
    required=False,
    default=20000,
    help='Directory to get the trained model.'
  )

  FLAGS, unparsed = parser.parse_known_args()

 
  print("----------")
  print("INTERFACE:")
  print("Root: ", FLAGS.root)
  print("Dataset  : ", FLAGS.dataset)
  print("Sequence : ",FLAGS.sequence)
  print("Max points : ",FLAGS.max_points)
  print("----------\n")

  dataset    = FLAGS.dataset
  sequence   = FLAGS.sequence
  max_points = FLAGS.max_points
  
  from dataloader.projections import SphericalProjection,BEVProjection
  from dataloader.defaultloader import loader,velo_parser
  
  bevprj = BEVProjection(512,512,parser=velo_parser())
  spherical_proj = SphericalProjection(512,512,parser=velo_parser())
  
  dataset = loader(FLAGS.root,FLAGS.sequence,bevprj)
  
  save_dir = os.path.join('temp',f'{sequence}')
  if not os.path.isdir(save_dir):
    os.makedirs(save_dir)
    
  
  #loader = OrchardDataset(root,'',sequence, modality = 'bev')

  fig = Figure(figsize=(5, 4), dpi=100,)
  fig, ax = plt.subplots()

  filename = 'projection.gif'
  canvas = FigureCanvasAgg(fig)
  writer = imageio.get_writer(filename, mode='I')

  fig, ax = plt.subplots(1, 1)
  num_samples = len(dataset)

  for i in tqdm(range(500,num_samples,10)):
    
    input = dataset[i] # Get only Bev projection
    if 'range' in input:
      input = input['range']
    if 'height' in input:
      input = input['height']
      
    
    input_im = input.astype(np.uint8)

    input_im  = T.ToTensor()(input_im).unsqueeze(0)
    pil_range = T.ToPILImage()(input_im[0])
    
   
    pil_range = ImageOps.colorize(pil_range, black="black", white="white")
    pil_range = ImageOps.autocontrast(pil_range,ignore=0)
    #pil_range.show()
    pil_range.save(os.path.join(save_dir,f'{i}.png'))
    #plt.show()
    #X = np.asarray(pil_range)
    #writer.append_data(X)






  
  