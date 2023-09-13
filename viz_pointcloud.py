#!/usr/bin/env python3

import yaml
from shutil import copyfile
import os
from tqdm import tqdm
import sys
import argparse
import numpy as np




if __name__ == '__main__':

  parser = argparse.ArgumentParser("./infer.py")
  parser.add_argument(
    '--root',
    type=str,
    required=False,
    # default='checkpoints/range-rerecord_sparce-AttVLAD_resnet50-0.87.pth',
    #default='/media/tiago/BIG',
    default="/media/tiago/vbig/dataset/GEOGIATECH-FR/onfoot/extracted",
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
    default=30000,
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

  root = FLAGS.root
  dataset    = FLAGS.dataset
  sequence   = FLAGS.sequence
  max_points = FLAGS.max_points
  
  from dataloader.laserscan import Scan
  from dataloader.defaultloader import loader,velo_parser
  
  scan = Scan(parser=velo_parser())
  
  dataset = loader(FLAGS.root,FLAGS.sequence,scan)
  
  from dataloader.laserscanvis import LaserScanVis


  #points = loader(0)
  #pose = loader.get_pose(0).reshape(1,-1)
  #color = colorize_pcl(points)

  

  laservis = LaserScanVis(dataset = dataset,size=10) 
  laservis.run()

  #print(10)  

  
  