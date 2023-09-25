#!/usr/bin/env python3

import yaml
from shutil import copyfile
import os
from tqdm import tqdm
import sys
import argparse


if __name__ == '__main__':

  parser = argparse.ArgumentParser("./infer.py")
  parser.add_argument(
    '--root',
    type=str,
    required=False,
    default='/media/tiago/vbig/dataset/LBORO-UK/orchard-uk/june23/extracted',
    help='Directory to get the trained model.'
  )
  parser.add_argument(
    '--max_points',
    type=str,
    required=False,
    default=30000,
    help='Directory to get the trained model.'
  )
  parser.add_argument(
    '--kitti_format',
    type=bool,
    default=True,
    required=False,
    help='Enable point cloud format.'
  )

  FLAGS, unparsed = parser.parse_known_args()

  print("----------")
  print("INTERFACE:")
  print("Root: ", FLAGS.root)
  print("Max points : ",FLAGS.max_points)
  print("----------\n")

  root       = FLAGS.root
  max_points = FLAGS.max_points
  
  from dataloader.laserscan import Scan
  from dataloader.defaultloader import loader,velo_parser
  
  scan = Scan()
  if not FLAGS.kitti_format:
    scan = Scan(parser=velo_parser())
  
  dataset = loader(FLAGS.root,"",scan)
  
  from dataloader.laserscanvis import LaserScanVis

  laservis = LaserScanVis(dataset = dataset,size=10) 
  laservis.run()
  
  