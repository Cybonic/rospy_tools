#!/usr/bin/env python3
# This file is covered by the LICENSE file in the root of this project.

import vispy
from vispy.scene import visuals, SceneCanvas
import numpy as np
from matplotlib import pyplot as plt
import os 
import yaml
#import config as cnf

class LaserScanVis:
  """Class that creates and handles a visualizer for a pointcloud"""

  def __init__(self,dataset,size= 10 ):
    
    #self.scan = scan
    self.size = size
    self.pointcloud = dataset
    self.offset = 0
    self.total = len(dataset)

    self.reset()
    self.update_scan()
    # make instance colors

  def reset(self):
    """ Reset. """
    # last key press (it should have a mutex, but visualization is not
    # safety critical, so let's do things wrong)
    self.action = "next"  # no, next, back, quit are the possibilities
    # new canvas prepared for visualizing data
    self.canvas = SceneCanvas(keys='interactive', show=True)
    self.canvas.show()
    # interface (n next, b back, q quit, very simple)
    self.canvas.events.key_press.connect(self.key_press)
    self.canvas.events.draw.connect(self.draw)
    # grid
    self.grid = self.canvas.central_widget.add_grid()

    # laserscan part
    self.scan_view = vispy.scene.widgets.ViewBox(
        border_color='white', parent=self.canvas.scene)

    #self.img_view = vispy.scene.widgets.ViewBox(border_color='blue', parent=self.canvas.scene)
    #self.pose_view = vispy.scene.widgets.ViewBox(
    #    border_color='blue', parent=self.canvas.scene)

    self.grid.add_widget(self.scan_view, 0, 0)
    #self.grid.add_widget(self.img_view, 0, 1)

    self.scan_vis = visuals.Markers()
    #self.image_vis = visuals.Image()
    #self.poses_vis = visuals.Markers()
    
    self.scan_view.camera =  'turntable'
    #self.img_view.camera = 'turntable'
    
    self.scan_view.add(self.scan_vis)
    #self.img_view.add(self.image_vis)
    #self.pose_view.add(self.poses_vis)

    visuals.XYZAxis(parent=self.scan_view.scene)
    #visuals.XYZAxis(parent=self.pose_view.scene)

 
  def update_scan(self):
    # first open data
    # then change names
    title = "scan "
    self.canvas.title = title
    points,_ = self.pointcloud[self.offset]
    
    #img = self.pointcloud.get_img(self.offset)
    #poses = self.pointcloud.get_poses(self.offset)

    #image1 = scene.visuals.Image(im1, parent=vb1.scene)
    #self.image_vis.set_data(img)
    self.scan_vis.set_data(points,size = self.size)
    #self.poses_vis.set_data(poses,size = 10)

  def draw(self, event):
    if self.canvas.events.key_press.blocked():
      self.canvas.events.key_press.unblock()
  
  # interface
  def key_press(self, event):
    self.canvas.events.key_press.block()
    if event.key == 'N':
      self.offset += 1
      if self.offset >= self.total:
        self.offset = 0
      self.update_scan()
    elif event.key == 'B':
      self.offset -= 1
      if self.offset < 0:
        self.offset = self.total - 1
      self.update_scan()
    elif event.key == 'Q' or event.key == 'Escape':
      self.destroy()


  def destroy(self):
    # destroy the visualization
    self.canvas.close()
    vispy.app.quit()

  def run(self):
    vispy.app.run()

