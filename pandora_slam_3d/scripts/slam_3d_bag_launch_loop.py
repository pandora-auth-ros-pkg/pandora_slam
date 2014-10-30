#! /usr/bin/env python

from subprocess import call, Popen
import time

for ii in range(0, 2):
  Popen(["roslaunch", "pandora_slam_3d", "slam_3d_bag.launch"])
  time.sleep(180)
  call(["rosnode", "kill", "--all"])
  time.sleep(5)
  print ii
