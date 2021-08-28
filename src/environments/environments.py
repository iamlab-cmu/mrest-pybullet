#!/usr/bin/env python

import pybullet as p
import pybullet_data

import os
import numpy as np

# Relative path to urdf/models folder
# TODO: not sure if this is the cleanest way of doing it
dirname = os.path.dirname(__file__)
URDF_FOLDER_NAME = dirname.replace('src/environments','urdf/')

class Colors(object):
    def __init__(self):
        self.red = [0.97, 0.25, 0.25, 1]
        self.green = [0.41, 0.68, 0.31, 1]
        self.yellow = [0.92, 0.73, 0, 1]
        self.blue = [0, 0.55, 0.81, 1]
        self.colors = [self.red, self.green, self.yellow, self.blue]

class TableEnv(object):
    def __init__(self, startPos = [0.,0.,0.], startOrientationEuler = [0.,0.,0.]):

        startOrientation = p.getQuaternionFromEuler(startOrientationEuler)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.table = p.loadURDF("table/table.urdf", startPos, startOrientation)

        colors = Colors()
        boxStartPos = np.array(startPos) + np.array([-0.3,0.,0.775])
        self.box1 = p.loadURDF(URDF_FOLDER_NAME + "cardboard_box/box.urdf",boxStartPos, startOrientation)
        p.changeVisualShape(self.box1, -1, rgbaColor=colors.red)

        boxStartPos = np.array(boxStartPos) + np.array([0.,0.,0.3])
        self.box2 = p.loadURDF(URDF_FOLDER_NAME + "cardboard_box/box.urdf",boxStartPos, startOrientation)
        p.changeVisualShape(self.box2, -1, rgbaColor=colors.blue)




class KivaShelf(object):
    def __init__(self, startPos = [0.,0.,0.], startOrientationEuler = [0.,0.,0.]):

        startOrientation = p.getQuaternionFromEuler(startOrientationEuler)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.table = p.loadSDF("kiva_shelf/model.sdf", startPos, startOrientation)

