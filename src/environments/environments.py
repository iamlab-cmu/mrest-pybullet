#!/usr/bin/env python

import pybullet as p
import pybullet_data

import os
import numpy as np

# Relative path to urdf/models folder
dirname = os.path.dirname(__file__)
foldername= os.path.join(dirname, '../urdf/')
URDF_FOLDER_NAME = '/home/ballbot/Workspace/pybullet_ws/src/ballbot_pybullet_sim/urdf/'

class TableEnv(object):
    def __init__(self, startPos = [0.,0.,0.], startOrientationEuler = [0.,0.,0.]):

        startOrientation = p.getQuaternionFromEuler(startOrientationEuler)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.table = p.loadURDF("table/table.urdf", startPos, startOrientation)
        boxStartPos = np.array(startPos) + np.array([0.,0.,0.775])
        self.box = p.loadURDF(URDF_FOLDER_NAME + "cardboard_box/box.urdf",boxStartPos, startOrientation)

        

class KivaShelf(object):
    def __init__(self, startPos = [0.,0.,0.], startOrientationEuler = [0.,0.,0.]):

        startOrientation = p.getQuaternionFromEuler(startOrientationEuler)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.table = p.loadSDF("kiva_shelf/model.sdf", startPos, startOrientation)

