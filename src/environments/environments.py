#!/usr/bin/env python

import pybullet as p
import pybullet_data

import os
import numpy as np

# Relative path to urdf/models folder
# TODO: not sure if this is the cleanest way of doing it
dirname = os.path.dirname(__file__)
URDF_FOLDER_NAME = dirname.replace('src/environments', 'urdf/')


class Colors(object):
    def __init__(self):
        self.red = [0.97, 0.25, 0.25, 1]
        self.green = [0.41, 0.68, 0.31, 1]
        self.yellow = [0.92, 0.73, 0, 1]
        self.blue = [0, 0.55, 0.81, 1]
        self.colors = [self.red, self.green, self.yellow, self.blue]


class TableEnv(object):
    def __init__(self, startPos=[0., 0., 0.], startOrientationEuler=[0., 0., 0.]):

        startOrientation = p.getQuaternionFromEuler(startOrientationEuler)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # self.table = p.loadURDF("table/table.urdf", startPos, startOrientation)
        self.table = p.loadURDF(URDF_FOLDER_NAME + "table/table.urdf", startPos, startOrientation)

        colors = Colors()
        # boxStartPos = np.array(startPos) + np.array([-0.3, 0., 0.775])
        # self.box1 = p.loadURDF(
        #     URDF_FOLDER_NAME + "cardboard_box/box.urdf", boxStartPos, startOrientation)
        # p.changeVisualShape(self.box1, -1, rgbaColor=colors.red)

        # boxStartPos = np.array(boxStartPos) + np.array([0., 0., 0.3])
        # self.box2 = p.loadURDF(
        #     URDF_FOLDER_NAME + "cardboard_box/box.urdf", boxStartPos, startOrientation)
        # p.changeVisualShape(self.box2, -1, rgbaColor=colors.blue)

        # carStartPos = np.array(startPos) + np.array([-0.3, 0., 0.775])
        # carId = p.loadURDF('racecar/racecar.urdf', basePosition=carStartPos)

        mugStartPos = np.array(startPos) + np.array([-0.4, -0., 0.95])
        mugId = p.loadURDF('objects/mug.urdf', basePosition=mugStartPos)
        duckStartPos = np.array(startPos) + np.array([0.4, -0., 0.95])
        duckId = p.loadURDF('duck_vhacd.urdf', basePosition=duckStartPos)
        cubeStartPos = np.array(startPos) + np.array([-0.6, -0.4, 0.95])
        cubeId = p.loadURDF(URDF_FOLDER_NAME + "cardboard_box/cube.urdf", cubeStartPos, startOrientation)
        p.changeVisualShape(cubeId, -1, rgbaColor=colors.blue)

        cubeStartPos = np.array(startPos) + np.array([0.2, -0.4, 0.95])
        self.cubeId2 = p.loadURDF(URDF_FOLDER_NAME + "cardboard_box/cube.urdf", cubeStartPos, startOrientation)
        p.changeVisualShape(self.cubeId2, -1, rgbaColor=colors.red)
        p.changeDynamics(self.cubeId2, -1, lateralFriction=1.7)

class KivaShelf(object):
    def __init__(self, startPos=[0., 0., 0.], startOrientationEuler=[0., 0., 0.]):

        startOrientation = p.getQuaternionFromEuler(startOrientationEuler)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.table = p.loadSDF("kiva_shelf/model.sdf",
                               startPos, startOrientation)

class MSLEnv2(object):
    def __init__(self, startPos=[0., 0., 0.], startOrientationEuler=[0., 0., 0.]):

        startOrientation = p.getQuaternionFromEuler(startOrientationEuler)
        
        # Dimensions of green drawer
        self.I2M = 0.0254 # inche to meters
        self.DRAWER_X_IN = 27.5
        self.DRAWER_Y_IN = (22.0 + (5.0/8.0))
        self.DRAWER_Z_IN = 33.5
        self.DRAWER_X_M = self.DRAWER_Z_IN*self.I2M 
        self.DRAWER_Y_M = self.DRAWER_Z_IN*self.I2M 
        self.DRAWER_Z_M = self.DRAWER_Z_IN*self.I2M 

        # Center position of drawers inf ft and inches w.r.t to /map frame
        d_ft = np.array([[0.0,  0.0],
                     [0.0,  3.0],
                     [0.0,  5.0],
                     [0.0,  7.0],
                     [15.0, 7.0],
                     [15.0, 9.0],
                     [15.0,11.0],
                     [15.0,13.0],
                     [15.0,17.0],
                     [15.0,19.0],
                     [15.0,21.0],
                     [15.0,23.0],
                     [15.0,26.0]])

        d_in = np.array([[-1.0 - self.DRAWER_X_IN,-1.0],
                        [-1.0 - self.DRAWER_X_IN,-0.5],
                        [-1.0 - self.DRAWER_X_IN, 0.0],
                        [-1.0 - self.DRAWER_X_IN,-1.0],
                        [8.0,-4.0],
                        [8.0,-4.5],
                        [8.0,-3.0],
                        [8.0,-3.5],
                        [8.0,-1.0],
                        [8.0,-2.5],
                        [8.0,-3.5],
                        [8.0,-5.0],
                        [8.0, 4.7]])

        xyz = np.hstack( [np.array([self.I2M*((12*d_ft[:,0]) + d_in[:,0])]).T,
                      np.array([self.I2M*((12*d_ft[:,1]) + d_in[:,1])]).T,
                      np.zeros((np.shape(d_ft)[0],1)) ] )

        d_m = np.array([[-0.7239,-0.0254, self.DRAWER_X_M/2],
                        [-0.7239, 0.9017,self.DRAWER_X_M/2],
                        [-0.7239, 1.524, self.DRAWER_X_M/2],
                        [-0.7239,2.1082, self.DRAWER_X_M/2],
                        [4.7752,2.032,  self.DRAWER_X_M/2],
                        [4.7752,2.6289, self.DRAWER_X_M/2],
                        [4.7752,3.2766, self.DRAWER_X_M/2],
                        [4.7752, 3.8735, self.DRAWER_X_M/2],
                        [4.7752, 5.1562, self.DRAWER_X_M/2],
                        [4.7752, 5.7277, self.DRAWER_X_M/2],
                        [4.7752, 6.3119, self.DRAWER_X_M/2],
                        [4.7752, 6.8834, self.DRAWER_X_M/2],
                        [4.7752, 8.04418, self.DRAWER_X_M/2]])

class MSLEnv(object):
    def __init__(self, startPos=[0., 0., 0.], startOrientationEuler=[0., 0., 0.]):

        startOrientation = p.getQuaternionFromEuler(startOrientationEuler)

        # Dimensions of green drawer
        self.I2M = 0.0254 # inche to meters
        self.DRAWER_X_IN = 27.5
        self.DRAWER_Y_IN = (22.0 + (5.0/8.0))
        self.DRAWER_Z_IN = 33.5
        self.DRAWER_X_M = self.DRAWER_Z_IN*self.I2M 
        self.DRAWER_Y_M = self.DRAWER_Z_IN*self.I2M 
        self.DRAWER_Z_M = self.DRAWER_Z_IN*self.I2M 
        pos_m = np.array([[-0.7239,-0.0254, self.DRAWER_X_M/2],
                        [-0.7239, 0.9017,self.DRAWER_X_M/2],
                        [-0.7239, 1.524, self.DRAWER_X_M/2],
                        [-0.7239,2.1082, self.DRAWER_X_M/2],
                        [4.7752,2.032,  self.DRAWER_X_M/2],
                        [4.7752,2.6289, self.DRAWER_X_M/2],
                        [4.7752,3.2766, self.DRAWER_X_M/2],
                        [4.7752, 3.8735, self.DRAWER_X_M/2],
                        [4.7752, 5.1562, self.DRAWER_X_M/2],
                        [4.7752, 5.7277, self.DRAWER_X_M/2],
                        [4.7752, 6.3119, self.DRAWER_X_M/2],
                        [4.7752, 6.8834, self.DRAWER_X_M/2],
                        [4.7752, 8.04418, self.DRAWER_X_M/2]])

        self.no_drawers = np.shape(pos_m)[0]
        
        self.drawers = []
        for i in range(self.no_drawers):
            drawer = p.loadURDF(URDF_FOLDER_NAME + "msl/drawer.urdf", pos_m[i], startOrientation)
            self.drawers.append(drawer)

        # Add wall
        #wall = p.loadURDF(URDF_FOLDER_NAME + "corner.urdf", [0,1.5,1.25], [0,0,0,1])

class CornerEnv(object):
    def __init__(self, startPos=[0.0, 2.0, 1.25], startOrientationEuler=[0., 0., 0.]):
        startOrientation = p.getQuaternionFromEuler(startOrientationEuler)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.corner = p.loadURDF(
            URDF_FOLDER_NAME + "corner.urdf", startPos, startOrientation, useFixedBase=1)
