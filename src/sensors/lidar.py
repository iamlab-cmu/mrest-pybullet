import pybullet as p
import numpy as np
import time
import math
import pybullet_data


class Lidar:
    def __init__(self, angle_min=-1.51, angle_max=1.51):
        '''
            Initialize lidary

            angle_min: 
                Controls the angle of the first range measurement in radians.

            angle_max: 
                Controls the angle of the last range measurement in radians
        '''

        # Lidar Properties
        self.rayLen = 13  # Range of Laser [m]
        self.rayDelta = 0.1  # Angle between rays [rad]
        self.angle_min = angle_min
        self.angle_max = angle_max

        # Lidar Data
        self.rayFrom = []
        self.rayTo = []
        self.rayCastAngles = np.arange(
            self.angle_min, self.angle_max, self.rayDelta)
        self.rayIds = []

        # Color of rays for visualization
        self.rayHitColor = [1, 0, 0]    # Red
        self.rayMissColor = [0, 1, 0]   # Green
        self.visualizeHit = True        # Enable visualizing rays that hit objects
        self.visualizeMiss = True      # Enable visualizing rayus that miss objects

        # Initialize Lidar rays
        self.update_rays()

        # Initialize rays for visualization
        for i in range(len(self.rayCastAngles)):
            self.rayIds.append(p.addUserDebugLine(
                self.rayFrom[i], self.rayFrom[i], self.rayMissColor))

    def removeAllRaysVis(self):
        for i in range(len(self.rayCastAngles)):
            p.removeUserDebugItem(self.rayIds[i])

    def update_rays(self, fromPos=[0, 0, 1]):
        """
            Updates the ray source and end point

            fromPos 
                3D position of source point from where all rays radiated out [x,y,z]
        """

        # Clear previous
        self.rayFrom.clear()
        self.rayTo.clear()

        for i in self.rayCastAngles:
            self.rayFrom.append(fromPos)
            self.rayTo.append([self.rayLen * math.sin(i),
                              self.rayLen * math.cos(i), 1])

    def update(self):
        """
            Cast lidar rays and find where it hit
        """

        results = p.rayTestBatch(self.rayFrom, self.rayTo)

        for i in range(len(self.rayCastAngles)):
            hitObjectUid = results[i][0]

            if (hitObjectUid < 0 and self.visualizeMiss):
                hitPosition = [0, 0, 0]
                p.addUserDebugLine(
                    self.rayFrom[i], self.rayTo[i], self.rayMissColor, replaceItemUniqueId=self.rayIds[i])
            elif(self.visualizeHit):
                hitPosition = results[i][3]
                p.addUserDebugLine(
                    self.rayFrom[i], hitPosition, self.rayHitColor, replaceItemUniqueId=self.rayIds[i])
