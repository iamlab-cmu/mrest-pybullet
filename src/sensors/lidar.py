#!/usr/bin/env python
#  Copyright Microdynamic Systems Laboratory 2021
#
# @author Roberto Shu <rshum@cmu.edu>
#
# @brief Class to simulate a lidar in pybullet. 
# It uses pybullet rayTestBatch() function to determine hit obstacles
#
import pybullet as p
import numpy as np
import time
import math
import pybullet_data


class Lidar:
    def __init__(self, bodyUniqueId, parentLinkId, angle_min=-1.51, angle_max=1.51, visualizeHit = True, visualizeMiss = True):
        '''
            Initialize lidary

            angle_min: 
                Controls the angle of the first range measurement in radians.

            angle_max: 
                Controls the angle of the last range measurement in radians
        '''
        # Mechanical Properties
        self.bodyUniqueId = bodyUniqueId
        self.parentLinkId = parentLinkId

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
        self.hitRanges = [] # range data of to hit[m]

        # Color of rays for visualization
        self.rayHitColor = [1, 0, 0]    # Red
        self.rayMissColor = [0, 1, 0]   # Green
        self.visualizeHit = visualizeHit        # Enable visualizing rays that hit objects
        self.visualizeMiss = visualizeMiss     # Enable visualizing rayus that miss objects

        # Initialize Lidar rays
        self.update_rays()

        # Initialize rays for visualization
        for i in range(len(self.rayCastAngles)):
            self.rayIds.append(p.addUserDebugLine(
                self.rayFrom[i], self.rayFrom[i], self.rayMissColor))

    def removeAllRaysVis(self):
        for i in range(len(self.rayCastAngles)):
            p.removeUserDebugItem(self.rayIds[i])

    def update_rays(self):
        """
            Updates the ray source and end point
        """
        
        # Update lidar link base position
        linkState = p.getLinkState(self.bodyUniqueId,self.parentLinkId)
        basePos = linkState[0]
        baseOrnt = p.getEulerFromQuaternion(linkState[1])

        # Clear previous
        self.rayFrom.clear()
        self.rayTo.clear()

        for i in self.rayCastAngles:
            self.rayFrom.append(basePos)
            self.rayTo.append([self.rayLen * math.cos(i+baseOrnt[2]),
                              self.rayLen * math.sin(i+baseOrnt[2]), basePos[2]])

    def update(self):
        """
            Cast lidar rays and find where it hit
        """

        self.update_rays()

        results = p.rayTestBatch(self.rayFrom, self.rayTo)

        self.hitRanges.clear()
        
        for i in range(len(self.rayCastAngles)):
            hitObjectUid = results[i][0]

            if (hitObjectUid < 0 and self.visualizeMiss):
                hitPosition = [0, 0, 0]
                self.hitRanges.append(hitPosition[2])
                p.addUserDebugLine(
                    self.rayFrom[i], self.rayTo[i], self.rayMissColor, replaceItemUniqueId=self.rayIds[i])
            elif(self.visualizeHit):
                hitPosition = results[i][3]
                self.hitRanges.append(hitPosition[2])
                p.addUserDebugLine(
                    self.rayFrom[i], hitPosition, self.rayHitColor, replaceItemUniqueId=self.rayIds[i])

        return self.hitRanges