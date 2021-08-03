import math
import numpy as np


def drawInertiaBox(pb, parentUid, parentLinkIndex, color):
    dyn = pb.getDynamicsInfo(parentUid, parentLinkIndex)
    mass = dyn[0]
    frictionCoeff = dyn[1]
    inertia = dyn[2]
    if (mass > 0):
        Ixx = inertia[0]
        Iyy = inertia[1]
        Izz = inertia[2]
        boxScaleX = 0.5 * math.sqrt(6 * (Izz + Iyy - Ixx) / mass)
        boxScaleY = 0.5 * math.sqrt(6 * (Izz + Ixx - Iyy) / mass)
        boxScaleZ = 0.5 * math.sqrt(6 * (Ixx + Iyy - Izz) / mass)

        halfExtents = [boxScaleX, boxScaleY, boxScaleZ]
        pts = [[halfExtents[0], halfExtents[1], halfExtents[2]],
               [-halfExtents[0], halfExtents[1], halfExtents[2]],
               [halfExtents[0], -halfExtents[1], halfExtents[2]],
               [-halfExtents[0], -halfExtents[1], halfExtents[2]],
               [halfExtents[0], halfExtents[1], -halfExtents[2]],
               [-halfExtents[0], halfExtents[1], -halfExtents[2]],
               [halfExtents[0], -halfExtents[1], -halfExtents[2]],
               [-halfExtents[0], -halfExtents[1], -halfExtents[2]]]

        pb.addUserDebugLine(pts[0],
                            pts[1],
                            color,
                            1,
                            parentObjectUniqueId=parentUid,
                            parentLinkIndex=parentLinkIndex)
        pb.addUserDebugLine(pts[1],
                            pts[3],
                            color,
                            1,
                            parentObjectUniqueId=parentUid,
                            parentLinkIndex=parentLinkIndex)
        pb.addUserDebugLine(pts[3],
                            pts[2],
                            color,
                            1,
                            parentObjectUniqueId=parentUid,
                            parentLinkIndex=parentLinkIndex)
        pb.addUserDebugLine(pts[2],
                            pts[0],
                            color,
                            1,
                            parentObjectUniqueId=parentUid,
                            parentLinkIndex=parentLinkIndex)

        pb.addUserDebugLine(pts[0],
                            pts[4],
                            color,
                            1,
                            parentObjectUniqueId=parentUid,
                            parentLinkIndex=parentLinkIndex)
        pb.addUserDebugLine(pts[1],
                            pts[5],
                            color,
                            1,
                            parentObjectUniqueId=parentUid,
                            parentLinkIndex=parentLinkIndex)
        pb.addUserDebugLine(pts[2],
                            pts[6],
                            color,
                            1,
                            parentObjectUniqueId=parentUid,
                            parentLinkIndex=parentLinkIndex)
        pb.addUserDebugLine(pts[3],
                            pts[7],
                            color,
                            1,
                            parentObjectUniqueId=parentUid,
                            parentLinkIndex=parentLinkIndex)

        pb.addUserDebugLine(pts[4 + 0],
                            pts[4 + 1],
                            color,
                            1,
                            parentObjectUniqueId=parentUid,
                            parentLinkIndex=parentLinkIndex)
        pb.addUserDebugLine(pts[4 + 1],
                            pts[4 + 3],
                            color,
                            1,
                            parentObjectUniqueId=parentUid,
                            parentLinkIndex=parentLinkIndex)
        pb.addUserDebugLine(pts[4 + 3],
                            pts[4 + 2],
                            color,
                            1,
                            parentObjectUniqueId=parentUid,
                            parentLinkIndex=parentLinkIndex)
        pb.addUserDebugLine(pts[4 + 2],
                            pts[4 + 0],
                            color,
                            1,
                            parentObjectUniqueId=parentUid,
                            parentLinkIndex=parentLinkIndex)


def computeCOMposVel(pb, uid: int):
    """Compute center-of-mass position and velocity."""
    jointIndices = range(pb.getNumJoints(uid))
    link_states = pb.getLinkStates(uid, jointIndices, computeLinkVelocity=1)
    link_pos = np.array([s[0] for s in link_states])
    link_vel = np.array([s[-2] for s in link_states])
    tot_mass = 0.
    masses = []
    for j in jointIndices:
        mass_, *_ = pb.getDynamicsInfo(uid, j)
        masses.append(mass_)
        tot_mass += mass_

    # add base position and velocity (Link_Body, id -1)
    body_mass, *_ = pb.getDynamicsInfo(uid, -1)
    tot_mass += body_mass

    body_position, body_orientation = pb.getBasePositionAndOrientation(uid)
    body_vel_linear, body_vel_angular = pb.getBaseVelocity(uid)

    masses = np.asarray(masses)[:, None]
    com_pos = np.sum(masses * link_pos, axis=0) / tot_mass
    com_pos += body_mass * np.asarray(body_position) / tot_mass
    com_vel = np.sum(masses * link_vel, axis=0) / tot_mass
    com_vel += body_mass * np.asarray(body_vel_linear) / tot_mass
    return com_pos, com_vel
