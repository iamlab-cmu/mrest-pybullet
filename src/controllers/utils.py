import quaternion
import numpy as np
import urdf_parser_py.urdf as urdf
import PyKDL as kdl

def angle_axis_between_quats(q0, q1):
    '''
    Finds dq s.t. dq * q1 = q0
    # Assumes q0 and q1 are of type np.quaternion
    '''
    if quaternion.as_float_array(q1) @ quaternion.as_float_array(q0) < 0:
        q0 = -q0
    dq = q0 * q1.inverse()
    return quaternion.as_rotation_vector(dq)