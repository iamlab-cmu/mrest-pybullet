""" Constat value definitions """

""" Ballbot Model Parameters """


from typing import Tuple
USE_ROS = False
NON_ROS_PATH = '/home/saumyas/ballbot_sim_py3_ws/src/'
# URDF_NAME = "/ballbot_arm_description/robots/urdf/ballbot_plus_pybullet.urdf"
URDF_NAME = "/ballbot_pybullet_sim/urdf/ballbot_pybullet_wBarrettHands_toolLR.urdf"
# URDF_NAME = "/ballbot_arm_description/robots/urdf/ballbot_pybullet_wBarrettHands2.urdf"
# URDF_NAME = "/ballbot_arm_description/robots/urdf/ballbot_pybullet_wBarrettHands.urdf"
# URDF_NAME = "/ballbot_arm_description/robots/urdf/test_barrett_cam.urdf"

""" Ballbot Control Parameters """
BALLBOT_TIME_PERIOD_MS = 2  # milliseconds

""" Ballbot Physical Parameters"""
BALLBOT_BALL_RADIUS_M = 0.105838037  # meters
BALL_ROLLER_RADIUS_M = 0.006335  # meters
BALLBOT_COM_M = 0.69  # meters

""" Ballbot Model Parameters """
# Arm joint friction properties
SJ100_STATIC_FRICTION = 3  # dimensionless
JOINT_DYNAMIC_FRICTION = 50  # dimensionless
SJ75_STATIC_FRICTION = 3  # dimensionless
LARM_JOINT_FRICTION = (SJ100_STATIC_FRICTION, SJ100_STATIC_FRICTION, SJ100_STATIC_FRICTION,
                       SJ100_STATIC_FRICTION, SJ75_STATIC_FRICTION, SJ75_STATIC_FRICTION, SJ75_STATIC_FRICTION)
RARM_JOINT_FRICTION = (SJ100_STATIC_FRICTION, SJ100_STATIC_FRICTION, SJ100_STATIC_FRICTION,
                       SJ100_STATIC_FRICTION, SJ75_STATIC_FRICTION, SJ75_STATIC_FRICTION, SJ75_STATIC_FRICTION)
ARMS_JOINT_FRICTION = RARM_JOINT_FRICTION + LARM_JOINT_FRICTION
TURRET_JOINT_FRICTION = (SJ100_STATIC_FRICTION, SJ100_STATIC_FRICTION)
STICTION_VEL_THRESHOLD = 0.01  # Rad/sec
BARRETT_HAND_JOINT_FRICTION = SJ100_STATIC_FRICTION

LARM_JOINT_NAMES = ('JLA1', 'JLA2', 'JLA3', 'JLA4', 'JLA5', 'JLA6', 'JLA7')
RARM_JOINT_NAMES = ('JRA1', 'JRA2', 'JRA3', 'JRA4', 'JRA5', 'JRA6', 'JRA7')
ARMS_JOINT_NAMES = LARM_JOINT_NAMES + RARM_JOINT_NAMES

# Turret joint names
TURRET_JOINT_NAMES = ('turret_pan', 'turret_tilt')

# Body joint names
BODY_JOINT_NAMES = ('xAngle', 'yAngle', 'yaw')

# Knob joint names
KNOB_JOINT_NAMES = ('JL_KnobHand', 'JR_KnobHand')

# Barrett left hand joint and link names
BARRETT_LEFT_JOINT_NAMES = ('L_bh_j32_joint', 'L_bh_j33_joint', 'L_bh_j11_joint', 'L_bh_j12_joint', 'L_bh_j13_joint', 'L_bh_j21_joint', 'L_bh_j22_joint', 'L_bh_j23_joint')
BARRETT_LEFT_LINK_NAMES = ('L_bh_base_link', 'L_bh_finger_31_link', 'L_bh_finger_32_link', 'L_bh_finger_33_link', 'L_bh_finger_11_link', 'L_bh_finger_12_link', 'L_bh_finger_13_link', 'L_bh_finger_21_link', 'L_bh_finger_22_link', 'L_bh_finger_23_link')

# Barrett right hand joint and link names
BARRETT_RIGHT_JOINT_NAMES = ('R_bh_j32_joint', 'R_bh_j33_joint', 'R_bh_j11_joint', 'R_bh_j12_joint', 'R_bh_j13_joint', 'R_bh_j21_joint', 'R_bh_j22_joint', 'R_bh_j23_joint')
BARRETT_RIGHT_LINK_NAMES = ('R_bh_base_link', 'R_bh_finger_31_link', 'R_bh_finger_32_link', 'R_bh_finger_33_link', 'R_bh_finger_11_link', 'R_bh_finger_12_link', 'R_bh_finger_13_link', 'R_bh_finger_21_link', 'R_bh_finger_22_link', 'R_bh_finger_23_link')

JOINT_NAMES = BODY_JOINT_NAMES + RARM_JOINT_NAMES + \
    LARM_JOINT_NAMES + TURRET_JOINT_NAMES

""" Ballbot End-Effectors """
REND_EFFECTOR_NAME = "toolR"
LEND_EFFECTOR_NAME = "toolL"

""" Ballbot Sensor Parameters """
BODY_LASER_LINK_NAME = "laser"   # name of body laser link in URDF
ENABLE_LASER = False

TURRET_CAMERA_LINK_NAME = "camera_rgb_frame"   # name of turret camera link in URDF
ENABLE_TURRET_CAMERA = True
ENABLE_STATIC_CAMERA = True

FT_SENSOR_JOINT_NAMES = ('RArm7-toolR','LArm7-toolL')

BALL_JOINT_NAME = "JBodytoBall"