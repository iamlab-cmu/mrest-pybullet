""" Constat value definitions """

""" Ballbot Model Parameters """
URDF_NAME = "/ballbot_arm_description/robots/urdf/ballbot_plus_pybullet.urdf"

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

LARM_JOINT_NAMES = ('JLA1', 'JLA2', 'JLA3', 'JLA4', 'JLA5', 'JLA6', 'JLA7')
RARM_JOINT_NAMES = ('JRA1', 'JRA2', 'JRA3', 'JRA4', 'JRA5', 'JRA6', 'JRA7')
ARMS_JOINT_NAMES = LARM_JOINT_NAMES + RARM_JOINT_NAMES
STICTION_VEL_THRESHOLD = 0.01  # Rad/sec

""" Ballbot Sensor Parameters """
BODY_LASER_LINK_NAME = "laser"   # name of body laser link in URDF
