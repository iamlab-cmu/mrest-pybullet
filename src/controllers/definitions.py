""" Constat value definitions """

""" Ballbot Model Parameters """
URDF_NAME = "/ballbot_arm_description/robots/urdf/ballbot_plus_pybullet.urdf"

""" Ballbot Control Parameters """
BALLBOT_TIME_PERIOD_MS = 2 # milliseconds

""" Ballbot Physical Parameters"""
BALLBOT_BALL_RADIUS_M = 0.105838037 # meters
BALL_ROLLER_RADIUS_M = 0.006335 # meters
BALLBOT_COM_M = 0.69 # meters

""" Ballbot Model Parameters """
# Arm joint friction properties
JOINT_STATIC_FRICTION = 105 # dimensionless
JOINT_DYNAMIC_FRICTION = 5 # dimensionless
STICTION_VEL_THRESHOLD = 0.01 # Rad/sec


