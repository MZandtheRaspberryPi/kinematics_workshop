import math

# this directory is where plots will be saved and the trajectory as a binary file
CACHE_DIR = "/home/developer/ros_ws/src/ros_sim"
DEFAULT_EPS_XYZ = 0.002
DEFAULT_EPS_ORI = 0.01
DEFAULT_MAX_IT = 30
DEFAULT_ALPHA_GRAD_DESC_NO_ORI = 1.0
DEFAULT_ALPHA_GRAD_DESC_ORI = 0.35
POS_GAIN_IK = 25
ORI_GAIN_IK = 0.2
# JOINT_ANGLE_DIFF_TOL = 0.06
JOINT_ANGLE_DIFF_TOL = 5 * math.pi / 180  # 5 degrees?
PAPER_HEIGHT = 0.01  # in sim
# PAPER_HEIGHT = -0.001  # in reality
# intrinsic
ROTATION_SCHEMA = "xyz"

LINK_NAME = "wand_tip"
N_DOF = 6

# d, a, alpha, theta
# TODO: Modify to include the base of the robot and the pen. For fixed frames, insert -1 into the DH_PARAMS_MYCOBOT_JOINT_IDX array.
DH_PARAMS_MYCOBOT_JOINT_IDX = (0, 1, 2, 3, 4, 5)
DH_PARAMS_MYCOBOT = (
    (0.13122, 0, 1.5708, 0),
    (0, -0.1104, 0, -1.5708),
    (0, -0.096, 0, 0),
    (0.0634, 0, 1.5708, -1.5708),
    (0.07505, 0, -1.5708, 1.5708),
    (0.0456, 0, 0, 0),
)


MYCOBOT_DOF_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]
