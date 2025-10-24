import numpy as np

MAX_FORCE = 20.0

DOF_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]

JOINT_LIMITS = [
    [-165, 165],
    [-165, 165],
    [-165, 165],
    [-165, 165],
    [-165, 165],
    [-179, 179],
]
JOINT_LIMITS = np.array(JOINT_LIMITS)
JOINT_LIMITS = np.radians(JOINT_LIMITS)
JOINT_DAMPING_COEF = [1.1] * JOINT_LIMITS.shape[0]

MAX_FORCES = [MAX_FORCE for i in range(JOINT_LIMITS.shape[0])]

POS_GAIN = [0.4 for i in range(len(JOINT_LIMITS))]
VEL_GAIN = [1.0 for i in range(len(JOINT_LIMITS))]
