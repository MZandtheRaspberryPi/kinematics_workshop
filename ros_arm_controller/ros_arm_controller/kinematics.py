import copy
import math
from typing import Tuple, Optional

import numpy as np
from numpy.typing import ArrayLike

import jax.typing as jt
import jax
import jax.numpy as jnp
from jax.typing import ArrayLike
from jax.scipy.spatial.transform import Rotation

from ros_arm_controller.constants import (
    DEFAULT_EPS_XYZ,
    DEFAULT_EPS_ORI,
    DH_PARAMS_MYCOBOT,
    DH_PARAMS_MYCOBOT_JOINT_IDX,
    MYCOBOT_DOF_NAMES,
    DEFAULT_MAX_IT,
    DEFAULT_ALPHA_GRAD_DESC_NO_ORI,
    DEFAULT_ALPHA_GRAD_DESC_ORI,
    ROTATION_SCHEMA,
    N_DOF,
    POS_GAIN_IK,
    ORI_GAIN_IK,
)

DH_PARAMS_MYCOBOT_JAX = jnp.array(DH_PARAMS_MYCOBOT)


def standard_dh(a, alpha, d, theta) -> ArrayLike:
    """This function computes the homogeneous 4x4 transformation matrix T_i based on the four standard DH parameters
        associated with link i and joint i.
    Args:
        a ([int, float]): Link Length. The distance along x_i ( the common normal) between z_{i-1} and z_i
        alpha ([int, float]): Link twist. The angle between z_{i-1} and z_i around x_i.
        d ([int, float]): Link Offset. The distance along z_{i-1} between x_{i-1} and x_i.
        theta ([int, float]): Joint angle. The angle between x_{i-1} and x_i around z_{i-1}
    Returns:
        [np.ndarray]: the 4x4 transformation matrix T_i describing  a coordinate transformation from
        the concurrent coordinate system i to the previous coordinate system i-1
    """
    assert isinstance(a, (int, float, jax.numpy.floating)), "wrong input type for a"
    assert isinstance(alpha, (int, float)), "wrong input type for =alpha"
    assert isinstance(d, (int, float)), "wrong input type for d"
    assert isinstance(theta, (int, float)), "wrong input type for theta"
    A = np.zeros((4, 4))

    # TODO: implement a method to get the transform matrix using DH Parameters

    assert isinstance(A, np.ndarray), "Output wasn't of type ndarray"
    assert A.shape == (4, 4), "Output had wrong dimensions"
    return A


def get_transform_last_frame(joint_positions: Tuple[float]) -> ArrayLike:
    """calculate forward kinematics, the position of the last frame in the DH parameters

    Args:
        joint_positions (Tuple[float]): joint_positions

    Returns:
        ArrayLike: transform matrix, 4x4
    """
    assert len(joint_positions) == N_DOF

    T = np.identity(4)

    for i in range(len(DH_PARAMS_MYCOBOT)):

        d = DH_PARAMS_MYCOBOT[i][0]
        a = DH_PARAMS_MYCOBOT[i][1]
        alpha = DH_PARAMS_MYCOBOT[i][2]
        theta = DH_PARAMS_MYCOBOT[i][3]

        joint_idx = DH_PARAMS_MYCOBOT_JOINT_IDX[i]

        joint_angle = 0.0
        if joint_idx != -1:
            joint_angle = joint_positions[joint_idx]

        A = standard_dh(a, alpha, d, theta + joint_angle)

        T = T.dot(A)
    assert isinstance(T, np.ndarray), "Output wasn't of type ndarray"
    assert T.shape == (4, 4), "Output had wrong dimensions"
    return T


@jax.jit
def standard_dh_jax(
    a: jnp.floating, alpha: jnp.floating, d: jnp.floating, theta: jnp.floating
):
    """Jax version of DH parameters, so that it is differentiable using Jax.

    Args:
        a ([int, float]): Link Length. The distance along x_i ( the common normal) between z_{i-1} and z_i
        alpha ([int, float]): Link twist. The angle between z_{i-1} and z_i around x_i.
        d ([int, float]): Link Offset. The distance along z_{i-1} between x_{i-1} and x_i.
        theta ([int, float]): Joint angle. The angle between x_{i-1} and x_i around z_{i-1}

    Returns:
        ArrayLike: transform matrix, 4x4
    """
    A = jnp.zeros((4, 4))

    A = A.at[0, 0].set(jnp.cos(theta))
    A = A.at[0, 1].set(-jnp.sin(theta) * jnp.cos(alpha))
    A = A.at[0, 2].set(jnp.sin(theta) * jnp.sin(alpha))
    A = A.at[0, 3].set(a * jnp.cos(theta))

    A = A.at[1, 0].set(jnp.sin(theta))
    A = A.at[1, 1].set(jnp.cos(theta) * jnp.cos(alpha))
    A = A.at[1, 2].set(-jnp.cos(theta) * jnp.sin(alpha))
    A = A.at[1, 3].set(a * jnp.sin(theta))

    A = A.at[2, 1].set(jnp.sin(alpha))
    A = A.at[2, 2].set(jnp.cos(alpha))
    A = A.at[2, 3].set(d)

    A = A.at[3, 3].set(1.0)

    return A


def rot_to_euler(rot_mat: jt.ArrayLike) -> jt.ArrayLike:
    """Convenience function to get euler angles from a rotation matrix representation, using Jax to be differentiable.

    Args:
        rot_mat (jt.ArrayLike): 3x3 rotation matrix, jax type

    Returns:
        jt.ArrayLike: vector of length 3, euler angles according to constant ROTATION_SCHEMA
    """
    rotation_mat = Rotation.from_matrix(rot_mat)
    euler = rotation_mat.as_euler(ROTATION_SCHEMA, degrees=False)
    return euler


def euler_to_quat(euler: jt.ArrayLike) -> jt.ArrayLike:
    """Convenience function to get quaternion from euler angles representation, using Jax to be differentiable.

    Args:
        euler (jt.ArrayLike): vector of length 3, euler angles according to constant ROTATION_SCHEMA

    Returns:
        jt.ArrayLike: vector of length 4, quaternion
    """
    rot = Rotation.from_euler(ROTATION_SCHEMA, euler, degrees=False)
    quat = rot.as_quat(scalar_first=False)
    return quat


def quat_to_euler(quat: jt.ArrayLike) -> jt.ArrayLike:
    """Convenience function to get euler angles from quaternion representation, using Jax to be differentiable.

    Args:
        quat (jt.ArrayLike): vector of length 4, quaternion

    Returns:
        jt.ArrayLike: vector of length 3, euler angles according to constant ROTATION_SCHEMA
    """
    # scalar first false
    rot = Rotation.from_quat(quat)
    euler = rot.as_euler(ROTATION_SCHEMA, degrees=False)
    return euler


@jax.jit
def get_transform_last_frame_jax(jax_joint_positions: jt.ArrayLike) -> jt.ArrayLike:
    """calculate forward kinematics, the position of the last frame in the DH parameters using Jax to be differentiable.

    Args:
        joint_positions (jt.ArrayLike): joint_positions

    Returns:
        jt.ArrayLike: transform matrix, 4x4
    """
    T = jax.numpy.identity(4)

    for i in range(DH_PARAMS_MYCOBOT_JAX.shape[0]):
        d = DH_PARAMS_MYCOBOT_JAX[i][0]
        a = DH_PARAMS_MYCOBOT_JAX[i][1]
        alpha = DH_PARAMS_MYCOBOT_JAX[i][2]
        theta = DH_PARAMS_MYCOBOT_JAX[i][3]

        joint_angle = jax_joint_positions[i]

        A = standard_dh_jax(a, alpha, d, theta + joint_angle)

        T = T.dot(A)

    euler = rot_to_euler(T[:3, :3])

    pos = T[:3, 3]

    return jnp.concat((pos, euler))


# here create a function to get the gradient of the forward kinematics (jacobian) and compile it using just-in-time compilation
grad_transform_last_frame = jax.jacobian(get_transform_last_frame_jax, argnums=0)
grad_transform_last_frame = jax.jit(grad_transform_last_frame)


def get_jacobian_last_frame_jax(joint_positions: Tuple[float]) -> ArrayLike:
    """Get the jacobian of the last frame in the DH parameters using autodifferention (Jax).

    Args:
        joint_positions (Tuple[float]): joint positions

    Returns:
        ArrayLike: Jacobian, 6x6 matrix
    """
    # add buffer for the dh frames that have fixed joints
    joint_pos = []
    for i in range(len(DH_PARAMS_MYCOBOT_JOINT_IDX)):
        idx = DH_PARAMS_MYCOBOT_JOINT_IDX[i]
        if idx == -1:
            joint_pos.append(0.0)
        else:
            joint_pos.append(joint_positions[idx])
    jax_pos = jnp.array(joint_pos)
    jacobian = grad_transform_last_frame(jax_pos)
    return np.array(jacobian[:, [1, 2, 3, 4, 5, 6]]).tolist()


def get_ik(
    target_link_xyz: Tuple[float],
    cur_angles: Tuple[float],
    target_ori_rpy_radians: Optional[Tuple[float]] = None,
    eps_xyz: float = DEFAULT_EPS_XYZ,
    eps_ori: float = DEFAULT_EPS_ORI,
    max_iter: int = DEFAULT_MAX_IT,
    pos_gain: float = POS_GAIN_IK,
    ori_gain: float = ORI_GAIN_IK,
) -> Tuple[Tuple[float], bool, bool, bool]:
    """Calculates inverse kinematics using iterative gradient ascent

    Args:
        target_link_xyz (Tuple[float]): target position of the end effector, 3 dimensions
        cur_angles (Tuple[float]): current angles, 6 dimensions
        target_ori_rpy_radians (Optional[Tuple[float]], optional): target orientation in euler angles (radians). Can be None for just position. Defaults to None.
        eps_xyz (float, optional): tolerance controlling when to exit iterations compared to position error. Defaults to DEFAULT_EPS_XYZ.
        eps_ori (float, optional): tolerance controlling when to exit iterations compared to orientation error. Defaults to DEFAULT_EPS_ORI.
        max_iter (int, optional): maximum number of iterations. Defaults to DEFAULT_MAX_IT.
        pos_gain (float, optional): gain on the position error for gradient ascent. Defaults to POS_GAIN_IK.
        ori_gain (float, optional): gain on the orientation error for gradient ascent. Defaults to ORI_GAIN_IK.

    Returns:
        Tuple[Tuple[float], bool, bool, bool]: joint angle solutions, whether overall succesful in terms of iterations, position, and orientation, whether succesful in terms of position, whether succesful in terms of orientation
    """

    # TODO: use the given algorithm to calculate a solution to the given task vector

    angles_sol = [0.0] * len(cur_angles)
    cur_it = 0
    pos_err = 1000
    ori_err = 1000

    assert type(angles_sol) == list, type(angles_sol)
    assert len(angles_sol) == len(cur_angles)

    return (
        angles_sol.flatten().tolist(),
        cur_it < max_iter and (pos_err < eps_xyz and ori_err < eps_ori),
        pos_err < eps_xyz,
        ori_err < eps_ori,
    )


def get_ik_around_cur_angles(
    target_link_xyz: Tuple[float],
    cur_angles: Tuple[float],
    target_ori_rpy_radians: Optional[Tuple[float]] = None,
    eps_xyz: float = DEFAULT_EPS_XYZ,
    eps_ori: float = DEFAULT_EPS_ORI,
    max_iter: int = DEFAULT_MAX_IT,
) -> Tuple[Tuple[float], bool]:
    """Wrapper to call IK as many times as needed until an adequate solution is found

    Args:
        target_link_xyz (Tuple[float]): target position of the end effector, 3 dimensions
        cur_angles (Tuple[float]): current angles, 6 dimensions
        target_ori_rpy_radians (Optional[Tuple[float]], optional): target orientation in euler angles (radians). Can be None for just position. Defaults to None.
        eps_xyz (float, optional): tolerance controlling when to exit iterations compared to position error. Defaults to DEFAULT_EPS_XYZ.
        eps_ori (float, optional): tolerance controlling when to exit iterations compared to orientation error. Defaults to DEFAULT_EPS_ORI.
        max_iter (int, optional): maximum number of iterations. Defaults to DEFAULT_MAX_IT.

    Returns:
        Tuple[Tuple[float], bool]: joint angles of solution, whether it converged
    """

    sol, success, pos_within, ori_within = get_ik(
        target_link_xyz,
        cur_angles=cur_angles,
        target_ori_rpy_radians=target_ori_rpy_radians,
        eps_xyz=eps_xyz,
        eps_ori=eps_ori,
        max_iter=max_iter,
    )

    cur_it = 0
    while not success:
        joint_angle_change = np.linalg.norm(sol - np.array(cur_angles))

        new_angles = copy.copy(sol)
        sol, success, pos_within, ori_within = get_ik(
            target_link_xyz,
            cur_angles=new_angles,
            target_ori_rpy_radians=target_ori_rpy_radians,
            eps_xyz=eps_xyz,
            eps_ori=eps_ori,
            max_iter=max_iter,
        )
        cur_it += 1

        if cur_it > 15:
            break

    for i in range(len(sol)):
        while sol[i] < -np.pi:
            sol[i] += 2 * np.pi
        while sol[i] > np.pi:
            sol[i] -= 2 * np.pi

    return sol, success
