""" a set of useful little numpy functions.

TODO: having trouble properly sharing this file between different packages.
"""

import numpy as np

def wrap(angle):
    """Takes in any angle (in radians), and expresses it inside the [-np.pi, np.pi] range.
    
    Arguments:
        angle {float} -- An angle in radians
    
    Returns:
        float -- The angle projected within the range (-np.pi, np.pi]
    """
    two_pi = 2 * np.pi
    angle %= two_pi
    if angle < 0:
        angle += two_pi
    # angle is now inside [0, 2pi]
    if angle > np.pi:
        angle -= two_pi
    assert (- np.pi) < angle <= np.pi
    return angle

def R(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)],
    ])

def rotation(theta):
    return R(theta)

def cos_and_sin(x):
    return np.array([
        np.cos(x),
        np.sin(x),
    ])

def rotation_then_translation(theta_rad, Ax, Ay):
    return np.array([
        [np.cos(theta_rad), -np.sin(theta_rad), Ax],
        [np.sin(theta_rad),  np.cos(theta_rad), Ay],
        [0, 0, 1],
    ])

def point_to_np(point):
    return np.array([
        point.x,
        point.y,
        point.z,
    ])

def points_to_np(points_list):
    return np.array([
        [point.x, point.y, point.z] for point in points_list
    ])

