#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
import carla
import numpy as np
from dataclasses import dataclass, field

@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    orientation: float = 0.0

@dataclass
class Velocity:
    long: float = 0.0
    lat: float = 0.0

@dataclass
class Acceleration:
    long: float = 0.0
    lat: float = 0.0

def transform_actor_kinematics_to_ego_frame(ego: carla.Actor, other: carla.Actor):
    global_to_ego_matrix = np.array(ego.get_transform().get_inverse_matrix())

    other_loc_g: carla.Location = other.get_location()
    other_loc_g_homog = np.array([other_loc_g.x, other_loc_g.y, other_loc_g.z, 1])
    other_loc_e_homog = global_to_ego_matrix.dot(other_loc_g_homog)

    global_to_ego_matrix_rot_only = global_to_ego_matrix
    # Cancel out translation as only rotation shall be applied to consider different directions, but similar abs value
    global_to_ego_matrix_rot_only[:, 3] = np.array([0, 0, 0, 1])

    other_speed_o: carla.Vector3D = other.get_velocity()
    other_speed_o_homog = np.array([other_speed_o.x, other_speed_o.y, other_speed_o.z, 1])
    other_speed_e_homog = global_to_ego_matrix_rot_only.dot(other_speed_o_homog)

    other_accel_o: carla.Vector3D = other.get_acceleration()
    other_accel_o_homog = np.array([other_accel_o.x, other_accel_o.y, other_accel_o.z, 1])
    other_accel_e_homog = global_to_ego_matrix_rot_only.dot(other_accel_o_homog)

    pos = Pose(other_loc_e_homog[0], other_loc_e_homog[1])
    speed = Velocity(other_speed_e_homog[0], other_speed_e_homog[1])
    acceleration = Acceleration(other_accel_e_homog[0], other_accel_e_homog[1])

    np_vec = np.array([other_loc_e_homog[0],
                       other_loc_e_homog[1],
                       other_speed_e_homog[0],
                       other_speed_e_homog[1],
                       other_accel_e_homog[0],
                       other_accel_e_homog[1]])

    return pos, speed, acceleration, np_vec


def transform_ego_kinematics_to_ego_frame(ego_actor):
    global_to_ego_matrix = np.array(ego_actor.get_transform().get_inverse_matrix())
    global_to_ego_matrix_rot_only = global_to_ego_matrix
    # Cancel out translation as only rotation shall be applied to consider different directions, but similar abs value
    global_to_ego_matrix_rot_only[:, 3] = np.array([0, 0, 0, 1])
    ego_speed_g: carla.Vector3D = ego_actor.get_velocity()
    ego_speed_g_homog = np.array([ego_speed_g.x, ego_speed_g.y, ego_speed_g.z, 1])
    ego_speed_e_homog = global_to_ego_matrix_rot_only.dot(ego_speed_g_homog)
    ego_accel_g: carla.Vector3D = ego_actor.get_acceleration()
    ego_accel_g_homog = np.array([ego_accel_g.x, ego_accel_g.y, ego_accel_g.z, 1])
    ego_accel_e_homog = global_to_ego_matrix_rot_only.dot(ego_accel_g_homog)
    np_vec = np.array([0, 0, ego_speed_e_homog[0], ego_speed_e_homog[1], ego_accel_e_homog[0], ego_accel_e_homog[1]])

    pos = Pose(0,0)  # coordinate frame is set to ego center
    speed = Velocity(ego_speed_e_homog[0], ego_speed_e_homog[1])
    acceleration = Acceleration(ego_accel_e_homog[0], ego_accel_e_homog[1])

    return pos, speed, acceleration, np_vec


def transform_global_pos_to_ego_frame(ego: carla.Actor, pos: carla.Location) -> Pose:
    global_to_ego_matrix = np.array(ego.get_transform().get_inverse_matrix())

    other_loc_g_homog = np.array([pos.x, pos.y, pos.z, 1])
    other_loc_e_homog = global_to_ego_matrix.dot(other_loc_g_homog)

    return Pose(x=other_loc_e_homog[0], y=other_loc_e_homog[1])