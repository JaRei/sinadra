#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
import math
import numpy as np
from sinadra_configuration_parameters import EMERGENCY_ACC_MEAN, EMERGENCY_ACC_STD, EMERGENCY_POS_STD, \
    CONST_ACCEL_MEAN, CONST_ACCEL_STD, CONST_ACCEL_POS_STD, TB_POS_STD, TB_MAX_DECELERATION, IDM_POS_STD, \
    IDM_TIMEGAP_STD, IDM_TIME_GAP_FRONT_VEHICLE, S_0, V_DESIRED, DELTA, A_MAX, B_COMFORT

##########################################################################################
# Trajectory Distribution Generators (Longitudinal Behaviors)
##########################################################################################


def gen_emergency_brake(kinematic_init, num_traj, time_horizon, time_inc):
    # Generate emergency deceleration and position uncertainty samples
    d_samples = np.random.normal(EMERGENCY_ACC_MEAN, EMERGENCY_ACC_STD, num_traj)  # [1,...,num_traj]
    pos_sample_size = int((time_horizon / time_inc + 1) * num_traj)
    pos_samples = np.random.normal(0.0, EMERGENCY_POS_STD, pos_sample_size)

    init = kinematic_init # = np.array([p_x, p_y, v_x, v_y, a_x, a_y])
    # Duplicate initial position for all trajectories to be generated
    time0 = np.tile(init, (num_traj, 1))
    for i in range(0, num_traj):
        traj = time0[i]
        traj[0] = kinematic_init[0] + pos_samples[i]
    trajectories = np.array(time0)
    next_time = time0
    for time_idx in np.arange(time_inc, time_horizon + time_inc, time_inc):
        next_time = np.array([emergency_decel_timestep(time_inc, next_time[i], d_samples[i],
                                                       pos_samples[int(1 / time_inc * time_idx) * num_traj + i]) for i
                              in range(next_time.shape[0])])
        trajectories = np.concatenate((trajectories, next_time))
    #sprint(trajectories)
    pos_list = trajectories[0:len(trajectories), 0].reshape((int(time_horizon / time_inc) + 1, num_traj))
    #print(pos_list)
    pos_mean = np.mean(pos_list, axis=1)
    #print(pos_mean)
    pos_std = np.std(pos_list, axis=1)
    #print(pos_std)
    return pos_mean, pos_std


def emergency_decel_timestep(time_inc, last, decel, pos_uncertainty):
    # p_t+1 = p_t + v_t * dt + a * dt * dt
    # v_t+1 = v_t + a * dt
    # a_t+1 = sampled deceleration

    if last[2] <= 0:
        stopped = last
        stopped[2] = 0.0  # Zero Velocity
        stopped[4] = 0.0  # Zero Acceleration
        return stopped
    return np.array([last[0] + last[2] * time_inc + decel * time_inc * time_inc + pos_uncertainty,
                     last[1],
                     last[2] + decel * time_inc,
                     last[3],
                     decel,
                     last[5]])


def gen_constant_accel(vehicle_init, num_traj, time_horizon, time_inc):
    # Generate constant deceleration and position uncertainty samples
    d_samples = np.random.normal(CONST_ACCEL_MEAN, CONST_ACCEL_STD, num_traj)  # [1,...,num_traj]
    pos_sample_size = int((time_horizon / time_inc + 1) * num_traj)
    pos_samples = np.random.normal(0.0, CONST_ACCEL_POS_STD, pos_sample_size)

    init = vehicle_init # = np.array([p_x, p_y, v_x, v_y, a_x, a_y])
    # Duplicate initial position for all trajectories to be generated
    time0 = np.tile(init, (num_traj, 1))
    for i in range(0, num_traj):
        traj = time0[i]
        traj[0] = vehicle_init[0] + pos_samples[i]
    trajectories = np.array(time0)
    next_time = time0
    for time_idx in np.arange(time_inc, time_horizon + time_inc, time_inc):
        next_time = np.array([constant_accel_timestep(time_inc, next_time[i], d_samples[i],
                                                      pos_samples[int(1 / time_inc * time_idx) * num_traj + i]) for i in
                              range(next_time.shape[0])])
        trajectories = np.concatenate((trajectories, next_time))
    # print(trajectories)
    pos_list = trajectories[0:len(trajectories), 0].reshape((int(time_horizon / time_inc) + 1, num_traj))
    # print("Ego constant accel:")
    # for i in range(num_traj):
    #     print("x:")
    #     print(pos_list[:, i].tolist())
    #     print("y:")
    #     print([0] * int(pos_sample_size / 2))
    # print("-----")
    pos_mean = np.mean(pos_list, axis=1)
    # print(pos_mean)
    pos_std = np.std(pos_list, axis=1)
    # print(pos_std)
    return pos_mean, pos_std


def constant_accel_timestep(time_inc, last, decel, pos_uncertainty):
    # p_t+1 = p_t + v_t * dt + a * dt * dt
    # v_t+1 = v_t + a * dt
    # a_t+1 = sampled deceleration

    if last[2] <= 0:
        stopped = last
        stopped[2] = 0.0  # Zero Velocity
        stopped[4] = 0.0  # Zero Acceleration
        return stopped
    return np.array([last[0] + last[2] * time_inc + (last[4] + decel) * time_inc * time_inc + pos_uncertainty,
                     last[1],
                     last[2] + (last[4] + decel) * time_inc,
                     last[3],
                     last[4] + decel,
                     last[5]])


def gen_targetbrake(vehicle_init, target_distance, target_safe_distance, num_traj, time_horizon, time_inc):
    # Generate position uncertainty samples
    pos_sample_size = int((time_horizon / time_inc + 1) * num_traj)
    pos_samples = np.random.normal(0.0, TB_POS_STD, pos_sample_size)

    init = vehicle_init  # = np.array([p_x, p_y, v_x, v_y, a_x, a_y])
    # Duplicate initial position for all trajectories to be generated
    time0 = np.tile(init, (num_traj, 1))
    for i in range(0, num_traj):
        traj = time0[i]
        traj[0] = vehicle_init[0] + pos_samples[i]
    trajectories = np.array(time0)

    ##########################################
    # Generate constant deceleration samples based on target distance variation
    #########################################

    d_samples = np.zeros(num_traj)

    # Standard deviation chosen according to Schreier paper
    # [1,...,num_traj]
    safe_distance_variation_samples = np.random.normal(0.0, (target_safe_distance / 3.0) ** 2, num_traj)

    for i in range(0, num_traj):
        # This is the safe distance accounting for the variation of distance from target among different drivers
        intended_stop_point_sample = target_distance - target_safe_distance + safe_distance_variation_samples[i]

        # Compute const. deceleration required to come to a stop at the safe distance before the braking target
        # Formula taken from Schreier approach

        dist_from_intended_stop_sample = intended_stop_point_sample - trajectories[i][0]
        if dist_from_intended_stop_sample > 0:
            req_deceleration = - (trajectories[i][2] ** 2) / (2 * dist_from_intended_stop_sample)
            chosen_deceleration = max(req_deceleration, TB_MAX_DECELERATION)
        else:
            chosen_deceleration = 0.0
        d_samples[i] = chosen_deceleration

    ##########################################
    # Generate time steps
    #########################################

    next_time = time0
    for time_idx in np.arange(time_inc, time_horizon + time_inc, time_inc):
        next_time = np.array([targetbrake_timestep(time_inc, next_time[i], d_samples[i],
                                                   pos_samples[int(1 / time_inc * time_idx) * num_traj + i]) for i in
                              range(next_time.shape[0])])
        trajectories = np.concatenate((trajectories, next_time))
    # print(trajectories)
    pos_list = trajectories[0:len(trajectories), 0].reshape((int(time_horizon / time_inc) + 1, num_traj))
    # print(pos_list)
    pos_mean = np.mean(pos_list, axis=1)
    # print(pos_mean)
    pos_std = np.std(pos_list, axis=1)
    # print(pos_std)
    return pos_mean, pos_std


def targetbrake_timestep(time_inc, last, decel, pos_uncertainty):
    # p_t+1 = p_t + v_t * dt + a * dt * dt
    # v_t+1 = v_t + a * dt
    # a_t+1 = sampled deceleration

    if last[2] <= 0:
        stopped = last
        stopped[2] = 0.0  # Zero Velocity
        stopped[4] = 0.0  # Zero Acceleration
        return stopped
    return np.array([last[0] + last[2] * time_inc + decel * time_inc * time_inc + pos_uncertainty,
                     last[1],
                     last[2] + decel * time_inc,
                     last[3],
                     decel,
                     last[5]])


def gen_idm(vehicle_init_rear, vehicle_length, vehicle_front_init_rear, num_traj, time_horizon, time_inc):
    """

    Parameters
    ----------
    vehicle_init_rear: Kinematic info of vehicle
    vehicle_length
    vehicle_front_init_rear
    num_traj
    time_horizon
    time_inc

    Returns
    -------
    (pos_mean: float, pos_std: float)
    Bla bla

    """

    # Generate constant deceleration and position uncertainty samples
    pos_sample_size = int((time_horizon / time_inc + 1) * num_traj)
    pos_samples = np.random.normal(0.0, IDM_POS_STD, pos_sample_size)
    timegap_samples = np.random.normal(0.0, IDM_TIMEGAP_STD, pos_sample_size)

    init = vehicle_init_rear  # init = np.array([p_x, p_y, v_x, v_y, a_x, a_y])
    # Duplicate initial position for all trajectories to be generated
    time0 = np.tile(init, (num_traj, 1))
    for i in range(0, num_traj):
        traj = time0[i]
        traj[0] = vehicle_init_rear[0] + pos_samples[i]
    trajectories = np.array(time0)

    next_time = time0
    for time_idx in np.arange(time_inc, time_horizon + time_inc, time_inc):
        next_time = np.array(
            [
                idm_timestep(
                    time_inc,
                    next_time[i],
                    vehicle_length,
                    vehicle_front_init_rear,
                    timegap_samples[int(1 / time_inc * time_idx) * num_traj + i],
                    pos_samples[int(1 / time_inc * time_idx) * num_traj + i]
                ) for i in range(next_time.shape[0])
            ]
        )
        trajectories = np.concatenate((trajectories, next_time))
    # print(trajectories)
    pos_list = trajectories[0:len(trajectories), 0].reshape((int(time_horizon / time_inc) + 1, num_traj))
    # print(pos_list)
    pos_mean = np.mean(pos_list, axis=1)
    # print(pos_mean)
    pos_std = np.std(pos_list, axis=1)
    # print(pos_std)
    return pos_mean, pos_std


def idm_timestep(time_inc, last, length_self, vehicle_front_init, timegap_sample, pos_uncertainty):
    # Intelligent Driver Model (IDM) formula, e.g. https://en.wikipedia.org/wiki/Intelligent_driver_model

    v_self = last[2]
    v_other = vehicle_front_init[2]
    pos_self = last[0] + length_self
    initial_pos_other = vehicle_front_init[0]  # Constant speed assumption regarding front front vehicle
    driven_distance_other = v_other * time_inc
    distance = initial_pos_other + driven_distance_other - pos_self

    # desired time headway: the minimum possible time to the vehicle in front
    desired_time_gap = IDM_TIME_GAP_FRONT_VEHICLE + timegap_sample  # [s]

    # IDM Equations
    a_free_flow = 1 - (v_self / V_DESIRED) ** DELTA
    a_interaction = -((S_0 + v_self * desired_time_gap + (v_self * (v_self - v_other))
                       / (2 * math.sqrt(A_MAX * B_COMFORT))) / distance) ** 2
    chosen_acceleration = A_MAX * (a_free_flow + a_interaction)

    # p_t+1 = p_t + v_t * dt + a * dt * dt
    # v_t+1 = v_t + a * dt
    # a_t+1 = sampled deceleration

    if last[2] <= 0:
        stopped = last
        stopped[2] = 0.0  # Zero Velocity
        stopped[4] = 0.0  # Zero Acceleration
        return stopped

    # Assuring that the position does not get smaller (driving backwards)
    # and that the velocity does not get negative
    # Can happen one time step before the vehicle is detected as stopped with the if clause above
    new_v_x = last[2] + chosen_acceleration * time_inc
    new_v_x = 0.0 if new_v_x < 0.0 else new_v_x
    new_pos_x = last[0] + last[2] * time_inc + chosen_acceleration * time_inc * time_inc + pos_uncertainty
    new_pos_x = last[0] if new_pos_x < last[0] else new_pos_x

    return np.array([new_pos_x,
                     last[1],
                     new_v_x,
                     last[3],
                     chosen_acceleration,
                     last[5]])
