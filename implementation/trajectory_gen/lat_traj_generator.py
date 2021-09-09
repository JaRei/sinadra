#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
import numpy as np
import math
from sinadra_configuration_parameters import LC_ENDPOINT_VARIATION_STD

##########################################################################################
# Trajectory Distribution Generators (Lateral Behaviors)
##########################################################################################


def gen_lanechange(vehicle_init, lc_target, avg_curve_speed, num_traj, time_horizon, time_inc):
    '''

    :param vehicle_init: Side Vehicle kinematic vector, Pos is center of vehicle in ego frame
    :param lc_target:
    :param avg_curve_speed:
    :param num_traj:
    :param time_horizon:
    :param time_inc:
    :return:
    '''
    # print(f"Time Horizon: {time_horizon}, Time Increment: {time_inc}, Num Traj: {num_traj}")
    # This is the distance accounting for the variation of cut-in distance from in front of other vehicle
    # [1,...,num_traj]
    endpoint_x_variation_samples = np.random.normal(lc_target[0], LC_ENDPOINT_VARIATION_STD, num_traj)
    # print(f'Endpoint variation:{endpoint_x_variation_samples}')

    all_timesteps = int(time_horizon / time_inc + 1)

    pos_x_list = np.zeros(shape=(all_timesteps, num_traj))
    pos_y_list = np.zeros(shape=(all_timesteps, num_traj))

    avg_curve_speed = 0.001 if avg_curve_speed < 0.001 else avg_curve_speed

    for i in range(0, num_traj):
        # To own vehicle coordinate system

        start_point = [vehicle_init[0], vehicle_init[1]]
        mid_x = vehicle_init[0] + (endpoint_x_variation_samples[i] - vehicle_init[0]) * 0.5
        control_point_1 = [mid_x, vehicle_init[1]]
        control_point_2 = [mid_x, lc_target[1]]
        end_point = [endpoint_x_variation_samples[i], lc_target[1]]
        #print(f'Start:{start_point}, C1:{control_point_1}, C2:{control_point_2}, End:{end_point}')

        # Compose control points
        control_points = np.array([start_point, control_point_1, control_point_2, end_point])

        # Generate trajectory functions
        pos_func = generate_bezier(control_points)
        vel_func = generate_bezier(control_points, derivative=1)
        acc_func = generate_bezier(control_points, derivative=2)

        # Calculate length of curve at end-point
        # Use Legendre-Gauss approximation for n = 2, z = 1
        # see https://pomax.github.io/bezierinfo/#arclength
        legendre_gauss = lambda vel_func, params: 0.5 * np.sum([np.linalg.norm(vel_func(x)) for x in params])
        curve_length = legendre_gauss(vel_func, np.array([-0.5 * 0.577 + 0.5, 0.5 * 0.577 + 0.5]))
        # print(f"Curve Length: {curve_length}")

        # Get time, when vehicle reaches endpoint given the initial speed of vehicle
        time_at_endpoint = curve_length / avg_curve_speed
        # print(f"Time at end of bezier: {time_at_endpoint}")

        # Get all sample times, where the position is located on the Bezier curve
        time_space = np.arange(0, time_at_endpoint, time_inc)
        # print(f"Time Space: {time_space}")

        # Get parametric points for samples (the parametric space is [0,1] for Bezier)
        # This means normalizing the time space [0s, time_at_end] to [0,1]
        parametric_space = time_space / time_at_endpoint
        # Assure that there are not too many initial trajectory values
        # in case of very slow longitudinal speeds (near 0)
        # (e.g., in some cases the parametric space had shape (4720,)
        # which then is downsampled to the correct (21,))
        if parametric_space.shape[0] > (all_timesteps - 1):
            parametric_space = parametric_space[::math.ceil(parametric_space.shape[0] / (all_timesteps - 1))]
        parametric_space = np.append(parametric_space, 1.0)
        # print(f"Parametric Space: {parametric_space}")

        positions = pos_func(parametric_space)
        velocities = vel_func(parametric_space)

        # Normalize velocities to match initial speed
        velocities = velocities / (velocities[0][0] / vehicle_init[2])

        # print(positions)

        trajectory = positions[:-1]

        # Add next point after bezier to trajectory (needs to be done manually, because bezier end is not the position
        # at the next time step of the generation
        pos_at_bezier_end = positions[-1]
        v_at_bezier_end = velocities[-1]
        next_t_after_end = time_space[-1] + time_inc
        delta_t = next_t_after_end - time_at_endpoint

        next_pos = np.array([np.array([pos_at_bezier_end[0] + v_at_bezier_end[0] * delta_t, pos_at_bezier_end[1]])])
        trajectory = np.concatenate((trajectory, next_pos))

        # Continue trajectory after bezier until prediction horizon with a constant velocity assumption
        existing_timesteps = int(trajectory.shape[0])
        num_remaining_timesteps = all_timesteps - existing_timesteps
        for j in range(0, num_remaining_timesteps):
            last_pos = trajectory[-1]
            next_pos = np.array([np.array([last_pos[0] + v_at_bezier_end[0] * time_inc, last_pos[1]])])
            trajectory = np.concatenate((trajectory, next_pos))

        # print(trajectory[:, 0].shape)

        pos_x_list[:, i] = trajectory[:, 0]  # Copy x positions for current trajectory in pos list at column i
        pos_y_list[:, i] = trajectory[:, 1]  # Copy y positions for current trajectory in pos list at column i

    # print("lane_change trajectory:")
    # for i in range(num_traj):
    #     print("x:")
    #     print(pos_x_list[:, i].tolist())
    #     print("y:")
    #     print(pos_y_list[:, i].tolist())
    # print("----------")
    pos_mean_x = np.mean(pos_x_list, axis=1)
    pos_mean_y = np.mean(pos_y_list, axis=1)
    # print(f"Position Means: x={pos_mean_x}, y={pos_mean_y}")
    pos_std_x = np.std(pos_x_list, axis=1)
    pos_std_y = np.std(pos_y_list, axis=1)
    # print(f"Position Std: x={pos_std_x}, y={pos_std_y}")
    return pos_mean_x, pos_std_x, pos_mean_y, pos_std_y


def generate_bezier(control_points: np.array, derivative: int = 0):
    # B(3, t) = (1-t)**3 * P0 + 3 * (1-t)**2*t * P1 + 3 * (1-t)*t**2 * P2 + t**3 * P3
    def internal_bezier(t: float):
        mt = 1 - t
        mt2 = mt * mt
        mt3 = mt2 * mt
        t2 = t * t
        t3 = t2 * t

        return np.dot(np.array([mt3, 3 * mt2 * t, 3 * mt * t2, t3]).T, control_points[0:4])

    # B'(3, t) = 3 * (1-t)**2 * (P1 - P0) + 6 * (1-t)*t * (P2 - P1) + 3 * t**2 (P3 - P2)
    def internal_bezier_derivative(t: float):
        mt = 1 - t
        mt2 = mt * mt
        t2 = t * t

        forward_differences = control_points[1:4] - control_points[0:3]

        return np.dot(np.array([3 * mt2, 6 * mt * t, 3 * t2]).T, np.array(forward_differences))

    # B''(3, t) = 6 * (1-t) * (P2 - 2*P1 + P0) + 6 * t * (P3 - 2*P2 + P1)
    def internal_bezier_second_derivative(t: float):
        p = control_points

        return np.dot(np.array([6 * (1 - t), 6 * t]).T, np.array([p[2] - 2 * p[1] + p[0], p[3] - 2 * p[2] + p[1]]))

    switch_derivative = {
        0: internal_bezier,
        1: internal_bezier_derivative,
        2: internal_bezier_second_derivative}

    return switch_derivative[derivative]
