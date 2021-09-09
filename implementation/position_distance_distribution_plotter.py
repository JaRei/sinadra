#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################

from matplotlib import pyplot as plt
from matplotlib import rc
from scipy import stats
import numpy as np
import math
import pickle

# TODO: Import the parameters as soon as the configuration script does not depend on CARLA anymore
# from sinadra_configuration_parameters import PREDICTION_TIMESTEP, PREDICTION_HORIZON


PREDICTION_HORIZON = 4  # [seconds]: Future time, until which the risk computation is performed
PREDICTION_TIMESTEP = 0.2  # [seconds]


# trajectory: (x_mean, x_std, y_mean, y_std)
def position_distance_distribution_plot_generating_and_saving(ego_trajectory, other_trajectory, save_path, label,
                                                              time_step_dist=1.0):
    font_size = 16
    rc('font', size=font_size)  # controls default text sizes

    fig, (ego_pos, other_pos, distance_dist) = plt.subplots(3, dpi=300)
    fig.set_size_inches((8, 24))
    # if label == "Emergency Brake":
    #     fig.suptitle(f'b4) Distance Distribution: {label}')
    # elif label == "Target Brake":
    #     fig.suptitle(f'b2) Distance Distribution: {label}')
    # elif label == "IDM":
    #     fig.suptitle(f'b3) Distance Distribution: {label}')
    fig.suptitle(f'Distance Distribution: {label}')

    # Get ranges for distribution plots
    p_lower = 0
    p_upper = max(math.ceil(ego_trajectory[0][-1]), math.ceil(other_trajectory[0][-1]))
    p_lower -= 5
    p_upper += 10

    # Ego Position Distribution
    x = np.linspace(p_lower, p_upper, num=(p_upper - p_lower) * 6)
    for t in np.arange(0, PREDICTION_HORIZON + time_step_dist, time_step_dist):
        trajectory_index = int(t / PREDICTION_TIMESTEP)
        p_x = stats.norm.pdf(x, loc=ego_trajectory[0][trajectory_index],
                                   scale=ego_trajectory[1][trajectory_index])

        ego_pos.plot(x, p_x, label=f"t={t}s")

    ego_pos.set_title('')
    ego_pos.set(xlabel="Predicted Ego Position [m]", ylabel='Probability Density Function')
    ego_pos.legend(loc="best")

    # Other Position Distribution
    for t in np.arange(0, PREDICTION_HORIZON + time_step_dist, time_step_dist):
        trajectory_index = int(t / PREDICTION_TIMESTEP)
        p_x = stats.norm.pdf(x, loc=other_trajectory[0][trajectory_index],
                                   scale=other_trajectory[1][trajectory_index])

        other_pos.plot(x, p_x, label=f"t={t}s")

    other_pos.set_title('')
    other_pos.set(xlabel="Predicted Other Position [m]", ylabel='Probability Density Function')
    other_pos.legend(loc="best")

    # Distance Distribution
    d_lower = -35
    d_upper = 10
    x = np.linspace(d_lower, d_upper, num=(d_upper - d_lower) * 6)
    for t in np.arange(0, PREDICTION_HORIZON + time_step_dist, time_step_dist):
        trajectory_index = int(t / PREDICTION_TIMESTEP)
        mean_d_t = other_trajectory[0][trajectory_index] - ego_trajectory[0][trajectory_index]
        std_d_t = math.sqrt(ego_trajectory[1][trajectory_index] ** 2 + other_trajectory[1][trajectory_index] ** 2)
        p_x = stats.norm.pdf(x, loc=mean_d_t, scale=std_d_t)

        distance_dist.plot(x, p_x, label=f"t={t}s")
        distance_dist.fill_between(x, p_x, where=x <= 0.2, facecolor="red", alpha=0.3)

    distance_dist.axvline(x=0, color='k', linestyle='--', linewidth='1', alpha=0.5)
    distance_dist.set_title('')
    distance_dist.set(xlabel="Predicted Distance Ego-Front Vehicle [m]", ylabel='Probability Density Function')
    distance_dist.legend(loc="best")

    plt.savefig(save_path)
    # plt.show()
    plt.close(fig)


if __name__ == '__main__':
    ##########################
    # Load Trajectories File #
    ##########################

    stored_data_directory = "./stored_data/"
    scenario_data_directory = "LF3_50_trajectories/"
    start_cycle_counter = 169  # counter of first saved file
    # file_cycle_counter = 160  # "scenario tick"
    file_cycle_counter = 248  # "scenario tick"
    file_cycle_counter += start_cycle_counter  # corresponding file name counter
    plot_postfix = "_distance_distribution_plot.png"
    file_postfix = "_trajectories.pickle"

    pickle_file_path = f"{stored_data_directory}{scenario_data_directory}{file_cycle_counter}{file_postfix}"

    trajectories_file = open(pickle_file_path, "rb")
    trajectories = pickle.load(trajectories_file)
    trajectories_file.close()

    ########################
    # Extract Trajectories #
    ########################

    ego_trajectory = trajectories["Ego"]
    front_idm_trajectory = trajectories["FrontIDM"]
    front_emergency_trajectory = trajectories["FrontEmergency"]
    front_target_brake_trajectory = trajectories["FrontTargetBrake"]

    plot_file_path_prefix = f"{stored_data_directory}{scenario_data_directory}{file_cycle_counter}"
    plot_file_path_idm = f"{plot_file_path_prefix}_idm{plot_postfix}"
    plot_file_path_emergency = f"{plot_file_path_prefix}_emergency{plot_postfix}"
    plot_file_path_target_brake = f"{plot_file_path_prefix}_target_brake{plot_postfix}"

    position_distance_distribution_plot_generating_and_saving(ego_trajectory, front_idm_trajectory, plot_file_path_idm, "IDM")
    position_distance_distribution_plot_generating_and_saving(ego_trajectory, front_emergency_trajectory,
                                                              plot_file_path_emergency, "Emergency Brake")
    position_distance_distribution_plot_generating_and_saving(ego_trajectory, front_target_brake_trajectory,
                                                              plot_file_path_target_brake, "Target Brake")
