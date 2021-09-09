#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################

import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D
from matplotlib import rc
import pickle

# TODO: Import the parameters as soon as the configuration script does not depend on CARLA anymore
# Difficult because carla is imported in the configuration script
# from sinadra_configuration_parameters import PREDICTION_HORIZON, PREDICTION_TIMESTEP


PREDICTION_HORIZON = 4  # [seconds]: Future time, until which the risk computation is performed
PREDICTION_TIMESTEP = 0.2  # [seconds]


def save_and_plot_vehicle_trajectories_with_distribution(ego_trajectory, front_idm_trajectory,
                                                         front_emergency_trajectory, front_target_brake_trajectory,
                                                         plot_file_path):
    ego_x_pos, ego_x_std, _, _ = ego_trajectory
    front_idm_x_pos, front_idm_x_std, _, _ = front_idm_trajectory
    front_emergency_x_pos, front_emergency_x_std, _, _ = front_emergency_trajectory
    front_target_brake_x_pos, front_target_brake_x_std, _, _ = front_target_brake_trajectory
    jump = int(1 / PREDICTION_TIMESTEP)

    x_times_sigma = 3
    ego_x_pos = ego_x_pos[0::jump]
    ego_x_std = ego_x_std[0::jump]
    ego_x_std = [x_times_sigma * x for x in ego_x_std]
    ego_y = ["Ego"] * len(ego_x_pos)

    front_target_brake_x_pos = front_target_brake_x_pos[0::jump]
    front_target_brake_x_std = front_target_brake_x_std[0::jump]
    front_target_brake_x_std = [x_times_sigma * x for x in front_target_brake_x_std]
    front_target_brake_y = ["FTB"] * len(front_target_brake_x_pos)

    front_idm_x_pos = front_idm_x_pos[0::jump]
    front_idm_x_std = front_idm_x_std[0::jump]
    front_idm_x_std = [x_times_sigma * x for x in front_idm_x_std]
    front_idm_y = ["FIDM"] * len(front_idm_x_pos)

    front_emergency_x_pos = front_emergency_x_pos[0::jump]
    front_emergency_x_std = front_emergency_x_std[0::jump]
    front_emergency_x_std = [x_times_sigma * x for x in front_emergency_x_std]
    front_emergency_y = ["FEB"] * len(front_emergency_x_pos)

    #####################
    # Plot Trajectories #
    #####################

    rc('text', usetex=True)  # activate latex text rendering
    font_size = 10
    rc('font', size=font_size)  # controls default text sizes

    dpi = 300
    fig, ax = plt.subplots(figsize=(2200/dpi, 800/dpi), dpi=dpi)
    fig.suptitle(f'Trajectory Distribution for Brake Behavior Intents')

    translation_factor = -0.15
    translation_offset = -2 * translation_factor

    for i in range(len(ego_x_pos)):
        trans1 = Affine2D().translate(0, i * translation_factor + translation_offset) + ax.transData

        # Add legend labels once
        if i == 0:
            ax.errorbar([front_emergency_x_pos[i]], [front_emergency_y[i]], xerr=[front_emergency_x_std[i]], capsize=2,
                        elinewidth=1, linestyle="None", marker="o", markersize=3, color="red", transform=trans1,
                        label=r"\textbf{F}ront \textbf{E}mergency \textbf{B}rake")
            ax.errorbar([front_idm_x_pos[i]], [front_idm_y[i]], xerr=[front_idm_x_std[i]], capsize=2, elinewidth=1,
                        linestyle="None", marker="v", markersize=3.5, color="red", transform=trans1,
                        label=r"\textbf{F}ront \textbf{IDM}")
            ax.errorbar([front_target_brake_x_pos[i]], [front_target_brake_y[i]], xerr=[front_target_brake_x_std[i]],
                        capsize=2, elinewidth=1, linestyle="None", marker="^", markersize=3.5, color="red",
                        transform=trans1, label=r"\textbf{F}ront \textbf{T}arget \textbf{B}rake")

        else:
            ax.errorbar([front_emergency_x_pos[i]], [front_emergency_y[i]], xerr=[front_emergency_x_std[i]], capsize=2,
                        elinewidth=1, linestyle="None", marker="o", markersize=3, color="red", transform=trans1)
            ax.errorbar([front_idm_x_pos[i]], [front_idm_y[i]], xerr=[front_idm_x_std[i]], capsize=2, elinewidth=1,
                        linestyle="None", marker="v", markersize=3.5, color="red", transform=trans1)
            ax.errorbar([front_target_brake_x_pos[i]], [front_target_brake_y[i]], xerr=[front_target_brake_x_std[i]],
                        capsize=2, elinewidth=1, linestyle="None", marker="^", markersize=3.5, color="red",
                        transform=trans1)

        ax.errorbar([ego_x_pos[i]], [ego_y[i]], xerr=[ego_x_std[i]], capsize=2, elinewidth=1, linestyle="None",
                    marker=".", color="blue", transform=trans1)

    plt.xlim(-5, 60)

    plt.xlabel("Distance from Ego [m]")
    t_labels = ["t = 0", "t = 1", "t = 2", "t = 3", "t = 4"]
    t_label_positions = list(zip(ego_x_pos, ego_y))
    for t_label, t_label_position in list(zip(t_labels, t_label_positions)):
        ax.annotate(text=t_label, xy=t_label_position, xytext=(-46, 80),  textcoords="offset pixels")

    zipped_trajectories = zip(
        ego_x_pos, front_idm_x_pos, front_emergency_x_pos, front_target_brake_x_pos,
        ego_y, front_idm_y, front_emergency_y, front_target_brake_y
      )
    zipped_trajectories_list = list(zipped_trajectories)
    for e_x, f_idm_x, f_emergency_x, f_target_brake_x, e_y, f_idm_y, f_emergency_y, f_target_brake_y in (
            zipped_trajectories_list):
        trans1 = Affine2D().translate(0, ego_x_pos.index(e_x) * translation_factor + translation_offset) + ax.transData
        # vertical connections (time steps between different trajectories)
        ax.plot([f_target_brake_x, f_idm_x], [f_target_brake_y, f_idm_y], linewidth=0.3, color="black", alpha=0.5,
                transform=trans1)
        ax.plot([f_idm_x, f_emergency_x], [f_idm_y, f_emergency_y], linewidth=0.3, color="black", alpha=0.5,
                transform=trans1)
        ax.plot([e_x, f_target_brake_x], [e_y, f_target_brake_y], linewidth=0.3, color="black", alpha=0.5,
                 transform=trans1)

    plt.tight_layout()
    # For ordering the legend
    handles, labels = plt.gca().get_legend_handles_labels()
    order = [2, 1, 0]
    plt.legend([handles[idx] for idx in order], [labels[idx] for idx in order])

    resolution_factor = 1
    plt.savefig(plot_file_path, dpi=dpi*resolution_factor)

    # plt.show()

    plt.close(fig)


def save_and_plot_vehicle_trajectories_with_distribution_single(ego_trajectory, other_trajectory, plot_file_path,
                                                                other_label):
    ego_x_pos, ego_x_std, _, _ = ego_trajectory
    other_x_pos, other_x_std, _, _ = other_trajectory
    jump = int(1 / PREDICTION_TIMESTEP)

    x_times_sigma = 3

    ego_x_pos = ego_x_pos[0::jump]
    ego_x_std = ego_x_std[0::jump]
    ego_x_std = [x_times_sigma * x for x in ego_x_std]
    ego_y = ["Ego"] * len(ego_x_pos)

    other_x_pos = other_x_pos[0::jump]
    other_x_std = other_x_std[0::jump]
    other_x_std = [x_times_sigma * x for x in other_x_std]
    other_y = [other_label] * len(other_x_pos)

    #####################
    # Plot Trajectories #
    #####################

    dpi = 300
    fig, ax = plt.subplots(figsize=(3000/dpi, 500/dpi), dpi=dpi)
    translation_factor = -0.1
    translation_offset = -2 * translation_factor

    for i in range(len(ego_x_pos)):
        trans1 = Affine2D().translate(0, i * translation_factor + translation_offset) + ax.transData
        ax.errorbar([other_x_pos[i]], [other_y[i]], xerr=[other_x_std[i]], capsize=2,
                    elinewidth=1, linestyle="None", marker=".", color="red", transform=trans1)
        ax.errorbar([ego_x_pos[i]], [ego_y[i]], xerr=[ego_x_std[i]], capsize=2, elinewidth=1, linestyle="None",
                    marker=".", color="blue", transform=trans1)

    plt.xlim(-5, 60)

    plt.xlabel("Distance from Ego in m")
    t_labels = ["t = 0", "t = 1", "t = 2", "t = 3", "t = 4"]
    t_label_positions = list(zip(ego_x_pos, ego_y))
    for t_label, t_label_position in list(zip(t_labels, t_label_positions)):
        ax.annotate(text=t_label, xy=t_label_position, xytext=(-46, 100),  textcoords="offset pixels")

    zipped_trajectories = zip(ego_x_pos, other_x_pos, ego_y, other_y)
    for e_x, o_x, e_y, o_y in list(zipped_trajectories):
        trans1 = Affine2D().translate(0, ego_x_pos.index(e_x) * translation_factor + translation_offset) + ax.transData
        ax.plot([e_x, o_x], [e_y, o_y], linewidth=0.3, color="black", alpha=0.5, transform=trans1)

    plt.tight_layout()

    resolution_factor = 1
    plt.savefig(plot_file_path, dpi=dpi*resolution_factor)

    # plt.show()

    plt.close(fig)


if __name__ == '__main__':
    ##########################
    # Load Trajectories File #
    ##########################

    stored_data_directory = "./stored_data/"
    scenario_data_directory = "LF3_50_trajectories/"
    start_cycle_counter = 161  # counter of first saved file
    file_cycle_counter = 160  # "scenario tick"
    # file_cycle_counter = 248  # "scenario tick"
    file_cycle_counter += start_cycle_counter  # corresponding file name counter
    file_postfix = "_trajectories.pickle"
    plot_postfix = "_trajectories_plot.png"

    pickle_file_path = f"{stored_data_directory}{scenario_data_directory}{file_cycle_counter}{file_postfix}"
    plot_file_path = f"{stored_data_directory}{scenario_data_directory}{file_cycle_counter}{plot_postfix}"

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

    save_and_plot_vehicle_trajectories_with_distribution(ego_trajectory, front_idm_trajectory,
                                                         front_emergency_trajectory, front_target_brake_trajectory,
                                                         plot_file_path)
