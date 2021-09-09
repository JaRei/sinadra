#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
import multiprocessing
import signal
import cv2
import math
import pickle
import os
import time
import numpy as np
from pygame.time import Clock
from typing import Optional, List, Union, TYPE_CHECKING

from sinadra import evaluate_bayesian_networks, emergency_brake_risk, target_brake_risk, idm_risk, lc_right_risk, \
    lc_left_risk, compute_total_risk
from actor_situation_class_detection.town_data.town03_sinadra_data import SituationClassStateMachineTown03
from actor_situation_class_detection.bayesian_network_id_selection.bayesian_network_id_selector import \
    BayesianNetworkIdSelector
from actor_situation_class_detection.bayesian_network_id_selection.bayesian_network_id import BayesianNetId
from sinadra_configuration_parameters import FRAMERATE, SAVE_EVALUATION_DATA, \
    NUMBER_OF_PROCESSES_FOR_THE_BN_INFERENCE, BehaviorType, BEHAVIOR_TYPE_MAPPING, SKIP_CYCLE_COUNT, \
    PREDICTION_HORIZON, PREDICTION_TIMESTEP, NUM_TRAJECTORIES, BRAKE_TARGET_SAFE_DISTANCE_MARGIN, \
    IDM_TIME_GAP_FRONT_VEHICLE, EGO_POS_LAT_STD, LC_CUTIN_DISTANCE_FROM_EGO
from trajectory_gen.long_traj_generator import gen_constant_accel
from util.kinematic_transform import Pose, transform_actor_kinematics_to_ego_frame, \
    transform_ego_kinematics_to_ego_frame, transform_global_pos_to_ego_frame
from util.risk_plot import RiskPlot
from position_distance_distribution_plotter import position_distance_distribution_plot_generating_and_saving
from simulators.carla_simulator_controller import CarlaSimulatorController
from data_model.positions import Location
# data creation

if TYPE_CHECKING:
    from bayesian_network.inference.interfaces import BayesianNetworkOutput
    from data_model.vehicle import EgoVehicle, OtherVehicle
    from data_model.sinadra_data import SinadraData
    from data_model.map import Map
    from simulators.simulator_controller import SimulatorController
    from actor_situation_class_detection.situation_class_state_machine import SituationClassStateMachine


class SinadraClient:

    def __init__(self, simulator_controller: "SimulatorController"):
        self._cycle_counter: int = 0
        self._synchronization_clock: Optional[Clock] = Clock()

        # Initialize Simulator
        self._simulator_controller: "SimulatorController" = simulator_controller
        self._simulator_controller.connect_simulator()
        self._simulator_controller.setup_simulator()
        self._map: "Map" = self._simulator_controller.get_map()

        # Initialize Situation Class
        self._situation_class_state_machine: "SituationClassStateMachine" = SituationClassStateMachineTown03()

        # Initialize SINADRA Client Window
        self._cv_window_name = "SINADRA Risk Sensor"
        cv2.namedWindow(self._cv_window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._cv_window_name, (800, 1200))
        self._simulator_cv_image: Optional["np.ndarray"] = None
        self._subject_vehicle_cv_image: Optional["np.ndarray"] = None


        # Trajectory & BN storage for saving to disk (evaluation)

        # string vehicle ID -> (list x_mean, list x_std, list y_mean, list y_std)
        self.stored_trajectories = {}
        # string BN ID -> {nodeID: list (stateID, stateProbability)}
        self.stored_bayesian_output = {}

        self._risk_plot: RiskPlot = RiskPlot()

        processes_number_bn_inference = NUMBER_OF_PROCESSES_FOR_THE_BN_INFERENCE
        processes_number_bn_inference = processes_number_bn_inference if processes_number_bn_inference else 2
        self.bn_inference_multiprocessing_pool = multiprocessing.Pool(processes=processes_number_bn_inference,
                                                                      initializer=self._worker_init)

        self._sinadra_execution_loop()

    def _sinadra_execution_loop(self):
        # This while loop is the execution loop of the SINADRA client (!= the execution loop of the simulator)
        # Therefore it only controls the update of the risk plot.
        # Simulator-related work is triggered over the game loop.
        while True:
            self._simulator_controller.run_simulator_game_loop_step()
            self._simulator_cv_image = self._simulator_controller.get_scenario_image()

            if self._cycle_counter % SKIP_CYCLE_COUNT == 0:
                self._run_data_and_risk_update_step()
                self._risk_plot.update_and_draw_risk_plot()
            self._cycle_counter += 1

            if self._simulator_cv_image is not None:
                img_risk = np.fromstring(self._risk_plot.get_rgb_image(), dtype=np.uint8, sep='')
                img_risk = img_risk.reshape(self._risk_plot.get_width_height()[::-1] + (3,))
                img_risk = cv2.resize(img_risk, (800, 600), interpolation=cv2.INTER_AREA)
                scene_image = cv2.resize(self._simulator_cv_image, (800, 600), interpolation=cv2.INTER_AREA)
                merged_cv_image = cv2.vconcat([scene_image, img_risk])

                if SAVE_EVALUATION_DATA and self._cycle_counter % SKIP_CYCLE_COUNT == 1:
                    self.generate_and_save_data(img_risk)

                cv2.imshow(self._cv_window_name, merged_cv_image)

            cv2.waitKey(1)
            self._synchronization_clock.tick(FRAMERATE)

    def _run_data_and_risk_update_step(self):
        sinadra_data = self._simulator_controller.get_sinadra_data()
        if sinadra_data:
            self.risk_computation_loop_step(sinadra_data)
        else:
            print("No hero vehicle detected")

    def generate_and_save_data(self, img_risk):
        # Create directory if it does not exist
        path_to_data = "stored_data/"
        if not os.path.exists(path_to_data):
            os.makedirs(path_to_data)

        # Save simulator views
        cv2.imwrite(f"{path_to_data}{self._cycle_counter}_scenario.png", self._simulator_cv_image)
        cv2.imwrite(f"{path_to_data}{self._cycle_counter}_scenario_hero_view.png", self._subject_vehicle_cv_image)

        # Save risk plot
        # cv2.imwrite(f"{path_to_data}{self.cycle_counter}_risk_plot.png", img_risk)
        self._risk_plot.save_risk_plot_on_disk(path_to_data, f"{self._cycle_counter}_risk_plot.png")

        self.save_raw_trajectories(path_to_data)
        self.generate_and_save_bn_output(path_to_data)
        self.generate_and_save_distance_distribution_and_trajectory_plots(path_to_data)

    def save_raw_trajectories(self, path_to_data):
        with open(f"{path_to_data}{self._cycle_counter}_trajectories.txt", "w") as output:
            for vehicle_id, trajectory in self.stored_trajectories.items():
                output.write(vehicle_id + "\n")
                output.write(str(trajectory) + "\n\n")
        with open(f"{path_to_data}{self._cycle_counter}_trajectories.pickle", "wb") as output:
            pickle.dump(self.stored_trajectories, output)

    def generate_and_save_bn_output(self, path_to_data):
        with open(f"{path_to_data}{self._cycle_counter}_bn_outputs.txt", "w") as output:
            for bn_id, nodes in self.stored_bayesian_output.items():
                output.write(bn_id + "\n")
                for node_id, states in nodes.items():
                    output.write(node_id + "\n")
                    output.write(str(states) + "\n")
        with open(f"{path_to_data}{self._cycle_counter}_bn_outputs.pickle", "wb") as output:
            pickle.dump(self.stored_bayesian_output, output)

    def generate_and_save_distance_distribution_and_trajectory_plots(self, path_to_data):
        # if one Front key is inside the stored trajectories, all Front keys are inside
        if "Ego" in self.stored_trajectories and "FrontEmergency" in self.stored_trajectories:
            position_distance_distribution_plot_generating_and_saving(
                self.stored_trajectories["Ego"], self.stored_trajectories["FrontEmergency"],
                f"{path_to_data}{self._cycle_counter}_emergency_distance_distribution_plot.png", "Emergency Brake"
            )
            position_distance_distribution_plot_generating_and_saving(
                self.stored_trajectories["Ego"], self.stored_trajectories["FrontIDM"],
                f"{path_to_data}{self._cycle_counter}_idm_distance_distribution_plot.png", "IDM"
            )
            position_distance_distribution_plot_generating_and_saving(
                self.stored_trajectories["Ego"], self.stored_trajectories["FrontTargetBrake"],
                f"{path_to_data}{self._cycle_counter}_target_brake_distance_distribution_plot.png", "Target Brake"
            )
            # Requires a working Tex environment on the system -> currently not generated during runtime
            # save_and_plot_vehicle_trajectories_with_distribution(
            #     self.stored_trajectories["Ego"], self.stored_trajectories["FrontIDM"],
            #     self.stored_trajectories["FrontEmergency"], self.stored_trajectories["FrontTargetBrake"],
            #     f"{path_to_data}{self.cycle_counter}__trajectories_plot.png"
            # )
            # Currently not required
            # save_and_plot_vehicle_trajectories_with_distribution_single(
            #     self.stored_trajectories["Ego"], self.stored_trajectories["FrontIDM"],
            #     f"{path_to_data}{self.cycle_counter}__trajectories_plot_front_idm.png", "FrontIDM"
            # )
            # save_and_plot_vehicle_trajectories_with_distribution_single(
            #     self.stored_trajectories["Ego"], self.stored_trajectories["FrontEmergency"],
            #     f"{path_to_data}{self.cycle_counter}__trajectories_plot_front_emergency.png", "FrontEmergency"
            # )
            # save_and_plot_vehicle_trajectories_with_distribution_single(
            #     self.stored_trajectories["Ego"], self.stored_trajectories["FrontTargetBrake"],
            #     f"{path_to_data}{self.cycle_counter}__trajectories_plot_front_target_brake.png", "FrontTargetBrake"
            # )
        if "Ego" in self.stored_trajectories and "RightSideVehicleCutIn" in self.stored_trajectories:
            position_distance_distribution_plot_generating_and_saving(
                self.stored_trajectories["Ego"], self.stored_trajectories["RightSideVehicleCutIn"],
                f"{path_to_data}{self._cycle_counter}_right_side_vehicle_cut_in_distance_distribution_plot.png",
                "Right Side Vehicle Cut In"
            )
        if "Ego" in self.stored_trajectories and "LeftSideVehicleCutIn" in self.stored_trajectories:
            position_distance_distribution_plot_generating_and_saving(
                self.stored_trajectories["Ego"], self.stored_trajectories["LeftSideVehicleCutIn"],
                f"{path_to_data}{self._cycle_counter}_left_side_cut_vehicle_in_distance_distribution_plot.png",
                "Left Side Vehicle Cut In"
            )

    def __del__(self):
        # Release multiprocessing resources
        self.bn_inference_multiprocessing_pool.close()
        del self.bn_inference_multiprocessing_pool

        cv2.destroyAllWindows()

    def _worker_init(self):
        # This method is needed for parallel processing based on pools
        # SIGINT (=CTRL+C) will be handled by SIG_IGN (=Handler that ignores the signal)
        signal.signal(signal.SIGINT, signal.SIG_IGN)

    # =============  Game Loop
    
    def risk_computation_loop_step(self, data: "SinadraData"):
        # reset stored trajectories & BN outputs
        self.stored_trajectories = {}
        self.stored_bayesian_output = {}

        print("====================================================\nStart Tick\n\n")
        print(f"Cycle: {self._cycle_counter}")
        start = time.time()

        all_vehicles: List[Union["EgoVehicle", "OtherVehicle"]] = []
        all_vehicles.append(data.hero_vehicle)
        all_vehicles.extend(data.other_vehicles)

        ####################################
        # Classify actors in situation class
        ####################################

        # remove all actors from situation classes
        self._situation_class_state_machine.clear_actor_associations_from_situation_classes()

        # update situation classes with given vehicles
        self._situation_class_state_machine.update_situation_classes(all_vehicles)

        # retrieve situation class the hero vehicle is currently in
        hero_situation_class = self._situation_class_state_machine.get_hero_situation_class()
        self.log_hero_sit_class_info(data.hero_vehicle, hero_situation_class)

        #########################################################
        # Bayesian Network Inference for actors relevant for ego
        #########################################################

        # based on situation class and other vehicles, determine, which BN shall be evaluated
        bn_selector = BayesianNetworkIdSelector(data.hero_vehicle, hero_situation_class,
                                                data.other_vehicles, self._map, self._simulator_controller)
        vehicle_dependent_bn_ids = bn_selector.get_vehicle_dependent_bn_ids()
        self.log_active_bn_info(vehicle_dependent_bn_ids)

        if not any(vehicle_dependent_bn_ids):
            print("no relevant vehicles, for which risk can be computed")
            return

        # Infer behavior likelihoods in Bayesian Networks
        infered_bn_outputs = evaluate_bayesian_networks(self.bn_inference_multiprocessing_pool,
                                                        vehicle_dependent_bn_ids, data.environment, self._map,
                                                        all_vehicles)
        self.log_bayesian_network_inference_outputs(infered_bn_outputs)

        ##########################################################################################
        # Trajectory sampling for relevant behaviors of other actors and pair-wise risk assessment
        ##########################################################################################

        for network_output in infered_bn_outputs:
            vehicle_exists = self.generate_trajectory_and_compute_risk_for_vehicle_and_bn(data, network_output)
            if not vehicle_exists:
                return

        end = time.time()
        print("\n\nEnd Tick (Exec Time="+str(end-start)+"\n====================================================")

    def generate_trajectory_and_compute_risk_for_vehicle_and_bn(self, data, network_output):
        bn_node_dict = {}
        for node in network_output.output_nodes:
            node_output_list = []
            for outcome in node.outcomes:
                node_output_list.append((outcome.name, outcome.value))
            bn_node_dict[node.title] = node_output_list

            if node.title not in BEHAVIOR_TYPE_MAPPING:
                continue

            behavior_type = BEHAVIOR_TYPE_MAPPING[node.title]

            #############################
            # Ego Trajectory Generation
            #############################

            ego_half_length, ego_np_vec_front, ego_pos_x_mean, ego_pos_x_std = (
                self.generate_ego_vehicle_trajectory(data)
            )

            #############################################
            # Front Vehicle Brake Behavior Prediction
            #############################################
            if behavior_type == BehaviorType.Braking:
                front_vehicle_exists = self.compute_risk_front_vehicle_braking(
                    data, ego_half_length, ego_pos_x_mean, ego_pos_x_std, network_output, node
                )
                if not front_vehicle_exists:
                    return False

            #############################################
            # Side Vehicle Cutin From Right Behavior Prediction
            #############################################
            elif behavior_type == BehaviorType.LaneChangeToLeft:
                right_side_vehicle_exists = self.compute_risk_side_vehicle_cut_in_from_right(
                    data, ego_np_vec_front, ego_pos_x_mean, ego_pos_x_std, network_output, node
                )
                if not right_side_vehicle_exists:
                    return False

            #############################################
            # Side Vehicle Cutin From Left Behavior Prediction
            #############################################
            elif behavior_type == BehaviorType.LaneChangeToRight:
                left_side_vehicle_exists = self.compute_risk_side_vehicle_cut_in_from_left(
                    data, ego_np_vec_front, ego_pos_x_mean, ego_pos_x_std, network_output, node
                )
                if not left_side_vehicle_exists:
                    return False
            else:
                pass
        self.stored_bayesian_output[network_output.bayesian_network_id] = bn_node_dict

        return True

    def generate_ego_vehicle_trajectory(self, data):
        # Generate ego trajectory prediction (Constant Acceleration)
        # Positions of the following are relative to the ego vehicle center point, i.e. pos = (0,0)
        ego_pos_center, ego_speed, ego_accel, ego_np_vec_center = transform_ego_kinematics_to_ego_frame(
            data.hero_vehicle)
        # Add half of the ego vehicle length to move the position to the ego's front end
        # This is relevant to get the "correct" distances for risk analysis
        ego_np_vec_front = ego_np_vec_center
        ego_half_length = data.hero_vehicle.length / 2
        ego_np_vec_front[0] += ego_half_length
        ego_pos_front = ego_pos_center
        ego_pos_front.x += ego_half_length
        ego_pos_x_mean, ego_pos_x_std = gen_constant_accel(ego_np_vec_front, NUM_TRAJECTORIES,
                                                           PREDICTION_HORIZON, PREDICTION_TIMESTEP)
        self.stored_trajectories["Ego"] = (list(ego_pos_x_mean), list(ego_pos_x_std),
                                           [0] * len(ego_pos_x_mean), [0] * len(ego_pos_x_std))
        return ego_half_length, ego_np_vec_front, ego_pos_x_mean, ego_pos_x_std

    def compute_risk_front_vehicle_braking(self, data, ego_half_length, ego_pos_x_mean, ego_pos_x_std, network_output,
                                           node):
        # Relative braking behavior likelihoods
        emergency_prob = node.outcomes[0].value
        targetbrake_prob = node.outcomes[1].value
        followvehicle_prob = node.outcomes[2].value
        nobrake_prob = node.outcomes[3].value
        # Get actor reference of front vehicle
        vehicle_id = network_output.vehicle_id
        front_vehicle: "OtherVehicle" = filter(lambda v: v.id == vehicle_id, data.other_vehicles)
        front_vehicle = next(front_vehicle, None)
        if not front_vehicle:
            return False
        brake_behavior_risks = list()
        # Position of the following call is the position of the front vehicle's center point
        # relative to the ego vehicle's center point
        fv_pos_center, fv_speed, fv_accel, fv_np_vec_center = transform_actor_kinematics_to_ego_frame(data.hero_vehicle,
                                                                                                      front_vehicle)
        # We want to get the x location of the front vehicle's rear end relative to the ego
        # vehicle's front end to get the correct distance. Subtract half of the lengths of
        # both vehicles.
        fv_half_length = front_vehicle.length / 2
        fv_np_vec_rear = fv_np_vec_center
        fv_pos_rear = fv_pos_center
        fv_np_vec_rear[0] -= fv_half_length + ego_half_length
        fv_pos_rear.x -= fv_half_length + ego_half_length
        # Adapt Front Vehicle Position to align with the ego vehicle
        # (if it is executing a lane change)
        if (network_output.vehicle_situation_state_id ==
                BayesianNetId.TWO_LANE_FOLLOWING_FRONT_VEHICLE_IN_LANE_CHANGE):
            fv_pos_rear.y = 0
            fv_speed.lat = 0
            fv_accel.lat = 0
            fv_np_vec_rear[1], fv_np_vec_rear[3], fv_np_vec_rear[5] = 0, 0, 0
        ###########################
        # Emergency Brake Behavior Risk
        ###########################
        if emergency_prob > 0:
            emergency_brake_eggert_prob, fv_x_mean, fv_x_std = emergency_brake_risk(ego_pos_x_mean,
                                                                                    ego_pos_x_std,
                                                                                    fv_np_vec_rear)
            self.stored_trajectories["FrontEmergency"] = (list(fv_x_mean), list(fv_x_std),
                                                          [0] * len(fv_x_mean), [0] * len(fv_x_std))
            brake_behavior_risks.append((emergency_brake_eggert_prob, emergency_prob))
            # Update the dynamic risk plot
            self._risk_plot.set_front_vehicle_emergency_risk(emergency_brake_eggert_prob)
        ###########################
        # Target Brake Behavior Risk
        ###########################
        if targetbrake_prob > 0:
            # This is the global coordinate of the concrete stop line used in the Town03 scenario
            # TODO needs to be made dynamic by using the waypoint API
            stop_line_pos_g: Location = Location(-74.8, 127, -0.1)
            # The following position is relative to the ego vehicle's center point
            stop_line_pos_e: Pose = transform_global_pos_to_ego_frame(data.hero_vehicle, stop_line_pos_g)

            # The target distance should be the distance between front vehicle front end
            # and the stop line
            fv_frontend_x = fv_np_vec_center[0] + fv_half_length
            target_distance = stop_line_pos_e.x - fv_frontend_x

            target_brake_eggert_prob, fv_x_mean, fv_x_std = target_brake_risk(
                ego_pos_x_mean, ego_pos_x_std, fv_np_vec_rear, target_distance,
                BRAKE_TARGET_SAFE_DISTANCE_MARGIN
            )
            self.stored_trajectories["FrontTargetBrake"] = (
                list(fv_x_mean), list(fv_x_std), [0] * len(fv_x_mean), [0] * len(fv_x_std)
            )
            brake_behavior_risks.append((target_brake_eggert_prob, targetbrake_prob))
            # Update the dynamic risk plot
            self._risk_plot.set_front_vehicle_target_brake_risk(target_brake_eggert_prob)
        ###############################################################
        # Intelligent Driver Model Behavior Risk (Follow Vehicle and Follow Road)
        ###############################################################
        if followvehicle_prob > 0 or nobrake_prob > 0:

            # Role = adversary or adversary1 -> Vehicle that spawned directly in front of the ego
            # vehicle (specific for our scenarios)
            # Role = adversary2 -> Vehicle that spawned either in front of the adversary(1) vehicle
            # or on the neighboring lane
            front_role = front_vehicle.role_name

            if (front_role == "adversary1") or (front_role == "adversary"):
                # in the case that the adversary(1) vehicle is directly in front of the ego vehicle
                # the adversary2 vehicle can either be in front of the adversary(1) vehicle or
                # on the neighboring lane
                fv_front = filter(lambda v: v.role_name == "adversary2",
                                  data.other_vehicles)
                front_front_vehicle = next(fv_front, None) if fv_front else None
                if front_front_vehicle:
                    # Check whether the vehicles are on the same lane
                    # (specific for our lane following scenario location)
                    # TODO Coordinate transformation
                    if math.fabs(front_vehicle.get_location().x
                                 - front_front_vehicle.get_location().x) >= 1.5:
                        front_front_vehicle = None
            elif front_role == "adversary2":
                # in the case that the adversary2 vehicle is directly in front of the ego vehicle
                # the adversary1 vehicle is always in front of the adversary2 vehicle
                fv_front = filter(lambda v: v.role_name == "adversary1",
                                  data.other_vehicles)
                front_front_vehicle = next(fv_front, None) if fv_front else None

            if not front_front_vehicle:
                # print("No vehicle in front of the considered front vehicle exists")

                # No vehicle in front of the front vehicle exists in groundtruth.
                # Since the BN predicts a non-zero prob of follow vehicle behavior due to
                # uncertainties, we assume a vehicle to be in front with a time gap of 1,5s
                # and same speed like FV for the trajectory generation.

                # Time gap needs to be determined between front vehicle front end and
                # front front vehicle rear end

                fv_front_pos_center = fv_np_vec_center[0] + fv_np_vec_center[2] * IDM_TIME_GAP_FRONT_VEHICLE
                fv_front_pos_rear_x = fv_front_pos_center - (fv_half_length + ego_half_length)
                fv_front_pos_rear = Pose(x=fv_front_pos_rear_x, y=fv_np_vec_center[1])
                fv_front_speed = fv_speed
                fv_front_np_vec_rear = np.array(
                    [fv_front_pos_rear_x, fv_np_vec_center[1], fv_front_speed.long, fv_front_speed.lat, 0.0, 0.0])
            else:
                # Position of the front front vehicle's center point relative to the ego
                # vehicle's center point
                fv_front_pos_center, fv_front_speed, _, fv_front_np_vec_center = transform_actor_kinematics_to_ego_frame(
                    data.hero_vehicle, front_front_vehicle)

                # What we need for the IDM is the position of front front vehicle's rear end
                # relative to the ego vehicle's front end
                fv_front_pos_rear = fv_front_pos_center
                fv_front_np_vec_rear = fv_front_np_vec_center

                fv_front_half_length = front_front_vehicle.length / 2  # is already half of the vehicle length
                fv_front_np_vec_rear[0] -= fv_front_half_length + ego_half_length
                fv_front_pos_rear.x -= fv_front_half_length + ego_half_length

            # self.log_kinematic_info(ego_pos, ego_speed, fv_front_pos, fv_front_speed, fv_pos, fv_speed)

            idm_eggert_prob, fv_x_mean, fv_x_std = idm_risk(ego_pos_x_mean, ego_pos_x_std,
                                                            fv_np_vec_rear, 2 * fv_half_length,
                                                            fv_front_np_vec_rear)
            self.stored_trajectories["FrontIDM"] = (
                list(fv_x_mean), list(fv_x_std), [0] * len(fv_x_mean), [0] * len(fv_x_std)
            )
            brake_behavior_risks.append((idm_eggert_prob, followvehicle_prob + nobrake_prob))
            # Update the dynamic risk plot
            self._risk_plot.set_front_vehicle_idm_risk(idm_eggert_prob)
        # Weight the individual behavior risk scores based on BN output likelihoods
        weighted_total_risk = compute_total_risk(brake_behavior_risks)
        # Update the dynamic risk plot
        self._risk_plot.set_front_vehicle_cumulative_risk(weighted_total_risk)

        return True

    def compute_risk_side_vehicle_cut_in_from_right(self, data, ego_np_vec_front, ego_pos_x_mean, ego_pos_x_std,
                                                    network_output, node):
        # Relative cutin behavior likelihoods
        cutin_prob = node.outcomes[0].value
        nocutin_prob = 1.0 - cutin_prob
        # Get actor reference of side vehicle
        vehicle_id = network_output.vehicle_id
        side_vehicle: "OtherVehicle" = filter(lambda v: v.id == vehicle_id, data.other_vehicles)
        side_vehicle = next(side_vehicle, None)
        if not side_vehicle:
            return False
        # Note that all computations are performed in the local ego coordinate frame
        # TODO Think about whether to use rear end of side vehicle
        sv_pos, sv_speed, sv_accel, sv_np_vec = transform_actor_kinematics_to_ego_frame(data.hero_vehicle, side_vehicle)
        # End Point of lane change behavior [x,y]
        lc_target = [ego_np_vec_front[0] + LC_CUTIN_DISTANCE_FROM_EGO, ego_np_vec_front[1]]
        # print("Side Vehicle")
        # print(sv_np_vec)
        # print("Ego Vehicle")
        # print(ego_np_vec_front)
        # print("LC Target")
        # print(lc_target)
        # Generate ego lateral positions for a straight drive
        num_positions = int((PREDICTION_HORIZON / PREDICTION_TIMESTEP + 1))
        ego_pos_y_mean = np.zeros(num_positions)
        ego_pos_y_std = np.full(num_positions, EGO_POS_LAT_STD)  # Assumed standard dev of ego lat pos = 0.2
        collision_prob_x, collision_prob_y, sv_x_mean, sv_x_std, sv_y_mean, sv_y_std = (
            lc_left_risk(ego_pos_x_mean, ego_pos_x_std, ego_pos_y_mean, ego_pos_y_std, sv_np_vec,
                         lc_target)
        )
        self.stored_trajectories["RightSideVehicleCutIn"] = (
            list(sv_x_mean), list(sv_x_std), list(sv_y_mean), list(sv_y_std)
        )
        # print("Collision Probability X")
        # print(collision_prob_x)
        # print(collision_prob_y)
        lc_behavior_long_risk = list()
        lc_behavior_lat_risk = list()
        # Assuming that only cut-in of side vehicle into ego lane is critical --> Zero Risk
        # This is important for computing the weighted total risk
        length_sampled_time_horizon = len(np.arange(0, PREDICTION_HORIZON + PREDICTION_TIMESTEP, PREDICTION_TIMESTEP))
        lc_behavior_long_risk.append((list(np.zeros(length_sampled_time_horizon)), nocutin_prob))
        lc_behavior_long_risk.append((collision_prob_x, cutin_prob))
        # Weight the individual behavior risk scores based on BN output likelihoods
        weighted_total_long_risk = compute_total_risk(lc_behavior_long_risk)
        # Assuming that only cut-in of side vehicle into ego lane is critical --> Zero Risk
        # This is important for computing the weighted total risk
        lc_behavior_lat_risk.append((list(np.zeros(length_sampled_time_horizon)), nocutin_prob))
        lc_behavior_lat_risk.append((collision_prob_y, cutin_prob))
        # Weight the individual behavior risk scores based on BN output likelihoods
        weighted_total_lat_risk = compute_total_risk(lc_behavior_lat_risk)
        # Update the dynamic risk plot
        self._risk_plot.set_right_side_vehicle_longitudinal_risk(weighted_total_long_risk)
        self._risk_plot.set_right_side_vehicle_lateral_risk(weighted_total_lat_risk)

        return True

    def compute_risk_side_vehicle_cut_in_from_left(self, data, ego_np_vec_front, ego_pos_x_mean, ego_pos_x_std,
                                                   network_output, node):
        # Relative cutin behavior likelihoods
        cutin_prob = node.outcomes[0].value
        nocutin_prob = 1.0 - cutin_prob
        # Get actor reference of side vehicle
        vehicle_id = network_output.vehicle_id
        side_vehicle: "OtherVehicle" = filter(lambda v: v.id == vehicle_id, data.other_vehicles)
        side_vehicle = next(side_vehicle, None)
        if not side_vehicle:
            return False
        # Note that all computations are performed in the local ego coordinate frame
        # TODO Think about whether to use rear end of side vehicle
        sv_pos, sv_speed, sv_accel, sv_np_vec = transform_actor_kinematics_to_ego_frame(data.hero_vehicle, side_vehicle)
        # End Point of lane change behavior [x,y]
        lc_target = [ego_np_vec_front[0] + LC_CUTIN_DISTANCE_FROM_EGO, ego_np_vec_front[1]]
        # print("Side Vehicle")
        # print(sv_np_vec)
        # print("Ego Vehicle")
        # print(ego_np_vec_front)
        # print("LC Target")
        # print(lc_target)
        # Generate ego lateral positions for a straight drive
        num_positions = int((PREDICTION_HORIZON / PREDICTION_TIMESTEP + 1))
        ego_pos_y_mean = np.zeros(num_positions)
        ego_pos_y_std = np.full(num_positions, EGO_POS_LAT_STD)  # Assumed standard dev of ego lat pos = 0.2
        collision_prob_x, collision_prob_y, sv_x_mean, sv_x_std, sv_y_mean, sv_y_std = (
            lc_right_risk(ego_pos_x_mean, ego_pos_x_std, ego_pos_y_mean, ego_pos_y_std, sv_np_vec,
                          lc_target)
        )
        self.stored_trajectories["LeftSideVehicleCutIn"] = (
            list(sv_x_mean), list(sv_x_std), list(sv_y_mean), list(sv_y_std)
        )
        # print("Collision Probability X")
        # print(collision_prob_x)
        # print(collision_prob_y)
        lc_behavior_long_risk = list()
        lc_behavior_lat_risk = list()
        # Assuming that only cut-in of side vehicle into ego lane is critical --> Zero Risk
        # This is important for computing the weighted total risk
        length_sampled_time_horizon = len(np.arange(0, PREDICTION_HORIZON + PREDICTION_TIMESTEP, PREDICTION_TIMESTEP))
        lc_behavior_long_risk.append((list(np.zeros(length_sampled_time_horizon)), nocutin_prob))
        lc_behavior_long_risk.append((collision_prob_x, cutin_prob))
        # Weight the individual behavior risk scores based on BN output likelihoods
        weighted_total_long_risk = compute_total_risk(lc_behavior_long_risk)
        # Assuming that only cut-in of side vehicle into ego lane is critical --> Zero Risk
        # This is important for computing the weighted total risk
        lc_behavior_lat_risk.append((list(np.zeros(length_sampled_time_horizon)), nocutin_prob))
        lc_behavior_lat_risk.append((collision_prob_y, cutin_prob))
        # Weight the individual behavior risk scores based on BN output likelihoods
        weighted_total_lat_risk = compute_total_risk(lc_behavior_lat_risk)
        # Update the dynamic risk plot
        self._risk_plot.set_left_side_vehicle_longitudinal_risk(weighted_total_long_risk)
        self._risk_plot.set_left_side_vehicle_lateral_risk(weighted_total_lat_risk)

        return True

    ###########
    # Logging
    ###########

    def log_hero_sit_class_info(self, hero_vehicle, hero_situation_class):
        if hero_situation_class is None:
            print("Hero situation class = None")
        else:
            hero_situation_class.print_information(hero_vehicle)


    def log_active_bn_info(self, vehicle_dependent_bn_ids):
        if any(vehicle_dependent_bn_ids):
            print('BN_IDs by vehicle:')
            for vehicle_dependent_bn_id in vehicle_dependent_bn_ids:
                print(f'Vehicle {vehicle_dependent_bn_id.vehicle.role_name} : BN_ID = {vehicle_dependent_bn_id.bn_id}')


    def log_bayesian_network_inference_outputs(self, infered_bn_outputs: List["BayesianNetworkOutput"]) -> None:
        print("\n---------------\nBayesian Network(s) Inference Outputs:\n")
        for bn_output in infered_bn_outputs:
            print(f"Vehicle (ID: {bn_output.vehicle_id}):\n{bn_output}\n")
            print("---------------\n")

    def log_kinematic_info(self, ego_pos, ego_speed, fv_front_pos, fv_front_speed, fv_pos, fv_speed):
        print("Ego Vehicle")
        print(ego_pos)
        print(ego_speed)
        print("Front Vehicle")
        print(fv_pos)
        print(fv_speed)
        print("Front Front Vehicle")
        print(fv_front_pos)
        print(fv_front_speed)


if __name__ == "__main__":
    simulator_controller = CarlaSimulatorController()
    SinadraClient(simulator_controller)
