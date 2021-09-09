#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################

from typing import List, Tuple, Union, TYPE_CHECKING
import time
from bayesian_network.inference.file_extraction import BayesianNetworkFileExtractor
from bayesian_network.inference.inference import BayesianNetworkInference
from bayesian_network.inference.interfaces import BayesianNetworkData, BayesianNetworkInputFeatureData
from bayesian_network.inference.risk_sensor_data_collecting import RiskSensorDataBuilder
from sinadra_configuration_parameters import NUM_TRAJECTORIES, PREDICTION_HORIZON, PREDICTION_TIMESTEP
from risk_models.eggert_risk_model import eggert_risk
from trajectory_gen.lat_traj_generator import gen_lanechange
from trajectory_gen.long_traj_generator import gen_emergency_brake, gen_targetbrake, gen_idm


if TYPE_CHECKING:
    from bayesian_network.inference.interfaces import BayesianNetworkOutput
    from data_model.vehicle import EgoVehicle, OtherVehicle
    from data_model.positions import Location
    from data_model.environment import Environment
    from data_model.map import Map


##########################################################################################
# Bayesian Network Evaluation
##########################################################################################


def evaluate_bayesian_networks(bn_inference_multiprocessing_pool,
                               vehicle_dependent_bn_ids: List["VehicleDependentBNId"],
                               environment: "Environment", map: "Map",
                               all_vehicles: Union["EgoVehicle", "OtherVehicle"]) -> List["BayesianNetworkOutput"]:
    bayesian_network_data: List[BayesianNetworkData] = []
    carla_input_feature_data: List[BayesianNetworkInputFeatureData] = []

    for vehicle_dependent_bn_id in vehicle_dependent_bn_ids:
        vehicle: Union["EgoVehicle", "OtherVehicle"] = vehicle_dependent_bn_id.vehicle
        vehicle_id = vehicle.id
        vehicle_location: "Location" = vehicle.get_location()

        specific_bn_data = BayesianNetworkData(vehicle_id, vehicle_dependent_bn_id.bn_id, vehicle_location)
        bayesian_network_data.append(specific_bn_data)

        specific_carla_input_data = BayesianNetworkInputFeatureData(vehicle_id, vehicle)
        carla_input_feature_data.append(specific_carla_input_data)

    carla_input_feature_data[0].set_environment(environment)
    print("map at evaluate bayesian network in sinadra: ", map)
    carla_input_feature_data[0].set_map(map)
    print(carla_input_feature_data[0].map)

    risk_sensor_data_builder = RiskSensorDataBuilder()
    bayesian_network_data = (risk_sensor_data_builder
                             .collect_carla_data_and_build_risk_sensor_data(bayesian_network_data,
                                                                            carla_input_feature_data,
                                                                            all_vehicles))

    bn_file_extractor = BayesianNetworkFileExtractor()
    bayesian_network_data = bn_file_extractor.get_all_network_instances_for_situation(bayesian_network_data)

    bayesian_network_inference = BayesianNetworkInference()
    pool = bn_inference_multiprocessing_pool
    bayesian_network_outputs = bayesian_network_inference.bn_inferences_for_vehicles(bayesian_network_data, pool)

    return bayesian_network_outputs


##########################################################################################
# Trajectory Generation and Eggert Risk Computation for ego-front vehicle behavior pairs
##########################################################################################


def emergency_brake_risk(ego_pos_mean, ego_pos_std, fv_init):
    start = time.time()

    fv_pos_mean, fv_pos_std = gen_emergency_brake(fv_init, NUM_TRAJECTORIES, PREDICTION_HORIZON, PREDICTION_TIMESTEP)
    collision_prob = eggert_risk(ego_pos_mean, ego_pos_std, fv_pos_mean, fv_pos_std, PREDICTION_TIMESTEP)

    end = time.time()
    # print("Emergency Brake Risk Time: " + str(end - start))

    return collision_prob, fv_pos_mean, fv_pos_std


def target_brake_risk(ego_pos_mean, ego_pos_std, fv_init, target_distance, target_safe_distance):
    start = time.time()

    fv_pos_mean, fv_pos_std = gen_targetbrake(fv_init, target_distance, target_safe_distance, NUM_TRAJECTORIES,
                                              PREDICTION_HORIZON, PREDICTION_TIMESTEP)
    collision_prob = eggert_risk(ego_pos_mean, ego_pos_std, fv_pos_mean, fv_pos_std, PREDICTION_TIMESTEP)

    end = time.time()
    # print("Target Brake Risk Time: " + str(end - start))

    return collision_prob, fv_pos_mean, fv_pos_std


def idm_risk(ego_pos_mean, ego_pos_std, fv_init, fv_length, fv_front_init):
    start = time.time()

    fv_pos_mean, fv_pos_std = gen_idm(fv_init, fv_length, fv_front_init, NUM_TRAJECTORIES, PREDICTION_HORIZON,
                                      PREDICTION_TIMESTEP)
    collision_prob = eggert_risk(ego_pos_mean, ego_pos_std, fv_pos_mean, fv_pos_std, PREDICTION_TIMESTEP)

    end = time.time()
    # print("IDM Risk Time:  " + str(end - start))

    return collision_prob, fv_pos_mean, fv_pos_std


def lc_right_risk(ego_pos_x_mean, ego_pos_x_std, ego_pos_y_mean, ego_pos_y_std, sv_init, lc_target):
    return lc_risk(ego_pos_x_mean, ego_pos_x_std, ego_pos_y_mean, ego_pos_y_std, sv_init, lc_target)


def lc_left_risk(ego_pos_x_mean, ego_pos_x_std, ego_pos_y_mean, ego_pos_y_std, sv_init, lc_target):
    return lc_risk(ego_pos_x_mean, ego_pos_x_std, ego_pos_y_mean, ego_pos_y_std, sv_init, lc_target)


def lc_risk(ego_pos_x_mean, ego_pos_x_std, ego_pos_y_mean, ego_pos_y_std, sv_init, lc_target):
    # start = time.time()

    sv_pos_x_mean, sv_pos_x_std, sv_pos_y_mean, sv_pos_y_std = \
        gen_lanechange(sv_init, lc_target, sv_init[2], NUM_TRAJECTORIES, PREDICTION_HORIZON, PREDICTION_TIMESTEP)

    collision_prob_x = eggert_risk(ego_pos_x_mean, ego_pos_x_std, sv_pos_x_mean, sv_pos_x_std, PREDICTION_TIMESTEP)
    collision_prob_y = eggert_risk(ego_pos_y_mean, ego_pos_y_std, sv_pos_y_mean, sv_pos_y_std, PREDICTION_TIMESTEP)
    # end = time.time()
    # print("Lane Change Risk Time:  " + str(end - start))

    return collision_prob_x, collision_prob_y, sv_pos_x_mean, sv_pos_x_std, sv_pos_y_mean, sv_pos_y_std


def long_risk_parallel(parameter_dict):
    ego_pos_mean = parameter_dict['ego_pos_mean']
    ego_pos_std = parameter_dict['ego_pos_std']
    behavior = parameter_dict['behavior']
    if behavior == "idm":
        fv_np_vec = parameter_dict['fv_np_vec']
        fv_front_np_vec = parameter_dict['fv_front_np_vec']

        return idm_risk(ego_pos_mean, ego_pos_std, fv_np_vec, fv_front_np_vec)

    if behavior == "emergency":
        fv_np_vec = parameter_dict['fv_np_vec']
        return emergency_brake_risk(ego_pos_mean, ego_pos_std, fv_np_vec)

    if behavior == "targetbrake":
        fv_np_vec = parameter_dict['fv_np_vec']
        target_distance = parameter_dict['target_distance']
        target_safe_distance = parameter_dict['target_safe_distance']

        return target_brake_risk(ego_pos_mean, ego_pos_std, fv_np_vec, target_distance, target_safe_distance)


##########################################################################################
# Risk weighing
##########################################################################################


def compute_total_risk(brake_behavior_risks: List[Tuple[List[float], float]])-> List[float]:
    total_risk = list()
    if len(brake_behavior_risks) == 0:
        return total_risk

    for i in range(0, len(brake_behavior_risks[0][0])):
        total_risk_val = 0.0
        for (prob_lst, weight) in brake_behavior_risks:
            total_risk_val += prob_lst[i] * weight
        total_risk.append(total_risk_val)

    return total_risk


if __name__ == "__main__":
    # For testing purposes of SINADRA without CARLA client
    pass
