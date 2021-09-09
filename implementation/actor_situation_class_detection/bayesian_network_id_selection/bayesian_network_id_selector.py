#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
from actor_situation_class_detection.situation_class import SituationClassType
from actor_situation_class_detection.bayesian_network_id_selection.vehicle_dependent_bn_id import VehicleDependentBNId
from actor_situation_class_detection.bayesian_network_id_selection.two_lane_following_bn_id_selector import \
    TwoLaneFollowingBNIdSelector
from actor_situation_class_detection.bayesian_network_id_selection.bayesian_network_id import BayesianNetId

from actor_situation_class_detection.situation_class import SituationClass, TwoLaneFollowingSituationClass

from typing import List, TYPE_CHECKING

if TYPE_CHECKING:
    from data_model.vehicle import EgoVehicle, OtherVehicle
    from data_model.map import Map
    from simulators.simulator_controller import SimulatorController

class BayesianNetworkIdSelector(object):
    def __init__(self,
                 hero_vehicle: "EgoVehicle",
                 hero_situation_class: SituationClass,
                 other_vehicles: List["OtherVehicle"],
                 map: "Map",
                 simulator_controller: "SimulatorController"):
        self.hero_vehicle: "EgoVehicle" = hero_vehicle
        self.hero_situation_class = hero_situation_class
        self.other_vehicles: List["OtherVehicle"] = other_vehicles

        self.map: "Map" = map
        self._simulator_controller = simulator_controller

    def get_vehicle_dependent_bn_ids(self) -> List["VehicleDependentBNId"]:
        """
        This method first checks the current situation class the hero vehicle is in.
        Dependent on the active situation class, a specific bayesian network id selector object is created for obtaining
        the corresponding bayesian network ids for each vehicle.
        :return: List of VehicleDependentBNId objects
        """

        vehicle_dependent_bn_ids = []

        if self.hero_situation_class is None:
            return vehicle_dependent_bn_ids

        hero_bn_id = self._get_bayesian_network_id_for_hero()
        hero_vehicle_bn_id = VehicleDependentBNId(self.hero_vehicle, hero_bn_id)
        vehicle_dependent_bn_ids.append(hero_vehicle_bn_id)

        if isinstance(self.hero_situation_class, TwoLaneFollowingSituationClass):
            two_lane_foll_bn_id_selector: TwoLaneFollowingBNIdSelector = TwoLaneFollowingBNIdSelector(
                self.hero_situation_class,
                self.hero_vehicle,
                self.other_vehicles,
                self.map,
                self._simulator_controller)

            vehicle_dependent_bn_ids.extend(
                two_lane_foll_bn_id_selector.get_vehicle_dependent_bn_ids_for_two_lane_following_sit_class()
            )

        return vehicle_dependent_bn_ids

    def _get_bayesian_network_id_for_hero(self) -> BayesianNetId:
        if self.hero_situation_class.situation_type == SituationClassType.FOLLOWING_LANE_2_LANES:
            return BayesianNetId.TWO_LANE_FOLLOWING_EGO
