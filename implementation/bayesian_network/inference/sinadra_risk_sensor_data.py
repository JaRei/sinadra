#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
#####
# For each Bayesian network there must be a subclass of
# SINADRARiskSensorData that includes all required data for this
# specific BN.
# The specific BN data class will be the data interface for the
# corresponding BN config class (allowing setting dynamic evidences
# based on this data).
#
# Naming Convention:
# The data class shall be named the same as the Bayesian network.
# (= name of the folder with the Bayesian network's files)
# In case of leading numbers (not allowed in Python), the prefix "Data"
# is added.
# Also, remove any spaces for the Python data class name.
#
# In addition, for each data class there is a specific collect data
# method which needs to be implemented using CARLA data.
# Other collect data methods for other simulators can be added as well.
#####
from typing import TYPE_CHECKING, Tuple, Optional, List, Union
import math

if TYPE_CHECKING:
    from bayesian_network.inference.interfaces import BayesianNetworkInputFeatureData
    from data_model.vehicle import EgoVehicle, OtherVehicle
    from data_model.map import Map


class SINADRARiskSensorData:
    """Input feature data class for Bayesian networks. Each Bayesian network shall have its own custom implementation
    with its custom input features for situation-specific runtime updates.
    """

    def collect_data_from_carla(self, input_feature_data: "BayesianNetworkInputFeatureData",
                                all_vehicles: List[Union["EgoVehicle", "OtherVehicle"]]) -> None:
        """Method that extracts the required input feature from the CARLA simulator.

        Parameters
        ----------
        input_feature_data : BayesianNetworkInputFeatureData
            CARLA specific input comprising the CARLA world instance and the CARLA vehicle instance for the given
            vehicle.
        all_vehicles : List[Union["EgoVehicle", "OtherVehicle"]]
            List containing all vehicles currently active

        Raises
        -------
        NotImplementedError
            Interface base class, thus, this method must be implemented by the child classes.
        """
        raise NotImplementedError


class Data1LaneFollow_Simple(SINADRARiskSensorData):
    def __init__(self):
        self.is_raining_heavy: bool = None
        self.leading_vehicle_is_larger: bool = None
        self.speed: float = None
        self.lead_speed: float = None
        self.distance_between_vehicles: float = None
        self.lead_vehicle_exists: bool = None

    def collect_data_from_carla(self, input_feature_data: "BayesianNetworkInputFeatureData",
                                all_vehicles: List[Union["EgoVehicle", "OtherVehicle"]]):
        environment = input_feature_data.environment
        vehicle = input_feature_data.vehicle

        weather = environment.weather
        self.is_raining_heavy = (weather.precipitation.intensity > 80.0)
        self.leading_vehicle_is_larger = False
        self.speed = vehicle.kinematics.speed
        leading_carla_vehicle = self._extract_leading_vehicle(all_vehicles, vehicle.id)

        self.lead_vehicle_exists = True if leading_carla_vehicle else False

        if leading_carla_vehicle:
            lead_velocity = leading_carla_vehicle.get_velocity()
            self.lead_speed = math.hypot(lead_velocity.x, lead_velocity.y)
        else:
            self.lead_speed = None

        position_center = vehicle.get_location()

        if leading_carla_vehicle:
            distance = leading_carla_vehicle.get_location().distance(position_center)
            # For the distance beween vehicles (front end to rear end), we have to subtract half of the vehicle lengths
            # the bounding box extent value from the carla vehicle represents half of the vehicle length
            self.distance_between_vehicles = distance - leading_carla_vehicle.length / 2 - vehicle.length / 2
        else:
            self.distance_between_vehicles = None

    @staticmethod
    def _extract_leading_vehicle(all_vehicles: List[Union["EgoVehicle", "OtherVehicle"]], id_vehicle_in_focus):
        # TODO: Make more generic

        adversary_vehicle_one = None
        adversary_vehicle_two = None

        for vehicle in all_vehicles:
            if (vehicle.role_name == "adversary1") or (vehicle.role_name == "adversary"):
                adversary_vehicle_one = vehicle
            elif vehicle.role_name == "adversary2":
                adversary_vehicle_two = vehicle

        if (not adversary_vehicle_one) or (not adversary_vehicle_two):
            return None

        # Regarding our lane following location: Check if the two vehicles are on the same lane
        # TODO Coordinate transformation (specific for defined scenarios)
        if 2 >= (math.fabs(adversary_vehicle_one.get_location().x - adversary_vehicle_two.get_location().x)):
            front_vehicle = None
            rear_vehicle = None

            # Regarding our lane following location: Check which vehicles is in front
            # TODO Coordinate transformation (specific for defined scenarios)
            if adversary_vehicle_two.get_location().y >= adversary_vehicle_one.get_location().y:
                front_vehicle, rear_vehicle = adversary_vehicle_one, adversary_vehicle_two
            else:
                front_vehicle, rear_vehicle = adversary_vehicle_two, adversary_vehicle_one

            if id_vehicle_in_focus == rear_vehicle.id:
                return front_vehicle

        return None


class CutInFromLeft(SINADRARiskSensorData):
    def __init__(self):
        self.is_raining_heavy: bool = None
        self.is_indicating_lane_change: bool = None

        # seconds to the lane end
        # -1 equals no lane end detectable
        self.distance_to_lane_end: float = None

        # gap in meter
        # -1 equals only one or no vehicle on the target lane at the location
        self.right_cut_in_gap_size: float = None

        # in degrees based on the lane following steering angle
        self.steering_angle_from_lane_following: float = None
        # distance in meter from the right lane line, based on the nearest part of the vehicle
        self.distance_from_right_lane_line: float = None
        self.distance_from_lane_center: float = None

        # Using the CARLA map object does not work !
        # Reason: Python multiprocessing uses Pickle to serialize objects.
        #         But the carla.libcarla.Map objects does not allow pickling -> A RuntimeError is thrown.
        # storing the CARLA map to be more efficient
        # self.carla_map: "carla.Map" = None

        self.vehicle_speed: float = None
        self.vehicle_length: float = None

    def collect_data_from_carla(self, input_feature_data: "BayesianNetworkInputFeatureData",
                                all_vehicles: List[Union["EgoVehicle", "OtherVehicle"]]):
        environment = input_feature_data.environment
        vehicle = input_feature_data.vehicle
        map = input_feature_data.map

        self.vehicle_speed = vehicle.kinematics.speed
        self.vehicle_length = vehicle.length

        weather = environment.weather
        self.is_raining_heavy = (weather.precipitation.intensity > 80.0)

        self.is_indicating_lane_change = vehicle.light_state.right_blinker
        print("Is indicating lane change: ", self.is_indicating_lane_change)

        self.distance_to_lane_end = map.get_distance_to_lane_end(vehicle.get_location())

        gap_front_vehicle, gap_rear_vehicle = self._extract_vehicles_to_the_right(all_vehicles)
        # only one vehicle on the right lane (either front or rear)
        if (not gap_front_vehicle) or (not gap_rear_vehicle):
            self.right_cut_in_gap_size = -1
        # vehicle directly besides
        elif gap_front_vehicle == gap_rear_vehicle:
            self.right_cut_in_gap_size = 0
        # two vehicles with gap on the right lane
        else:
            distance = gap_front_vehicle.get_location().distance(gap_rear_vehicle.get_location())
            self.right_cut_in_gap_size = distance

        self._extract_vehicle_angle_deviation_from_lane(vehicle, map)

        self._extract_vehicle_distance_from_lane_center(vehicle, map)

    def _extract_distance_to_lane_end(self, vehicle: Union["EgoVehicle", "OtherVehicle"], map: "Map"):
        waypoint_at_location = map.get_waypoint(vehicle.get_location())
        precision = 1  # in m
        waypoints_to_lane_end = waypoint_at_location.next_until_lane_end(precision)
        self.distance_to_lane_end = len(waypoints_to_lane_end) * precision

    @staticmethod
    def _extract_vehicles_to_the_right(all_vehicles: List[Union["EgoVehicle", "OtherVehicle"]]
                                       ) -> Tuple[Optional[Union["EgoVehicle", "OtherVehicle"]],
                                                  Optional[Union["EgoVehicle", "OtherVehicle"]]]:
        right_leader = None
        right_rear = None

        # TODO: Make more generic
        #  Get Waypoint from the vehicle's location -> get waypoint for the right lane from the vehicle's waypoint
        #  Get certain number / distance of waypoints in back front and in the back of the right lane waypoint
        #  Compare all vehicle positions to the list of waypoints
        #  -> detect if vehicles exist and whether they are in front or in the back of the current right lane location

        adversary2_exists = False

        for vehicle in all_vehicles:
            if vehicle.role_name == "adversary1":
                right_leader = vehicle
            if vehicle.role_name == "adversary2":
                adversary2_exists = True
            elif vehicle.role_name == "hero":
                right_rear = vehicle

        if not adversary2_exists:
            right_leader = None
            return None, right_rear

        # check whether the target gap is between the hero and its lead or in front of the hero's lead vehicle
        position_vehicle = vehicle.get_location().y
        position_right_leader = right_leader.get_location().y
        position_right_rear = right_rear.get_location().y

        if position_vehicle > (position_right_rear - 3):
            return right_rear, None
        elif (position_right_rear + 3) > position_vehicle > (position_right_leader - 3):
            return right_leader, right_rear
        elif (position_right_leader + 3) > position_vehicle:
            return None, right_leader
        else:
            # indicating no gap (e.g. vehicle directly besides)
            # by returning the same vehicle which results in a gap of 0
            return vehicle, vehicle

    def _extract_vehicle_angle_deviation_from_lane(self, vehicle: Union["EgoVehicle", "OtherVehicle"], map: "Map"
                                                   ) -> None:
        waypoint_at_location = map.get_waypoint(vehicle.get_location())
        heading_of_the_lane = waypoint_at_location.get_orientation().heading
        heading_of_the_vehicle = vehicle.get_orientation().heading
        self.steering_angle_from_lane_following = math.fabs((heading_of_the_lane % 360) + (heading_of_the_vehicle % 360))

    def _extract_vehicle_distance_from_lane_center(self, vehicle: Union["EgoVehicle", "OtherVehicle"], map: "Map"
                                                   ) -> None:
        lane_width = map.get_waypoint(vehicle.get_location()).lane_width

        bounding_box = vehicle.bounding_box
        bounding_box_world_vertices = bounding_box.get_world_vertices(vehicle.position.world_position)
        front_mid_right_location = bounding_box_world_vertices[6]

        distance_front_right_to_lane_center = self._distance_to_lane_center(front_mid_right_location, map)

        self.distance_from_right_lane_line = lane_width / 2 - distance_front_right_to_lane_center
        self.distance_from_lane_center = self._distance_to_lane_center(vehicle.get_location(), map)

    @staticmethod
    def _distance_to_lane_center(location: "Location", map: "Map") -> float:
        # Waypoints are located at the center of the lane
        waypoint_for_location = map.get_waypoint(location)
        waypoint_for_location_location = waypoint_for_location.get_location()

        waypoints_x_diff = waypoint_for_location_location.x - location.x
        waypoints_y_diff = waypoint_for_location_location.y - location.y
        distance = math.hypot(waypoints_x_diff, waypoints_y_diff)

        return distance


class CutInFromRight(SINADRARiskSensorData):
    def __init__(self):
        self.is_raining_heavy: bool = None
        self.is_indicating_lane_change: bool = None

        # seconds to the lane end
        # -1 equals no lane end detectable
        self.distance_to_lane_end: float = None

        # gap in meter
        # -1 equals only one or no vehicle on the target lane at the location
        self.left_cut_in_gap_size: float = None

        # in degrees based on the lane following steering angle
        self.steering_angle_from_lane_following: float = None
        # distance in meter from the right lane line, based on the nearest part of the vehicle
        self.distance_from_left_lane_line: float = None
        self.distance_from_lane_center: float = None

        # Using the CARLA map object does not work !
        # Reason: Python multiprocessing uses Pickle to serialize objects.
        #         But the carla.libcarla.Map objects does not allow pickling -> A RuntimeError is thrown.
        # storing the CARLA map to be more efficient
        # self.carla_map: "carla.Map" = None

        self.vehicle_speed: float = None
        self.vehicle_length: float = None

    def collect_data_from_carla(self, input_feature_data: "BayesianNetworkInputFeatureData",
                                all_vehicles: List[Union["EgoVehicle", "OtherVehicle"]]):
        environment = input_feature_data.environment
        vehicle = input_feature_data.vehicle
        map = input_feature_data.map

        self.vehicle_speed = vehicle.kinematics.speed
        self.vehicle_length = vehicle.length

        weather = environment.weather
        self.is_raining_heavy = (weather.precipitation.intensity > 80.0)

        self.is_indicating_lane_change = vehicle.light_state.left_blinker
        print("Is indicating lane change: ", self.is_indicating_lane_change)

        self.distance_to_lane_end = map.get_distance_to_lane_end(vehicle.get_location())

        gap_front_vehicle, gap_rear_vehicle = self._extract_vehicles_to_the_right(all_vehicles)
        # only one vehicle on the left lane (either front or rear)
        if (not gap_front_vehicle) or (not gap_rear_vehicle):
            self.left_cut_in_gap_size = -1
        # vehicle directly besides
        elif gap_front_vehicle == gap_rear_vehicle:
            self.left_cut_in_gap_size = 0
        # two vehicles with gap on the left lane
        else:
            distance = gap_front_vehicle.get_location().distance(gap_rear_vehicle.get_location())
            self.left_cut_in_gap_size = distance

        self._extract_vehicle_angle_deviation_from_lane(vehicle, map)

        self._extract_vehicle_distance_from_lane_center(vehicle, map)

    def _extract_distance_to_lane_end(self, vehicle: Union["EgoVehicle", "OtherVehicle"], map: "Map"):
        waypoint_at_location = map.get_waypoint(vehicle.get_location())
        precision = 1  # in m
        waypoints_to_lane_end = waypoint_at_location.next_until_lane_end(precision)
        self.distance_to_lane_end = len(waypoints_to_lane_end) * precision

    @staticmethod
    def _extract_vehicles_to_the_right(all_vehicles: List[Union["EgoVehicle", "OtherVehicle"]]
                                       ) -> Tuple[Optional[Union["EgoVehicle", "OtherVehicle"]],
                                                  Optional[Union["EgoVehicle", "OtherVehicle"]]]:
        left_leader = None
        left_rear = None

        # TODO: Make more generic
        #  Get Waypoint from the vehicle's location -> get waypoint for the right lane from the vehicle's waypoint
        #  Get certain number / distance of waypoints in back front and in the back of the right lane waypoint
        #  Compare all vehicle positions to the list of waypoints
        #  -> detect if vehicles exist and whether they are in front or in the back of the current right lane location

        adversary2_exists = False

        for vehicle in all_vehicles:
            if vehicle.role_name == "adversary1":
                left_leader = vehicle
            if vehicle.role_name == "adversary2":
                adversary2_exists = True
            elif vehicle.role_name == "hero":
                left_rear = vehicle

        if not adversary2_exists:
            left_leader = None
            return None, left_rear

        # check whether the target gap is between the hero and its lead or in front of the hero's lead vehicle
        position_vehicle = vehicle.get_location().y
        position_left_leader = left_leader.get_location().y
        position_left_rear = left_rear.get_location().y

        if position_vehicle > (position_left_rear - 3):
            return left_rear, None
        elif (position_left_rear + 3) > position_vehicle > (position_left_leader - 3):
            return left_leader, left_rear
        elif (position_left_leader + 3) > position_vehicle:
            return None, left_leader
        else:
            # indicating no gap (e.g. vehicle directly besides)
            # by returning the same vehicle which results in a gap of 0
            return vehicle, vehicle

    def _extract_vehicle_angle_deviation_from_lane(self, vehicle: Union["EgoVehicle", "OtherVehicle"], map: "Map"
                                                   ) -> None:
        waypoint_at_location = map.get_waypoint(vehicle.get_location())
        heading_of_the_lane = waypoint_at_location.get_orientation().heading
        heading_of_the_vehicle = vehicle.get_orientation().heading
        self.steering_angle_from_lane_following = math.fabs((heading_of_the_lane % 360) + (heading_of_the_vehicle % 360))

    def _extract_vehicle_distance_from_lane_center(self, vehicle: Union["EgoVehicle", "OtherVehicle"], map: "Map"
                                                   ) -> None:
        lane_width = map.get_waypoint(vehicle.get_location()).lane_width

        bounding_box = vehicle.bounding_box
        bounding_box_world_vertices = bounding_box.get_world_vertices(vehicle.position.world_position)
        front_mid_left_location = bounding_box_world_vertices[4]

        distance_mid_left_to_lane_center = self._distance_to_lane_center(front_mid_left_location, map)

        self.distance_from_left_lane_line = lane_width / 2 - distance_mid_left_to_lane_center
        self.distance_from_lane_center = self._distance_to_lane_center(vehicle.get_location(), map)

    @staticmethod
    def _distance_to_lane_center(location: "Location", map: "Map") -> float:
        # Waypoints are located at the center of the lane
        waypoint_for_location = map.get_waypoint(location)
        waypoint_for_location_location = waypoint_for_location.get_location()

        waypoints_x_diff = waypoint_for_location_location.x - location.x
        waypoints_y_diff = waypoint_for_location_location.y - location.y
        distance = math.hypot(waypoints_x_diff, waypoints_y_diff)

        return distance
