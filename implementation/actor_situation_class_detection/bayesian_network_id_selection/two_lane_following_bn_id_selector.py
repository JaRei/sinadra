#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
from actor_situation_class_detection.bayesian_network_id_selection.bayesian_network_id import BayesianNetId
from actor_situation_class_detection.bayesian_network_id_selection.vehicle_dependent_bn_id import VehicleDependentBNId
from actor_situation_class_detection.situation_class import LaneIdentifier, TwoLaneFollowingSituationClass
from data_model.vehicle import VehiclePointId

from sinadra_configuration_parameters import DEBUG_MODE, ACCEPTABLE_DEVIATING_ANGLE_FOR_FRONT_VEHICLE, \
    ACCEPTABLE_DISTANCE_FROM_LANE_CENTER_FOR_FRONT_VEHICLE, DEBUG_COLOR_SENSING_AREA, \
    DEBUG_COLOR_SENSING_AREA_WITH_SENSED_OBJECT, DEBUG_COLOR_DEBUG_LINE_VEHICLE_REAR_END

from typing import List, Dict, TYPE_CHECKING, Union
from shapely.geometry import Point, Polygon, LineString

import math
import statistics
from data_model.positions import Vector3D

if TYPE_CHECKING:
    from data_model.vehicle import EgoVehicle, OtherVehicle
    from data_model.map import Map
    from simulators.simulator_controller import SimulatorController


class TwoLaneFollowingBNIdSelector(object):
    def __init__(self,
                 hero_situation_class: TwoLaneFollowingSituationClass,
                 hero_vehicle: "EgoVehicle",
                 other_vehicles: List["OtherVehicle"],
                 map: "Map",
                 simulator_controller: "SimulatorController"):

        self._hero_situation_class = hero_situation_class
        self._hero_vehicle = hero_vehicle
        self._other_vehicles = other_vehicles
        self._map = map
        self._simulator_controller = simulator_controller

    def get_vehicle_dependent_bn_ids_for_two_lane_following_sit_class(self) -> List["VehicleDependentBNId"]:
        """This method determines the bn ids for each vehicle within the sensing areas of the hero vehicle.
        Returns a list of VehicleDependentBNId objects, which map the bn ids to the vehicles respectively.

        Returns
        -------
        List[VehicleDependentBNId]
            List of VehicleDependentBNId objects that maps a BN id to a vehicle.
        """
        if DEBUG_MODE:
            self._draw_debug_lines_for_rear_points_of_other_vehicles()

        vehicle_dependent_bn_ids = []

        hero_lane_id = self._hero_situation_class.get_actor_lane(self._hero_vehicle)

        if hero_lane_id == LaneIdentifier.RIGHT_LANE:
            other_lane_id = LaneIdentifier.LEFT_LANE
        elif hero_lane_id == LaneIdentifier.LEFT_LANE:
            other_lane_id = LaneIdentifier.RIGHT_LANE

        # determine bn ids for vehicles in front sensing area
        front_vehicle_dependent_bn_ids = self._get_front_vehicle_dependent_bn_ids(hero_lane_id)
        vehicle_dependent_bn_ids.extend(front_vehicle_dependent_bn_ids)

        # determine bn ids for vehicles in side sensing area
        side_vehicle_dependent_bn_ids = self._get_side_vehicle_dependent_bn_ids(other_lane_id)
        vehicle_dependent_bn_ids.extend(side_vehicle_dependent_bn_ids)

        # Cluster dependent vehicle BN IDs for each vehicle
        vehicle_dependent_bn_ids_clustered_by_vehicle = {}
        for already_determined_vehicle_dependent_bn_id in vehicle_dependent_bn_ids:
            identifier = already_determined_vehicle_dependent_bn_id.vehicle.id
            if identifier not in vehicle_dependent_bn_ids_clustered_by_vehicle:
                vehicle_dependent_bn_ids_clustered_by_vehicle[identifier] = []
            vehicle_dependent_bn_ids_clustered_by_vehicle[identifier].append(already_determined_vehicle_dependent_bn_id)

        # Check for each vehicle whether there are multiple BN IDs for the vehicle
        for vehicle_identifier, bn_ids_for_vehicle in vehicle_dependent_bn_ids_clustered_by_vehicle.items():
            if len(bn_ids_for_vehicle) > 1:
                wrapped_vehicle = bn_ids_for_vehicle[0].vehicle
                raw_bn_ids = []
                for vehicle_dependent_bn_id in bn_ids_for_vehicle:
                    vehicle_dependent_bn_ids.remove(vehicle_dependent_bn_id)
                    if vehicle_dependent_bn_id.bn_id not in raw_bn_ids:
                        raw_bn_ids.append(vehicle_dependent_bn_id.bn_id)
                # Handle cases of multiple BN IDs for a vehicle
                if (BayesianNetId.TWO_LANE_FOLLOWING_FRONT_VEHICLE in raw_bn_ids
                        and (BayesianNetId.TWO_LANE_FOLLOWING_SIDE_VEHICLE_ON_OTHER_LANE_LEFT
                             or BayesianNetId.TWO_LANE_FOLLOWING_SIDE_VEHICLE_ON_OTHER_LANE_RIGHT)):
                    vehicle_dependent_bn_ids.append(
                        VehicleDependentBNId(wrapped_vehicle, BayesianNetId.TWO_LANE_FOLLOWING_FRONT_VEHICLE_IN_LANE_CHANGE)
                    )
                else:
                    vehicle_dependent_bn_ids.append(VehicleDependentBNId(wrapped_vehicle, BayesianNetId.UNKNOWN))

        return vehicle_dependent_bn_ids

    def _get_front_vehicle_dependent_bn_ids(self, hero_lane_id: LaneIdentifier) -> List["VehicleDependentBNId"]:
        front_vehicle_dependent_bn_ids = []

        front_sensing_area, debug_lines = self._hero_vehicle.get_front_sensing_area(self._map)

        if front_sensing_area is None and debug_lines is None:
            return front_vehicle_dependent_bn_ids

        reference_point_ids_by_vehicle_in_sensing_area = self._get_other_vehicles_within_sensing_area(
            front_sensing_area)

        for vehicle, ref_point_ids in reference_point_ids_by_vehicle_in_sensing_area.items():
            bn_id = BayesianNetId.UNKNOWN
            if self._check_if_other_vehicle_follows_lane(hero_lane_id, vehicle):
                bn_id = BayesianNetId.TWO_LANE_FOLLOWING_FRONT_VEHICLE
            front_vehicle_dependent_bn_ids.append(VehicleDependentBNId(vehicle, bn_id))

        if DEBUG_MODE and debug_lines is not None:
            self._draw_debug_lines_for_sensing_area(debug_lines, any(front_vehicle_dependent_bn_ids))

        return front_vehicle_dependent_bn_ids

    def _get_side_vehicle_dependent_bn_ids(self, other_lane_id: LaneIdentifier) -> List["VehicleDependentBNId"]:
        side_vehicle_dependent_bn_ids = []
        if other_lane_id == LaneIdentifier.RIGHT_LANE:
            side_vehicle_point_id = VehiclePointId.CENTER_MID_RIGHT
            bn_id_for_other_lane_vehicle = BayesianNetId.TWO_LANE_FOLLOWING_SIDE_VEHICLE_ON_OTHER_LANE_RIGHT
        elif other_lane_id == LaneIdentifier.LEFT_LANE:
            side_vehicle_point_id = VehiclePointId.CENTER_MID_LEFT
            bn_id_for_other_lane_vehicle = BayesianNetId.TWO_LANE_FOLLOWING_SIDE_VEHICLE_ON_OTHER_LANE_LEFT
        else:
            return side_vehicle_dependent_bn_ids

        side_sensing_area, debug_lines = self._hero_vehicle.get_side_sensing_area(map=self._map,
                                                                                  side_vehicle_point_id=side_vehicle_point_id)

        if side_sensing_area is None and debug_lines is None:
            return side_vehicle_dependent_bn_ids

        reference_point_ids_by_vehicle_in_sensing_area = self._get_other_vehicles_within_sensing_area(side_sensing_area)

        for vehicle, ref_point_ids in reference_point_ids_by_vehicle_in_sensing_area.items():
            bn_id = BayesianNetId.UNKNOWN
            if self._check_if_other_vehicle_follows_lane(other_lane_id, vehicle):
                bn_id = bn_id_for_other_lane_vehicle
            side_vehicle_dependent_bn_ids.append(VehicleDependentBNId(vehicle, bn_id))

        if DEBUG_MODE and debug_lines is not None:
            self._draw_debug_lines_for_sensing_area(debug_lines, any(side_vehicle_dependent_bn_ids))

        return side_vehicle_dependent_bn_ids

    def _get_other_vehicles_within_sensing_area(self,
                                                sensing_area: Polygon
                                                ) -> Dict[int, List["VehiclePointId"]]:

        reference_points_by_vehicle_in_sensing_area = {}

        for vehicle in self._other_vehicles:
            sensed_reference_points = []
            vehicle_in_sensing_area = False
            for ref_point_id, ref_location in vehicle.vehicle_points.items():
                reference_2d_point = Point(ref_location.x, ref_location.y)
                reference_point_in_sensing_area = sensing_area.contains(reference_2d_point)
                if reference_point_in_sensing_area:
                    vehicle_in_sensing_area = True
                    sensed_reference_points.append(ref_point_id)
            if vehicle_in_sensing_area:
                reference_points_by_vehicle_in_sensing_area[vehicle] = sensed_reference_points

        return reference_points_by_vehicle_in_sensing_area

    def _check_if_other_vehicle_follows_lane(self,
                                             expected_lane_id: LaneIdentifier,
                                             other_vehicle: Union["OtherVehicle", "EgoVehicle"]
                                             ) -> bool:
        #########################
        #
        # Check vehicle lane is predicted lane
        #
        #########################

        # 2 checks to assure that the vehicle is on its lane.
        # (prohibits that one vehicle can get side and front vehicle classification simultaneously)
        # -> Currently "disabled" because of the "Front Vehicle in Lane Change" Classification which
        # requires the Side and the Front Vehicle Classification

        # if self._hero_situation_class.get_actor_lane(wrapped_other_vehicle) != expected_lane_id:
        #     return False

        other_vehicle_center_point = other_vehicle.get_location()
        nearest_way_point_to_center_point = self._map.get_waypoint(other_vehicle_center_point)
        nearest_way_point_to_center_point_orientation = nearest_way_point_to_center_point.get_orientation()
        nearest_way_point_to_center_point_location = nearest_way_point_to_center_point.get_location()

        # if self._hero_situation_class.get_lane_id_by_location(
        #         nearest_way_point_to_front_mid_point_location) != expected_lane_id:
        #     return False

        #########################
        #
        # Check if other vehicle's orientation is according to lane orientation:
        # Check if angle between forward vector of way point and front mid point of other vehicle is less than
        # certain threshold.
        # If angle is greater than threshold --> other vehicle is not following lane properly
        #
        #########################

        angle_between_vectors = ((nearest_way_point_to_center_point_orientation.heading % 360)
                                 - (other_vehicle.get_orientation().heading % 360))
        angle_between_vectors = math.fabs(angle_between_vectors)

        forward_vector_orientation_according_to_lane_orientation = \
            angle_between_vectors < ACCEPTABLE_DEVIATING_ANGLE_FOR_FRONT_VEHICLE

        #########################
        #
        # Check if front mid point, rear mid point or center mid point of other vehicle are near the center line of the
        # lane (way point).
        # (Assures that the vehicle is following the lane properly.)
        # For threshold comparison the median value is taken.
        # (Assures that in case of lane changes the correct lane is taken for the comparison.)
        #
        #########################

        other_vehicle_rear_center_point = other_vehicle.vehicle_points[VehiclePointId.FRONT_MID_CENTER]
        nearest_way_point_to_rear_center_point = self._map.get_waypoint(other_vehicle_rear_center_point)
        nearest_way_point_to_rear_center_point_location = nearest_way_point_to_rear_center_point.get_location()

        other_vehicle_front_center_point = other_vehicle.vehicle_points[VehiclePointId.REAR_MID_CENTER]
        nearest_way_point_to_front_center_point = self._map.get_waypoint(other_vehicle_front_center_point)
        nearest_way_point_to_front_center_point_location = nearest_way_point_to_front_center_point.get_location()

        distance_between_center_point_and_lane_center = math.hypot(
            nearest_way_point_to_center_point_location.x - other_vehicle_center_point.x,
            nearest_way_point_to_center_point_location.y - other_vehicle_center_point.y
        )
        distance_between_front_center_point_and_lane_center = math.hypot(
            nearest_way_point_to_front_center_point_location.x - other_vehicle_front_center_point.x,
            nearest_way_point_to_front_center_point_location.y - other_vehicle_front_center_point.y
        )
        distance_between_rear_center_point_and_lane_center = math.hypot(
            nearest_way_point_to_rear_center_point_location.x - other_vehicle_rear_center_point.x,
            nearest_way_point_to_rear_center_point_location.y - other_vehicle_rear_center_point.y
        )

        distance_vehicle_to_lane_center = statistics.median([distance_between_front_center_point_and_lane_center,
                                                             distance_between_center_point_and_lane_center,
                                                             distance_between_rear_center_point_and_lane_center])

        other_vehicle_is_near_lane_center = (
                distance_vehicle_to_lane_center < ACCEPTABLE_DISTANCE_FROM_LANE_CENTER_FOR_FRONT_VEHICLE
        )

        return forward_vector_orientation_according_to_lane_orientation and other_vehicle_is_near_lane_center

    def _draw_debug_lines_for_sensing_area(self, debug_lines: List["LineString"], other_vehicles_inside_area: bool):
        debug_color = DEBUG_COLOR_SENSING_AREA
        if other_vehicles_inside_area:
            debug_color = DEBUG_COLOR_SENSING_AREA_WITH_SENSED_OBJECT

        z_coord_for_debug_lines = self._hero_vehicle.vehicle_points[
                                      VehiclePointId.FRONT_MID_CENTER].z * 0.7  # 70% of mid point's height to lower the frame a bit towards the ground
        for line_string in debug_lines:
            first_point_vec = Vector3D(line_string.coords[0][0], line_string.coords[0][1],
                                             z_coord_for_debug_lines)
            second_point_vec = Vector3D(line_string.coords[1][0], line_string.coords[1][1],
                                              z_coord_for_debug_lines)
            self._simulator_controller.draw_line(first_point_vec, second_point_vec, debug_color)

    def _draw_debug_lines_for_rear_points_of_other_vehicles(self):
        debug_color = DEBUG_COLOR_DEBUG_LINE_VEHICLE_REAR_END
        for vehicle_wrapper in self._other_vehicles:
            vehicle_point_dict = vehicle_wrapper.get_middle_points_dict(vehicle_wrapper.get_world_position())
            rear_mid_center_vector = vehicle_point_dict[VehiclePointId.REAR_MID_CENTER]
            upper_line_string_point_vec = Vector3D(rear_mid_center_vector.x, rear_mid_center_vector.y,
                                                   rear_mid_center_vector.z + 3)
            lower_line_string_point_vec = Vector3D(rear_mid_center_vector.x, rear_mid_center_vector.y,
                                                   rear_mid_center_vector.z - 3)
            self._simulator_controller.draw_line(upper_line_string_point_vec, lower_line_string_point_vec, debug_color)
        pass
