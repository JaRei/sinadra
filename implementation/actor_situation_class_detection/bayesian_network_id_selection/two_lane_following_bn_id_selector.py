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
from actor_situation_class_detection.vehicle_actor_wrapper import VehicleActorWrapper, VehiclePointId

from typing import List, Dict
from shapely.geometry import Point, Polygon, LineString

import math
import statistics
import carla

from sinadra_configuration_parameters import \
    DEBUG_MODE, \
    ACCEPTABLE_DEVIATING_ANGLE_FOR_FRONT_VEHICLE, \
    ACCEPTABLE_DISTANCE_FROM_LANE_CENTER_FOR_FRONT_VEHICLE, \
    CARLA_DEBUG_LINE_THICKNESS, \
    CARLA_DEBUG_DRAWING_LIFE_TIME, \
    DEBUG_COLOR_SENSING_AREA, \
    DEBUG_COLOR_SENSING_AREA_WITH_SENSED_OBJECT, \
    DEBUG_COLOR_DEBUG_LINE_VEHICLE_REAR_END


class TwoLaneFollowingBNIdSelector(object):
    def __init__(self,
                 hero_situation_class: TwoLaneFollowingSituationClass,
                 wrapped_hero_vehicle: VehicleActorWrapper,
                 wrapped_other_vehicles: List["VehicleActorWrapper"],
                 carla_map: carla.Map,
                 carla_debug_helper: carla.DebugHelper):

        self._hero_situation_class = hero_situation_class
        self._wrapped_hero_vehicle = wrapped_hero_vehicle
        self._wrapped_other_vehicles = wrapped_other_vehicles
        self._carla_map = carla_map
        self._carla_debug_helper = carla_debug_helper

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

        hero_lane_id = self._hero_situation_class.get_actor_lane(self._wrapped_hero_vehicle)

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
            identifier = already_determined_vehicle_dependent_bn_id.wrapped_vehicle.id
            if identifier not in vehicle_dependent_bn_ids_clustered_by_vehicle:
                vehicle_dependent_bn_ids_clustered_by_vehicle[identifier] = []
            vehicle_dependent_bn_ids_clustered_by_vehicle[identifier].append(already_determined_vehicle_dependent_bn_id)

        # Check for each vehicle whether there are multiple BN IDs for the vehicle
        for vehicle_identifier, bn_ids_for_vehicle in vehicle_dependent_bn_ids_clustered_by_vehicle.items():
            if len(bn_ids_for_vehicle) > 1:
                wrapped_vehicle = bn_ids_for_vehicle[0].wrapped_vehicle
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

        front_sensing_area, debug_lines = self._wrapped_hero_vehicle.get_front_sensing_area(self._carla_map)

        if front_sensing_area is None and debug_lines is None:
            return front_vehicle_dependent_bn_ids

        reference_point_ids_by_vehicle_in_sensing_area = self._get_other_vehicles_within_sensing_area(
            front_sensing_area)

        for wrapped_vehicle, ref_point_ids in reference_point_ids_by_vehicle_in_sensing_area.items():
            bn_id = BayesianNetId.UNKNOWN
            if self._check_if_other_vehicle_follows_lane(hero_lane_id, wrapped_vehicle):
                bn_id = BayesianNetId.TWO_LANE_FOLLOWING_FRONT_VEHICLE
            front_vehicle_dependent_bn_ids.append(VehicleDependentBNId(wrapped_vehicle, bn_id))

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

        side_sensing_area, debug_lines = self._wrapped_hero_vehicle.get_side_sensing_area(carla_map=self._carla_map,
                                                                                          side_vehicle_point_id=side_vehicle_point_id)

        if side_sensing_area is None and debug_lines is None:
            return side_vehicle_dependent_bn_ids

        reference_point_ids_by_vehicle_in_sensing_area = self._get_other_vehicles_within_sensing_area(side_sensing_area)

        for wrapped_vehicle, ref_point_ids in reference_point_ids_by_vehicle_in_sensing_area.items():
            bn_id = BayesianNetId.UNKNOWN
            if self._check_if_other_vehicle_follows_lane(other_lane_id, wrapped_vehicle):
                bn_id = bn_id_for_other_lane_vehicle
            side_vehicle_dependent_bn_ids.append(VehicleDependentBNId(wrapped_vehicle, bn_id))

        if DEBUG_MODE and debug_lines is not None:
            self._draw_debug_lines_for_sensing_area(debug_lines, any(side_vehicle_dependent_bn_ids))

        return side_vehicle_dependent_bn_ids

    def _get_other_vehicles_within_sensing_area(self,
                                                sensing_area: Polygon
                                                ) -> Dict["VehicleActorWrapper", List["VehiclePointId"]]:

        reference_points_by_vehicle_in_sensing_area = {}

        for vehicle_wrapper in self._wrapped_other_vehicles:
            sensed_reference_points = []
            vehicle_in_sensing_area = False
            for ref_point_id, ref_location in vehicle_wrapper.vehicle_points.items():
                reference_2d_point = Point(ref_location.x, ref_location.y)
                reference_point_in_sensing_area = sensing_area.contains(reference_2d_point)
                if reference_point_in_sensing_area:
                    vehicle_in_sensing_area = True
                    sensed_reference_points.append(ref_point_id)
            if vehicle_in_sensing_area:
                reference_points_by_vehicle_in_sensing_area[vehicle_wrapper] = sensed_reference_points

        return reference_points_by_vehicle_in_sensing_area

    def _check_if_other_vehicle_follows_lane(self,
                                             expected_lane_id: LaneIdentifier,
                                             wrapped_other_vehicle: VehicleActorWrapper
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

        other_vehicle_center_point = wrapped_other_vehicle.vehicle.get_location()
        nearest_way_point_to_center_point = self._carla_map.get_waypoint(other_vehicle_center_point)
        nearest_way_point_to_center_point_transform = nearest_way_point_to_center_point.transform
        nearest_way_point_to_center_point_location = nearest_way_point_to_center_point_transform.location

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

        angle_between_vectors = ((nearest_way_point_to_center_point_transform.rotation.yaw % 360)
                                 - (wrapped_other_vehicle.vehicle.get_transform().rotation.yaw % 360))
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

        other_vehicle_rear_center_point = wrapped_other_vehicle.vehicle_points[VehiclePointId.FRONT_MID_CENTER]
        nearest_way_point_to_rear_center_point = self._carla_map.get_waypoint(other_vehicle_rear_center_point)
        nearest_way_point_to_rear_center_point_transform = nearest_way_point_to_rear_center_point.transform
        nearest_way_point_to_rear_center_point_location = nearest_way_point_to_rear_center_point_transform.location

        other_vehicle_front_center_point = wrapped_other_vehicle.vehicle_points[VehiclePointId.REAR_MID_CENTER]
        nearest_way_point_to_front_center_point = self._carla_map.get_waypoint(other_vehicle_front_center_point)
        nearest_way_point_to_front_center_point_transform = nearest_way_point_to_front_center_point.transform
        nearest_way_point_to_front_center_point_location = nearest_way_point_to_front_center_point_transform.location

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

        z_coord_for_debug_lines = self._wrapped_hero_vehicle.vehicle_points[
                                      VehiclePointId.FRONT_MID_CENTER].z * 0.7  # 70% of mid point's height to lower the frame a bit towards the ground
        for line_string in debug_lines:
            first_point_vec = carla.Vector3D(line_string.coords[0][0], line_string.coords[0][1],
                                             z_coord_for_debug_lines)
            second_point_vec = carla.Vector3D(line_string.coords[1][0], line_string.coords[1][1],
                                              z_coord_for_debug_lines)
            self._carla_debug_helper.draw_line(first_point_vec, second_point_vec, CARLA_DEBUG_LINE_THICKNESS, debug_color, CARLA_DEBUG_DRAWING_LIFE_TIME)

    def _draw_debug_lines_for_rear_points_of_other_vehicles(self):
        debug_color = DEBUG_COLOR_DEBUG_LINE_VEHICLE_REAR_END
        for vehicle_wrapper in self._wrapped_other_vehicles:
            line_string_points = vehicle_wrapper.get_rear_mid_center_linestring_points()
            self._carla_debug_helper.draw_line(line_string_points[0], line_string_points[1], CARLA_DEBUG_LINE_THICKNESS, debug_color, CARLA_DEBUG_DRAWING_LIFE_TIME)
        pass
