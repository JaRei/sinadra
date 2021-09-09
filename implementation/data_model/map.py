#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################

import ad_map_access as ad
import math
import os
from typing import TYPE_CHECKING, Optional
from dataclasses import dataclass
from data_model.positions import Position, Location

if TYPE_CHECKING:
    from data_model.positions import Orientation


@dataclass()
class Waypoint:
    """Data class describing a Waypoint on the road

    Attributes
    ----------
    position : Position
        Describes the Position of the Waypoint on the Map for different use cases.
        Currently only WorldPosition is supported
    lane_width : float
        Width of the lane at the waypoints position
    """
    position: Position = Position()
    lane_width: float = 0

    def get_location(self) -> Optional[Location]:
        """Returns the location of waypoint in world coordinates.

        Returns
        -------
        Location
            Location of the waypoint in world coordinates
        """
        return self.position.world_position.location

    def get_orientation(self) -> "Orientation":
        """Returns the orientation of waypoint in world coordinates

        Returns
        -------
        Orientation
            Orientation of the waypoint in world coordinates
        """
        return self.position.world_position.orientation


class Map(object):
    """Map class that holds a representation of the ad map and provides severel functions to work with the map.
    Takes the map name of the current town to create an ad representation.

    Attributes;
    map : ad.map
        ad map object of the current map
    map_matching : ad.map.match.AdMapMatching
        Object that handles all map matching actions
    """
    def __init__(self, map_name):
        """Constructor for the Map class. Loads the ad map from an OpenDrive File

        Parameters
        ----------
        map_name : str
            Name of the map (i.e. Town03) without .xodr
        """
        # start = os.getcwd()
        wd = os.path.dirname(os.path.realpath(__file__))
        f = open(f"{wd}/map_config_file.txt", "w")
        f.write("[ADMap]\n")
        f.write(f"map=/maps/{map_name}.xodr\n")
        f.write("openDriveOverlapMargin=0.2")
        f.close()
        self.map = ad.map.access.init(f"{wd}/map_config_file.txt")
        self.map_matching = ad.map.match.AdMapMatching()

    def get_distance_to_lane_end(self, location: Location) -> float:
        """Returns the distance between the given location to the end of the lane in driving direction.

        Parameters
        ----------
        location : Location
            location of the entity

        Retruns
        -------
        float
            distance between the given location and the end of the lane"""

        lane = self.__get_lane(location)
        map_matching_result = self.__get_map_matched_position_from_enu_point(location.to_ENU())
        offset = float(map_matching_result.lanePoint.paraPoint.parametricOffset)
        length: ad.physics.Distance = map_matching_result.lanePoint.laneLength
        one_meter_as_longitudinal_offset = 1 / float(length)

        if lane.direction == ad.map.lane.LaneDirection.POSITIVE:
            distance = (1 - offset) * float(one_meter_as_longitudinal_offset)
        else:
            distance = offset * float(one_meter_as_longitudinal_offset)

        return distance
        # Lane is too short, go to the following lane

    def get_waypoint(self, location: Location) -> Waypoint:
        """Returns the point in the middle of the lane closest to the given location

        Parameters
        ----------
        location : Location
            reference location for the waypoint

        Returns
        -------
        Location
            location in the middel of the lane, closest to the given location"""

        waypoint = Waypoint()
        enu_point = location.to_ENU()
        map_matching_result = self.__get_map_matched_position_from_enu_point(enu_point)

        offset = float(map_matching_result.lanePoint.paraPoint.parametricOffset)
        lane = ad.map.lane.getLane(map_matching_result.lanePoint.paraPoint.laneId)
        heading = float(ad.map.lane.getLaneENUHeading(map_matching_result))
        heading = math.degrees(heading)
        lane_width = float(map_matching_result.lanePoint.laneWidth)

        ecef_point = ad.map.lane.getParametricPoint(lane, offset, 0.5)
        enu_point = ad.map.point.toENU(ecef_point)

        waypoint.position.world_position.location = location.ENU_to_location(enu_point)
        waypoint.position.world_position.orientation.heading = heading
        waypoint.lane_width = lane_width

        return waypoint

    def get_point_on_lane_right(self, location: Location) -> Location:
        """Returns the point in the middle of the lane to the right of given location

        Parameters
        ----------
        location : Location
            location of current entity

        Returns
        -------
        Location
            location in the middle of the right lane
        """
        enu_point = location.to_ENU()
        map_matched_position = self.__get_map_matched_position_from_enu_point(enu_point)
        heading = ad.map.lane.getLaneENUHeading(map_matched_position)

        current_lane = self.__get_lane(location)
        right_lane = self.__get_lane_right(location)

        distance = float(current_lane.width) / 2 + float(right_lane.width) / 2

        point_on_right_lane = ad.map.point.ENUPoint()
        point_on_right_lane.x = ad.map.point.ENUCoordinate(enu_point.x + math.sin(heading) * distance)
        point_on_right_lane.y = ad.map.point.ENUCoordinate(enu_point.y + math.cos(heading) * distance)
        point_on_right_lane.z = ad.map.point.ENUCoordinate(enu_point.z)

        return location.ENU_to_location(point_on_right_lane)

    def get_point_on_lane_left(self, location: Location) -> Location:
        """Returns the point in the middle of the lane to the left of given location

        Parameters
        ----------
        location : Location
            location of current entity

        Returns
        -------
        Location
            location in the middle of the left lane
        """
        enu_point = location.to_ENU()
        map_matched_position = self.__get_map_matched_position_from_enu_point(enu_point)
        heading = ad.map.lane.getLaneENUHeading(map_matched_position)

        current_lane = self.__get_lane(location)
        left_lane = self.__get_lane_left(location)

        distance = float(current_lane.width) / 2 + float(left_lane.width) / 2

        point_on_left_lane = ad.map.point.ENUPoint()
        point_on_left_lane.x = ad.map.point.ENUCoordinate(enu_point.x - math.sin(heading) * distance)
        point_on_left_lane.y = ad.map.point.ENUCoordinate(enu_point.y - math.cos(heading) * distance)
        point_on_left_lane.z = ad.map.point.ENUCoordinate(enu_point.z)

        return location.ENU_to_location(point_on_left_lane)

    def __get_lane_right(self, location: Location) -> ad.map.lane.Lane:
        """Returns the lane to the right of the lane of the given location.
        Right is determined by the driving direction of the lane

        Parameters
        ----------
        location : Location
            location on the lane the right lane is searched for

        Returns
        -------
        ad.map.lane.Lane:
            lane to the right of the given lane
        """
        lane: ad.map.lane.Lane = self.__get_lane(location)
        if lane.direction == ad.map.lane.LaneDirection.NEGATIVE:
            # Contact lanes of the ad map are based on the lane orientation.
            # lane orientation != lane driving direction
            right_lanes = ad.map.lane.getContactLanes(lane, ad.map.lane.ContactLocation.LEFT)
        else:
            right_lanes = ad.map.lane.getContactLanes(lane, ad.map.lane.ContactLocation.RIGHT)

        right_lane = ad.map.lane.getLane(right_lanes[0].toLane)

        return right_lane

    def __get_lane_left(self, location: Location) -> ad.map.lane.Lane:
        """Returns the lane to the left of the lane of the given location.
        Right is determined by the driving direction of the lane
        Parameters
        ----------
        location : Location
            location on the lane the left lane is searched for

        Returns
        -------
        ad.map.lane.Lane:
            lane to the left of the given lane
        """
        lane: ad.map.lane.Lane = self.__get_lane(location)
        if lane.direction == ad.map.lane.LaneDirection.NEGATIVE:
            # Contact lanes of the ad map are based on the lane orientation.
            # lane orientation != lane driving direction
            left_lanes = ad.map.lane.getContactLanes(lane, ad.map.lane.ContactLocation.RIGHT)
        else:
            left_lanes = ad.map.lane.getContactLanes(lane, ad.map.lane.ContactLocation.LEFT)

        left_lane = ad.map.lane.getLane(left_lanes[0].toLane)

        return left_lane

    def __get_lane(self, location: Location) -> ad.map.lane.Lane:
        """ Returns the lane that contains the given location

        Parameters
        ----------
        location : Location
            position that the corresponding lane is searched for

        Returns
        -------
        ad.map.lane.Lane
            lane containing the given position
        """
        enu_point = ad.map.point.ENUPoint()
        enu_point.x = ad.map.point.ENUCoordinate(location.x)
        enu_point.y = ad.map.point.ENUCoordinate(location.y)
        enu_point.z = ad.map.point.ENUCoordinate(0)

        map_matching_result = self.map_matching.getMapMatchedPositions(enu_point, ad.physics.Distance(0.5),
                                                                       ad.physics.Probability(0.05))
        lane = ad.map.lane.getLane(map_matching_result[0].lanePoint.paraPoint.laneId)

        return lane

    def get_next_point_on_lane(self, start_point: Location, distance: float) -> Location:
        """Returns the point with a certain distance [m] to the given position.
        If the given distance is longer then the remaining lane_length, it continues on the following lane.
        The distance is calculated along the lane.

        Parameters
        ----------
        start_point: positions.Location
            starting position for the distance search

        distance : float
            distance between the given point and the returned one in meter [m]

        Returns
        -------
        positions.Location
            position of the new point
        """
        lane = self.__get_lane(start_point)
        start_point_enu = ad.map.point.ENUPoint()
        start_point_enu.x = ad.map.point.ENUCoordinate(start_point.x)
        start_point_enu.y = ad.map.point.ENUCoordinate(start_point.y)
        start_point_enu.z = ad.map.point.ENUCoordinate(0)

        map_matching_result = self.map_matching.getMapMatchedPositions(start_point_enu, ad.physics.Distance(0.5),
                                                                       ad.physics.Probability(0.05))

        lane_point: ad.map.match.LanePoint = map_matching_result[0].lanePoint.paraPoint
        length: ad.physics.Distance = map_matching_result[0].lanePoint.laneLength
        one_meter_as_longitudinal_offset = 1 / float(length)
        if distance >= 0:
            distance_as_offset = one_meter_as_longitudinal_offset * distance
            if lane.direction == ad.map.lane.LaneDirection.POSITIVE:
                if (1 - float(lane_point.parametricOffset)) < distance_as_offset:
                    # Lane is too short, go to the following lane
                    remaining_distance = distance - ((1 - float(lane_point.parametricOffset)) * float(length))
                    successor = self.__get_successor_lane(lane)
                    next_point_enu = self.__get_next_point_in_lane(successor, remaining_distance)

                else:
                    next_point_enu = self.__get_point_on_lane_from_parametric_offset(
                        lane, float(lane_point.parametricOffset) + distance_as_offset
                    )
            else:
                if float(lane_point.parametricOffset) < distance_as_offset:
                    # Lane is too short, go to the following lane
                    remaining_distance = distance - (float(lane_point.parametricOffset) * float(length))
                    successor = self.__get_successor_lane(lane)
                    next_point_enu = self.__get_next_point_in_lane(successor, remaining_distance)

                else:
                    next_point_enu = self.__get_point_on_lane_from_parametric_offset(
                        lane, float(lane_point.parametricOffset) - distance_as_offset
                    )
        else:
            distance_as_offset = one_meter_as_longitudinal_offset * distance * -1
            if lane.direction == ad.map.lane.LaneDirection.POSITIVE:
                if float(lane_point.parametricOffset) < distance_as_offset:
                    # Lane is too short, go the predecessor lane
                    remaining_distance = distance + (float(lane_point.parametricOffset)) * float(length)
                    predecessor = self.__get_predecessor_lane(lane)
                    next_point_enu = self.__get_next_point_in_lane(predecessor, remaining_distance)
                else:
                    next_point_enu = self.__get_point_on_lane_from_parametric_offset(lane, distance_as_offset * -1)
            else:
                if (1 - float(lane_point.parametricOffset)) < distance_as_offset:
                    # Lane is too short
                    remaining_distance = distance + (1 - float(lane_point.parametricOffset)) * float(length)
                    predecessor = self.__get_predecessor_lane(lane)
                    next_point_enu = self.__get_next_point_in_lane(predecessor, remaining_distance)
                else:
                    next_point_enu = self.__get_point_on_lane_from_parametric_offset(lane, distance_as_offset * -1)
        next_point = Location(next_point_enu.x, next_point_enu.y, next_point_enu.z)

        return next_point

    def __get_successor_lane(self, lane: ad.map.lane.Lane) -> ad.map.lane.Lane:
        """Returns the following lane in driving direction to the given one.

        Parameters
        ----------
        lane : ad.map.lane.Lane
            Lane that the following lane is searched for

        Returns
        -------
        ad.map.lane.Lane
            Following lane  of the given lane
        """
        if lane.direction == ad.map.lane.LaneDirection.POSITIVE:
            successor_lanes = ad.map.lane.getContactLanes(lane, ad.map.lane.ContactLocation.SUCCESSOR)
            successor_lane = ad.map.lane.getLane(successor_lanes[0].toLane)
        else:
            successor_lanes = ad.map.lane.getContactLanes(lane, ad.map.lane.ContactLocation.PREDECESSOR)
            successor_lane = ad.map.lane.getLane(successor_lanes[0].toLane)

        return successor_lane

    def __get_predecessor_lane(self, lane: ad.map.lane.Lane) -> ad.map.lane.Lane:
        """Returns the predecessor lane in driving direction to the given one.

                Parameters
                ----------
                lane : ad.map.lane.Lane
                    Lane that the predecessor lane is searched for

                Returns
                -------
                ad.map.lane.Lane
                    Predecessor lane  of the given lane
                """
        if lane.direction == ad.map.lane.LaneDirection.NEGATIVE:
            predecessor_lanes = ad.map.lane.getContactLanes(lane, ad.map.lane.ContactLocation.SUCCESSOR)
            predecessor_lane = ad.map.lane.getLane(predecessor_lanes[0].toLane)
        else:
            predecessor_lanes = ad.map.lane.getContactLanes(lane, ad.map.lane.ContactLocation.PREDECESSOR)
            predecessor_lane = ad.map.lane.getLane(predecessor_lanes[0].toLane)

        return predecessor_lane

    def __get_map_matched_position_from_enu_point(self, enu_point: ad.map.point.ENUPoint, distance: float = 1,
                                                  probability: float = 0.05) -> ad.map.match.MapMatchedPosition:
        """Returns the map matched position of the given enu point

        Parameters
        ----------
        enu_point : ad.map.point.ENUPoint
            position of interest as enu point
        distance : float
            search radius around the given point to select a lane as result
        probability : float
            A probability threshold to be considered for the results

        Returns
        -------
        ad.map.match.MapMatchedPosition
            MapMatchedPosition object of the given enu point
        """

        ad_distance = ad.physics.Distance(distance)
        ad_probability = ad.physics.Probability(probability)
        map_matched_positions = self.map_matching.getMapMatchedPositions(enu_point, ad_distance, ad_probability)
        return  map_matched_positions[0]

    def __get_next_point_in_lane(self, lane: ad.map.lane.Lane, distance: float) -> ad.map.point.ENUPoint:
        """Returns the point on the given that has the specified distance in [m] from the starting point (in driving
        direction) of the lane. If the given distance is longer than the lane distance, it will continue its search in
        the following lane.
        If a negative distance value is given, it will go backwards compared to the driving direction.

         Parameters
         ----------
        lane : ad.map.lane.Lane
            current lane
        distance : float
            distance of the new point in meter [m] from the starting point of the lane

        Returns
        -------
        ad.map.point.ENUPoint
            position of the point with the specified distance
        """
        one_meter_as_longitudinal_offset = 1 / float(lane.length)
        distance_as_offset = one_meter_as_longitudinal_offset * distance
        next_point: ad.map.point.ENUPoint = None
        if distance >= 0:
            if distance > lane.length:
                # lane is too short
                successor = self.__get_successor_lane(lane)
                remaining_distance = distance - float(lane.length)
                next_point = self.__get_next_point_in_lane(successor, remaining_distance)
            else:
                next_point = self.__get_point_on_lane_from_parametric_offset(lane, distance_as_offset)
        else:
            if distance > lane.length:
                # lane is too short
                predecessor = self.__get_predecessor_lane(lane)
                remaining_distance = distance + float(lane.length)
                next_point = self.__get_next_point_in_lane(lane, remaining_distance)
            else:
                next_point = self.__get_point_on_lane_from_parametric_offset(lane, distance_as_offset)

        return next_point

    def __get_point_on_lane_from_parametric_offset(self, lane: ad.map.lane.Lane,
                                                   longitudinal_distance: float) -> ad.map.point.ENUPoint:
        """Returns the point of the lane with a given longitudinal offset. The offset has the range from [0 - 1]
        and specifies the length of the lane. 0 refers to the start point the lane. The start point is determined using
        the lanes orientation.
        Caution: The orientation can differ from the driving direction

        Parameters
        ----------
        lane : ad.map.lane.Lane
            current lane

        longitudinal_distance : float
            parametric position inside the lane [-1; 1]. Negative value indicates that it takes the lanes EndPosition
            as starting point

        Returns
        -------
        ad.map.point.ENUPoint
            world position of the point with longitudinal_distance of the given lane
        """
        if longitudinal_distance < 0:
            offset = 1 - longitudinal_distance
        else:
            offset = longitudinal_distance
        ecef_point = ad.map.lane.getParametricPoint(lane, offset, 0.5)
        enu_point = ad.map.point.toENU(ecef_point)
        return enu_point
