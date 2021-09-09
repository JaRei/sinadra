#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
import inspect
import math
from enum import Enum
from typing import List, Dict, TYPE_CHECKING

from data_model.positions import Vector3D
from shapely.geometry import Polygon, LineString, Point
from shapely_ext.decompose import Decomposer

from sinadra_configuration_parameters import DEBUG_MODE, FRONT_SENSING_DISTANCE_TIME_GAP_IN_SECONDS, \
    SENSING_AREA_WAYPOINT_DISTANCE, SIDE_SENSING_AREA_LENGTH

if TYPE_CHECKING:
    from data_model.positions import Location
    from carla import Vehicle
    from carla import Map
    from carla import Waypoint


class VehiclePointId(Enum):
    FRONT_MID_LEFT = 1
    FRONT_MID_CENTER = 2
    FRONT_MID_RIGHT = 3
    REAR_MID_LEFT = 4
    REAR_MID_CENTER = 5
    REAR_MID_RIGHT = 6
    CENTER_MID_LEFT = 7
    CENTER_MID_RIGHT = 8


class VehicleActorWrapper(object):
    """Wrapper class for a carla.Vehicle object. The carla.Vehicle object can be accessed through the vehicle attribute.

    Attributes
    ----------
    vehicle: carla.Vehicle
        carla.Vehicle object that is wrapped within this object.
    vehicle_points: Dict[VehiclePointId, carla.Vector3D]
        Wrapped vehicle actors that are on the right lane


    """
    def __init__(self, vehicle: "Vehicle"):
        self.vehicle = vehicle
        self.vehicle_points = self._get_vehicle_points()
        pass

    @property
    def role_name(self):
        """Retrieves the role_name attribute value of the wrapped carla.Vehicle object"""
        return self.vehicle.attributes["role_name"]

    @property
    def id(self):
        """Retrieves the id of the wrapped carla.Vehicle object"""
        return self.vehicle.id

    @property
    def transform(self):
        """Retrieves the carla.Transform object of the wrapped carla.Vehicle object"""
        return self.vehicle.get_transform()

    @property
    def forward_vector(self):
        """Retrieves the forward vector as carla.Vector3D object of the wrapped carla.Vehicle object"""
        return self.transform.get_forward_vector()

    def get_front_sensing_area(self, carla_map: "Map") -> (Polygon, List["LineString"]):
        """Determines front sensing area for this vehicle using the configuration parameter
        FRONT_SENSING_DISTANCE_TIME_GAP_IN_SECONDS. Returns front sensing area as shapely.geometry.Polygon object.
        If configuration parameter DEBUG_MODE is set to true, this method additionally returns the lines of the polygon
        as a list of shapely.geometry.LineString objects for visualization in carla.

        Parameters
        ----------
        carla_map: carla.Map
            Map object to obtain way points for sensing area calculation

        Returns
        -------
        Polygon
            A shapely.geometry.Polygon object that defines the front sensing area
        List[LineString]
            A list of shapely.geometry.LineString objects that represent the edges of the front sensing area.
            The line strings are only needed for drawing debug lines of the sensing area on the Carla server. Therefore
            the list is only returned if DEBUG_MODE flag is set to True. Otherwise None is returned instead.
        """

        polygon = None
        lines_of_sensing_area = None

        hero_waypoint = carla_map.get_waypoint(self.vehicle_points[VehiclePointId.FRONT_MID_CENTER])
        distance = self._get_speed_value() * FRONT_SENSING_DISTANCE_TIME_GAP_IN_SECONDS
        if distance < 1:
            return polygon, lines_of_sensing_area

        waypoints_in_front = self._get_waypoints_in_front(hero_waypoint, distance)

        if len(waypoints_in_front) < 2:
            return polygon, lines_of_sensing_area

        polygon = self._create_polygon_from_waypoints(waypoints_in_front)

        lines_of_sensing_area = self._get_lines_of_sensing_area(polygon) if DEBUG_MODE else None

        return polygon, lines_of_sensing_area

    def get_side_sensing_area(self,
                              carla_map: "Map",
                              side_vehicle_point_id: VehiclePointId
                              ) -> (Polygon, List["LineString"]):
        """Determines sensing area next to ego vehicle on other lane. On which side the sensing area is created,
        depends on parameter side_vehicle_point_id.

        Parameters
        ----------
        carla_map: carla.Map
            Map object to obtain way points for sensing area calculation
        side_vehicle_point_id: VehiclePointId
            Center mid location of the vehicle side the sensing area shall be created on.
            Possible values are CENTER_MID_LEFT and CENTER_MID_RIGHT

        Returns
        -------
        Polygon
            A shapely.geometry.Polygon object that defines the side sensing area
        List[LineString]
            A list of shapely.geometry.LineString objects that represent the edges of the side sensing area.
            The line strings are only needed for drawing debug lines of the sensing area on the Carla server. Therefore
            the list is only returned if DEBUG_MODE flag is set to True. Otherwise None is returned instead.
        """
        waypoints_next_to_vehicle = self._get_waypoints_next_to_vehicle_by_side(carla_map, side_vehicle_point_id)
        if len(waypoints_next_to_vehicle) < 2:
            return None, None
        polygon = self._create_polygon_from_waypoints(waypoints_next_to_vehicle)
        lines_of_sensing_area = self._get_lines_of_sensing_area(polygon) if DEBUG_MODE else None
        return polygon, lines_of_sensing_area

    def _get_vehicle_points(self) -> List["VehiclePointId"]:
        bounding_box = self.vehicle.bounding_box

        bb_world_vertices = bounding_box.get_world_vertices(self.vehicle.get_transform())

        # front mid left
        front_top_left = bb_world_vertices[5]
        front_bottom_left = bb_world_vertices[4]
        front_mid_left = self._get_middle_point(front_top_left, front_bottom_left)

        # front mid right
        front_top_right = bb_world_vertices[7]
        front_bottom_right = bb_world_vertices[6]
        front_mid_right = self._get_middle_point(front_top_right, front_bottom_right)

        # front mid center
        front_mid_center = self._get_middle_point(front_mid_left, front_mid_right)

        # rear mid left
        rear_top_left = bb_world_vertices[1]
        rear_bottom_left = bb_world_vertices[0]
        rear_mid_left = self._get_middle_point(rear_top_left, rear_bottom_left)

        # rear mid right
        rear_top_right = bb_world_vertices[3]
        rear_bottom_right = bb_world_vertices[2]
        rear_mid_right = self._get_middle_point(rear_top_right, rear_bottom_right)

        # rear mid center
        rear_mid_center = self._get_middle_point(rear_mid_left, rear_mid_right)

        center_mid_left = self._get_middle_point(rear_mid_left, front_mid_left)
        center_mid_right = self._get_middle_point(rear_mid_right, front_mid_right)

        vehicle_points = {
            VehiclePointId.FRONT_MID_LEFT: front_mid_left,
            VehiclePointId.FRONT_MID_CENTER: front_mid_center,
            VehiclePointId.FRONT_MID_RIGHT: front_mid_right,
            VehiclePointId.REAR_MID_LEFT: rear_mid_left,
            VehiclePointId.REAR_MID_CENTER: rear_mid_center,
            VehiclePointId.REAR_MID_RIGHT: rear_mid_right,
            VehiclePointId.CENTER_MID_LEFT: center_mid_left,
            VehiclePointId.CENTER_MID_RIGHT: center_mid_right
        }
        return vehicle_points

    def _get_middle_point(self, starting_point, end_point):
        vector_between_points = end_point - starting_point
        half_vector_between_points = vector_between_points * 0.5
        middle_point = half_vector_between_points + starting_point
        return middle_point

    def _get_waypoints_in_front(self, start_waypoint, distance):
        waypoint_number = distance / SENSING_AREA_WAYPOINT_DISTANCE
        waypoints = []
        start_waypoint = start_waypoint
        for _ in range(int(round(waypoint_number))):
            next_waypoint = None
            next_waypoints = list(
                filter(lambda wp: wp.lane_id == start_waypoint.lane_id, start_waypoint.next(SENSING_AREA_WAYPOINT_DISTANCE)))

            if any(next_waypoints):
                next_waypoint = next_waypoints[0]

            if next_waypoint is not None:
                waypoints.append(next_waypoint)
                start_waypoint = next_waypoint
        return waypoints

    def _get_waypoints_next_to_vehicle_by_side(self, carla_map: "Map", side_vehicle_point_id: VehiclePointId):
        right_vector = self.transform.get_right_vector()
        reference_point_on_vehicle_side = None
        if side_vehicle_point_id == VehiclePointId.CENTER_MID_RIGHT or side_vehicle_point_id == VehiclePointId.CENTER_MID_LEFT:
            reference_point_on_vehicle_side = self.vehicle_points[side_vehicle_point_id]

        if reference_point_on_vehicle_side is None:
            raise Exception(
                f'exception from {self.__class__.__name__}.{inspect.currentframe().f_code.co_name}:\nside_vehicle_point_id valid values: {VehiclePointId.CENTER_MID_LEFT.name} and {VehiclePointId.CENTER_MID_RIGHT.name}')

        vehicle_reference_waypoint = carla_map.get_waypoint(reference_point_on_vehicle_side)

        if side_vehicle_point_id == VehiclePointId.CENTER_MID_RIGHT:
            side_vector = right_vector
        else:
            side_vector = right_vector * -1

        wp_next_to_ego_on_other_lane = carla_map.get_waypoint(reference_point_on_vehicle_side + (side_vector * 3))
        waypoint_number = int(round(SIDE_SENSING_AREA_LENGTH / SENSING_AREA_WAYPOINT_DISTANCE))

        if waypoint_number % 2 != 0:
            waypoint_number += 1

        waypoint_number_to_each_direction = int(waypoint_number / 2)

        # previous waypoints
        start_waypoint_for_prev = wp_next_to_ego_on_other_lane
        previous_waypoints = self._get_next_or_previous_waypoints_for_side_lane_sensing(start_waypoint_for_prev,
                                                                                        'previous',
                                                                                        waypoint_number_to_each_direction,
                                                                                        vehicle_reference_waypoint.lane_id)

        start_waypoint_for_next = wp_next_to_ego_on_other_lane
        next_waypoints = self._get_next_or_previous_waypoints_for_side_lane_sensing(start_waypoint_for_next,
                                                                                    'next',
                                                                                    waypoint_number_to_each_direction,
                                                                                    vehicle_reference_waypoint.lane_id)

        waypoints_next_to_ego = []
        waypoints_next_to_ego.extend(previous_waypoints)
        waypoints_next_to_ego.append(wp_next_to_ego_on_other_lane)
        waypoints_next_to_ego.extend(next_waypoints)
        return waypoints_next_to_ego

    def _get_next_or_previous_waypoints_for_side_lane_sensing(self,
                                                              starting_waypoint: "Waypoint",
                                                              direction_string: str,
                                                              number_of_waypoints: int,
                                                              vehicle_wp_lane_id: int
                                                              ) -> List["Waypoint"]:
        """Returns waypoints to a single given direction depending on value of parameter direction_string for side
        sensing area.

        Parameters
        ----------
        starting_waypoint: carla.Waypoint
            Way point to start searching for either next or previous way points
        direction_string: str
            Indicates if next or previous way points are collected. Valid values: 'next', 'previous'
        number_of_waypoints: int
            Number of way points that shall be retrieved for one direction
        vehicle_wp_lane_id: int
            OPENDRIVE lane id of the lane the ego vehicle is currently on
        Returns
        -------
        List[carla.Waypoint]
            List of way points for given direction
        """
        direction_string_lower = str.lower(direction_string)
        if not (direction_string_lower == 'next' or direction_string_lower == 'previous'):
            raise Exception('Value of param direction_string invalid. Valid values: next, previous')

        waypoints = []
        for _ in range(number_of_waypoints):
            subsequent_wp = None
            if direction_string_lower == 'next':
                possible_subsequent_waypoints = list(filter(lambda wp: wp.lane_id != vehicle_wp_lane_id,
                                                            starting_waypoint.next(SENSING_AREA_WAYPOINT_DISTANCE)))
            else:
                possible_subsequent_waypoints = list(filter(lambda wp: wp.lane_id != vehicle_wp_lane_id,
                                                            starting_waypoint.previous(SENSING_AREA_WAYPOINT_DISTANCE)))

            if any(possible_subsequent_waypoints):
                subsequent_wp = possible_subsequent_waypoints[0]

            if subsequent_wp is not None:
                waypoints.append(subsequent_wp)
                starting_waypoint = subsequent_wp
        return waypoints

    def _create_polygon_from_waypoints(self, waypoints: List["Waypoint"]):
        line = self._get_line_string_from_waypoints(waypoints)
        dilated = line.buffer(1.5)
        polygon_dict = dilated.__geo_interface__  # {'type':'LineString', 'coordinates':tuple(coords)}
        polygon_cords = []
        for cord in polygon_dict['coordinates']:
            for x in cord:
                polygon_cords.append(x)

        polygon = Polygon(polygon_cords)
        return polygon

    def _get_line_string_from_waypoints(self, waypoints: List["Waypoint"]):
        line_strings = []
        if waypoints is None:
            return line_strings
        for waypoint in waypoints:
            wp_location = waypoint.transform.location
            line_strings.append((wp_location.x, wp_location.y))
        line_string_obj = LineString(line_strings)
        return line_string_obj

    def _get_lines_of_sensing_area(self, sensing_area_polygon):
        decomposer = Decomposer()
        try:
            lines_of_sensing_area = decomposer.decompose(geometry=sensing_area_polygon)
        except ValueError as e:
            # exception is caused by external library but does not affect functional logic as method in try block is
            # only for determining debug lines. Debug lines are set to None for this simulation tick
            lines_of_sensing_area = None
        return lines_of_sensing_area

    def get_rear_mid_center_linestring_points(self) -> List["Point"]:
        """Method is used for drawing debug lines. It returns a tuple of two points. First is located above and
        second is located beneath the rear center point of the vehicle.

        Returns
        -------
        List[Point]
            A tuple of shapely.geometry.Point objects, where the first point is located above
            and the second is located beneath the rear center point of the vehicle.
        """
        rear_mid_center_vector = self.vehicle_points[VehiclePointId.REAR_MID_CENTER]

        upper_line_string_point_vec = Vector3D(rear_mid_center_vector.x, rear_mid_center_vector.y,
                                               rear_mid_center_vector.z + 3)
        lower_line_string_point_vec = Vector3D(rear_mid_center_vector.x, rear_mid_center_vector.y,
                                               rear_mid_center_vector.z - 3)

        return [upper_line_string_point_vec, lower_line_string_point_vec]

    def _get_speed_value(self) -> float:
        velocity = self.vehicle.get_velocity()
        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        return speed


def get_vehicles(carla_world):
    all_vehicles = carla_world.get_actors().filter("vehicle.*")

    hero_vehicle = None

    for vehicle in all_vehicles:
        if vehicle.attributes["role_name"] == "hero":
            hero_vehicle = vehicle
            # set_hero_location(hero_vehicle)
            break
    other_vehicles = list(filter(lambda v: v.attributes["role_name"] != "hero", all_vehicles))

    return (other_vehicles, hero_vehicle)


def get_wrapped_vehicles(hero_vehicle, other_vehicles):
    wrapped_hero_vehicle = VehicleActorWrapper(hero_vehicle)
    wrapped_other_vehicles = []
    for vehicle in other_vehicles:
        wrapped_other_vehicles.append(VehicleActorWrapper(vehicle))
    all_wrapped_vehicles = [wrapped_hero_vehicle]
    all_wrapped_vehicles.extend(wrapped_other_vehicles)
    return all_wrapped_vehicles, wrapped_hero_vehicle, wrapped_other_vehicles