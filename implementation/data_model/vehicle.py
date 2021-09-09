#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################

from typing import TYPE_CHECKING, Optional, List, Dict, Tuple
from enum import Enum
from dataclasses import dataclass, field
from shapely_ext.decompose import Decomposer
from shapely.geometry import Polygon, LineString

from data_model.entity import EntityObject
from sinadra_configuration_parameters import DEBUG_MODE, SIDE_SENSING_AREA_LENGTH, SENSING_AREA_WAYPOINT_DISTANCE, \
    FRONT_SENSING_DISTANCE_TIME_GAP_IN_SECONDS
from data_model.positions import Vector3D

if TYPE_CHECKING:
    from actor_situation_class_detection.situation_class import SituationClass
    from data_model.positions import Location, WorldPosition
    from data_model.map import Map


class VehiclePointId(Enum):
    FRONT_MID_LEFT = 1
    FRONT_MID_CENTER = 2
    FRONT_MID_RIGHT = 3
    REAR_MID_LEFT = 4
    REAR_MID_CENTER = 5
    REAR_MID_RIGHT = 6
    CENTER_MID_LEFT = 7
    CENTER_MID_RIGHT = 8


class VehicleCategory(Enum):
    BICYCLE = 1
    BUS = 2
    CAR = 3
    MOTORBIKE = 4
    SEMITRAILER = 5
    TRAILER = 6
    TRAM = 7
    TRAIN = 8
    TRUCK = 9
    VAN = 10


@dataclass()
class VehicleLightState(object):
    position: bool = False
    lowbeam: bool = False
    highbeam: bool = False
    right_blinker: bool = False
    left_blinker: bool = False
    reverse: bool = False
    fog: bool = False
    interior: bool = False
    special_1: bool = False
    special_2: bool = False
    all: bool = False
    NONE: bool = False


@dataclass
class KinematicStatus:
    acceleration: float = 0
    acceleration_vector: Vector3D = Vector3D()
    speed: float = 0
    velocity: Vector3D = Vector3D()
    steering_angle: float = 0


@dataclass
class SurroundingVehicles:
    front: Optional["Vehicle"] = None
    front_left: Optional["Vehicle"] = None
    front_right: Optional["Vehicle"] = None
    rear: Optional["Vehicle"] = None
    rear_left: Optional["Vehicle"] = None
    rear_right: Optional["Vehicle"] = None


@dataclass()
class Axle:
    max_steering: Optional[float] = 0
    position_x: Optional[float] = 0
    position_z: Optional[float] = 0
    track_width: Optional[float] = 0
    wheelDiameter: Optional[float] = 0


@dataclass()
class Axles:
    front_axle: Optional[Axle] = None
    rear_axle: [Axle] = None
    additionalAxles: Optional[List[Axle]] = field(default_factory=[])


@dataclass()
class Performance:
    max_acceleration: Optional[float] = 0
    max_deceleration: Optional[float] = 0
    max_speed: Optional[float] = 0


@dataclass()
class VehicleControl():
    throttle: float = 0.0
    steer: float = 0.0
    brake: float = 0.0
    hand_brake: bool = False
    reverse: bool = False
    gear: int = 1


@dataclass()
class Vehicle(EntityObject):
    id: int = -1
    role_name: str = ""
    length: float = -1
    kinematics: Optional[KinematicStatus] = KinematicStatus()
    surrounding_vehicles: Optional[SurroundingVehicles] = None
    light_state: VehicleLightState = VehicleLightState.NONE
    vehicle_control: VehicleControl = VehicleControl()
    vehicle_category: VehicleCategory = VehicleCategory.CAR
    vehicle_points: Dict[VehiclePointId, "Location"] = None

    def get_middle_points_dict(self, vehicle_world_position: "WorldPosition") -> Dict[VehiclePointId, "Location"]:
        """
        Creates a dictionary mapping the middle vertex points of the objects bounding box
         to a VehiclePointID using world coordinates

        Parameters
        ----------
        vehicle_world_position : WorldPosition
            current position of the vehicle in world coordinates

        Returns
        -------
        Dict[VehiclePointId, Location]
            Dictionary mapping the VehiclePointId to the World location
        """
        middle_points = self.bounding_box.get_middle_points(vehicle_world_position)
        middle_points_dict = {
            VehiclePointId.FRONT_MID_LEFT: middle_points[4],
            VehiclePointId.FRONT_MID_CENTER: middle_points[5],
            VehiclePointId.FRONT_MID_RIGHT: middle_points[6],
            VehiclePointId.REAR_MID_LEFT: middle_points[0],
            VehiclePointId.REAR_MID_CENTER: middle_points[1],
            VehiclePointId.REAR_MID_RIGHT: middle_points[2],
            VehiclePointId.CENTER_MID_LEFT: middle_points[7],
            VehiclePointId.CENTER_MID_RIGHT: middle_points[3]
        }
        return middle_points_dict

    def get_acceleration_vector(self) -> Vector3D:
        return self.kinematics.acceleration_vector

    def get_velocity(self) -> Vector3D:
        return self.kinematics.velocity

    def __hash__(self):
        return hash((self.id, self.role_name, self.length))


@dataclass()
class EgoVehicle(Vehicle):
    situation_class: "SituationClass" = None

    def get_side_sensing_area(self, map: "Map", side_vehicle_point_id: VehiclePointId
                              ) -> Tuple[Polygon, List[LineString]]:
        """Determines sensing area next to ego vehicle on other lane. On which side the sensing area is created,
        depends on parameter side_vehicle_point_id.

        Parameters
        ----------
        map: Map
            Map object to obtain points for sensing area calculation
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
        point_next_to_vehicle = self.__get_points_next_to_vehicle_by_side(map, self.get_location(),
                                                                          side_vehicle_point_id)
        if len(point_next_to_vehicle) < 2:
            return None, None
        polygon = self.__create_polygon_from_waypoints(point_next_to_vehicle)
        lines_of_sensing_area = self.__get_lines_of_sensing_area(polygon) if DEBUG_MODE else None
        return polygon, lines_of_sensing_area

    def __get_points_next_to_vehicle_by_side(self, map: "Map", location: "Location",
                                             side_vehicle_point_id: VehiclePointId) -> List["Location"]:
        """
        Returns points on the lane next to the vehicle. The side is determindes by the side_vehicle_point_id.

        Parameters
        ----------
        map : Map
            Reference to the map object to get the point locations
        side_vehicle_point_id : VehiclePointId
            Center mid location of the vehicle side the sensing area shall be created on.
            Possible values are CENTER_MID_LEFT and CENTER_MID_RIGHT

        Returns
        -------
        List[Location]
            list of locations describing the points on the lane next to the vehicle
        """
        point_next_to_ego_on_other_lane = None
        if side_vehicle_point_id == VehiclePointId.CENTER_MID_RIGHT:
            point_next_to_ego_on_other_lane = map.get_point_on_lane_right(location)
        else:
            point_next_to_ego_on_other_lane = map.get_point_on_lane_left(location)

        points_number = int(round(SIDE_SENSING_AREA_LENGTH / SENSING_AREA_WAYPOINT_DISTANCE))

        if points_number % 2 != 0:
            points_number += 1

        points_number_to_each_direction = int(points_number/2)
        start_point = point_next_to_ego_on_other_lane

        points: List["Location"] = []
        offset: float = 0
        for _ in range(points_number_to_each_direction):
            possible_point = map.get_next_point_on_lane(start_point, SENSING_AREA_WAYPOINT_DISTANCE + offset)
            offset += SENSING_AREA_WAYPOINT_DISTANCE
            if possible_point is not None:
                points.append(possible_point)

        points.append(start_point)
        offset = 0
        for _ in range(points_number_to_each_direction):
            possible_point = map.get_next_point_on_lane(start_point, -SENSING_AREA_WAYPOINT_DISTANCE- offset)
            offset -= SENSING_AREA_WAYPOINT_DISTANCE
            if possible_point is not None:
                points.append(possible_point)
        return points

    def get_front_sensing_area(self, map) -> (Polygon, List["LineString"]):
        """Determines front sensing area for this vehicle using the configuration parameter
        FRONT_SENSING_DISTANCE_TIME_GAP_IN_SECONDS. Returns front sensing area as shapely.geometry.Polygon object.
        If configuration parameter DEBUG_MODE is set to true, this method additionally returns the lines of the polygon
        as a list of shapely.geometry.LineString objects for visualization in carla.

        Parameters
        ----------
        map: Map
            Map object to obtain points for sensing area calculation

        Returns
        -------
        Polygon
            A shapely.geometry.Polygon object that defines the front sensing area
        List[LineString]
            A list of shapely.geometry.LineString objects that represent the edges of the front sensing area.
            The line strings are only needed for drawing debug lines of the sensing area on the Carla server. Therefore
            the list is only returned if DEBUG_MODE flag is set to True. Otherwise None is returned instead.
        """
        polygon: Polygon = None
        lines_of_sensing_area: LineString = None

        distance = self.kinematics.speed * FRONT_SENSING_DISTANCE_TIME_GAP_IN_SECONDS
        if distance < 1:
            return polygon, lines_of_sensing_area

        points_in_front = self.__get_points_in_front(self.position.world_position.location, distance, map)

        if len(points_in_front) < 2:
            return polygon, lines_of_sensing_area

        polygon = self.__create_polygon_from_waypoints(points_in_front)
        lines_of_sensing_area = self.__get_lines_of_sensing_area(polygon) if DEBUG_MODE else None

        return polygon, lines_of_sensing_area

    def __get_points_in_front(self, start_location: "Location", distance: float, map: "Map") -> List["Location"]:
        """Returns points in front of the given location. Distance between the points is given by the
        SENSING_AREA_WAYPOINT_DISTANCE parameter.

        Parameters
        ----------
        start_location : Location
            location to the start the calculation of the following points from
        distance : float
            maximum distance to calculate the points. Distance between the last and the starting point <= distance.
        map : Map
            map object to get the next locations

        Returns
        -------
        List[Location]
            list containing the location of all points in front of the start location
        """
        point_number = distance / SENSING_AREA_WAYPOINT_DISTANCE
        points = []
        offset = 0
        for _ in range(int(round(point_number))):
            next_point = None
            next_point = map.get_next_point_on_lane(start_location, SENSING_AREA_WAYPOINT_DISTANCE + offset)
            offset = offset + SENSING_AREA_WAYPOINT_DISTANCE
            if next_point is not None:
                points.append(next_point)
        return points

    def __create_polygon_from_waypoints(self, points: List["Location"]) -> Polygon:
        line = self.__get_line_string_from_waypoints(points)
        dilated = line.buffer(1.5)
        polygon_dict = dilated.__geo_interface__
        polygon_coords = []
        for coord in polygon_dict["coordinates"]:
            for x in  coord:
                polygon_coords.append(x)

        polygon = Polygon(polygon_coords)
        return polygon

    def __get_lines_of_sensing_area(self, polygon):
        decomposer = Decomposer()
        try:
            lines_of_sensing_area = decomposer.decompose(geometry=polygon)
        except ValueError as e:
            # exception is caused by external library but does not affect functional logic as method in try block is
            # only for determining debug lines. Debug lines are set to None for this simulation tick
            lines_of_sensing_area = None
        return lines_of_sensing_area

    def __get_line_string_from_waypoints(self, waypoints: List["Location"]):
        line_strings = []
        if waypoints is None:
            return line_strings
        for point in waypoints:
            line_strings.append((point.x, point.y))
        line_string_obj = LineString(line_strings)
        return line_string_obj


@dataclass()
class OtherVehicle(Vehicle):
    bayesian_network_id: int = 0
    situation_class_role: str = ""

    def __hash__(self):
        return hash((self.id, self.role_name, self.length))
