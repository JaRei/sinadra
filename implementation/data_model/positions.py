#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################

from typing import Optional, Union, Tuple, List
from dataclasses import dataclass
from enum import Enum
import ad_map_access as ad
import math


class ReferenceContext(Enum):
    ABSOLUTE = 1
    RELATIVE = 2


@dataclass()
class Vector3D:
    x: float = 0
    y: float = 0
    z: float = 0

    def invert(self) -> "Vector3D":
        """Returns the inverted Vector

        Returns
        -------
        Vector3D
            Inverted Vector"""
        vec = Vector3D
        vec.x = -self.x
        vec.y = -self.y
        vec.z = -self.z
        return vec

    def __add__(self, other_location: Union["Location", "Vector3D"]) -> "Vector3D":
        new_loc_x = self.x + other_location.x
        new_loc_y = self.y + other_location.y
        new_loc_z = self.z + other_location.z
        return Location(new_loc_x, new_loc_y, new_loc_z)

    def __sub__(self, other_location: Union["Location", "Vector3D"]) -> "Vector3D":
        new_loc_x = self.x - other_location.x
        new_loc_y = self.y - other_location.y
        new_loc_z = self.z - other_location.z
        return Location(new_loc_x, new_loc_y, new_loc_z)

    def __mul__(self, other: Union["Location", "Vector3D", float]) -> "Vector3D":
        if isinstance(other, float):
            self.x *= other
            self.y *= other
            self.z *= other
        elif isinstance(other, (Location, Vector3D)):
            self.x *= other.x
            self.y *= other.y
            self.z *= other.z
        return self

    def __iter__(self):
        return iter([self.x, self.y, self.z])


@dataclass()
class Location(Vector3D):

    def to_ENU(self) -> ad.map.point.ENUPoint:
        """Returns an ENU Point of the Location

        Returns
        -------
        ad.map.point.ENUPoint
            ENU Point of the Location"""
        enu_point = ad.map.point.ENUPoint()
        enu_point.x = ad.map.point.ENUCoordinate(self.x)
        enu_point.y = ad.map.point.ENUCoordinate(self.y)
        enu_point.z = ad.map.point.ENUCoordinate(0)
        return enu_point

    def ENU_to_location(self, enu_point: ad.map.point.ENUPoint) -> "Location":

        """Returns the Location object of the given enu point

        Parameters
        ----------
        enu_point : ad.map.point.ENUPoint
            enu_point on the Map

        Returns
        -------
        Location
            location object of the given enu point"""
        location = Location()
        location.x = enu_point.x
        location.y = enu_point.y
        location.z = enu_point.z
        return location

    def get_2d_distance(self, other_location: "Location") -> float:
        """Calculates the 2-dimensional distance of the object location and the given other location.

        Parameters
        ----------
        other_location : VehicleLocation
            Second location to calculate the distance to.

        Returns
        -------
        float
            Distance between the two locations.
        """
        distance = math.hypot(self.x - other_location.x, self.y - other_location.y)
        return distance

    def distance(self, other_location: "Location") -> float:
        """Calculates the 3-dimensional distance of the object location and the given other location.

        Parameters
        ----------
        other_location : VehicleLocation
            Second location to calculate the distance to.

        Returns
        -------
        float
            Distance between the two locations.
        """
        distance = math.sqrt(
            (self.x - other_location.x)**2 + (self.y - other_location.y)**2 + (self.z - other_location.z)**2
        )
        return distance

    def invert(self) -> "Location":
        """Returns the inverted Location

                Returns
                -------
                Location
                    Inverted Location"""
        loc = Location()
        loc.x = -self.x
        loc.y = -self.y
        loc.z = -self.z
        return loc


@dataclass
class Orientation:
    """Describes the Orientation of its Entity, either in the local or the global coordinate system

    Attributes
    ----------
    heading : float
        Heading (Yaw) of the Entity (Rotation around z-axe)
    pitch : float
        Rotation around the y-axe of the entity
    roll : float
        Rotation around the x-axe of the entity
    type : ReferenceContext
        Describes if the Orientaion is given in global (Absolut) or local (relative) space.
        Currently not set
    """
    heading: float = 0
    pitch: float = 0
    roll: float = 0
    type: Optional[ReferenceContext] = None

    def get_forward_vector(self) -> Vector3D:
        """Returns a vector directing in forward direction of the vehicle.

        Returns
        -------
        Vector3D
            vector in forward direction with length 1
        """

        vector = Vector3D()
        vector.x = math.cos(math.radians(self.heading))
        vector.y = math.sin(math.radians(self.heading))

        return vector


@dataclass
class WorldPosition:
    """Data class containg an entities transform in world coordinates.

    Attributes
    ----------
    location : Location
        position of the entity in world coordinates (x, y, z)
    orientation : Orientation
        orientation of the entity in world coordinates (heading, pitch, roll)
    """
    location: Optional[Location] = Location()
    orientation: Optional[Orientation] = Orientation()

    def get_forward_vector(self) -> Vector3D:
        """Returns a vector directing in forward direction of the vehicle.

        Returns
        -------
        Vector3D
            vector in forward direction with length 1
        """

        return Orientation.get_forward_vector()

    def get_inverse_matrix(self) -> List[List[float]]:
        """Returns the inverse 4x4 transformation matrix of the entity

        Returns
        -------
        List[List[float]]
            4x4 inverse transformation matrix
        """
        ch, sh = self.__create_cos_sin(self.orientation.heading)
        cr, sr = self.__create_cos_sin(self.orientation.roll)
        cp, sp = self.__create_cos_sin(self.orientation.pitch)
        a = Vector3D(0, 0, 0)
        loc = self.__inverse_transform_point()

        # loc.z value is different than from carla
        inverse_matrix = [
            [cp * ch, cp * sh, -sp, loc.x],
            [ch * sp * sr - sh * cr, sh * sp * sr + ch * cr, cp * sr, loc.y],
            [ch * sp * cr + sh * sr, sh * sp * cr - ch * sr, cp * cr, loc.z],
            [0.0, 0.0, 0.0, 1.0]
        ]
        return inverse_matrix

    def get_matrix(self) -> List[List[float]]:
        """Returns the 4x4 transformation matrix of the entity
        R|t
        0|1
        Returns
        -------
        List[List[float]]
            4x4 transformation matrix
        """
        ch, sh = self.__create_cos_sin(self.orientation.heading)
        cr, sr = self.__create_cos_sin(self.orientation.roll)
        cp, sp = self.__create_cos_sin(self.orientation.pitch)
        loc = self.location

        matrix = [
            [cp * ch, ch * sp * sr - sh * cr, ch * sp * cr + sh * sr, loc.x],
            [cp * sh, sh * sp * sr + ch * cr, sh * sp * cr - ch * sr, loc.y],
            [-sp, cp * sr, cp * cr, loc.z],
            [0.0, 0.0, 0.0, 1.0]
        ]
        return matrix

    def __inverse_transform_point(self) -> Location:
        """Returns the inverse transformation of the point
        Returns
        -------
        Location
            Inverse of the Location
        """
        inv_loc = Location.invert(self.location) # translate the point
        ch, sh = self.__create_cos_sin(self.orientation.heading)
        cr, sr = self.__create_cos_sin(self.orientation.roll)
        cp, sp = self.__create_cos_sin(self.orientation.pitch)

        out_point = Location()  # than rotate the point
        out_point.x = inv_loc.x * (cp * ch) + inv_loc.y * (cp * sh) + inv_loc.z * (-sp)
        out_point.y = (
                inv_loc.x * (ch * sp * sr - sh * cr) + inv_loc.y * (sh * sp * sr + ch * cr) + inv_loc.z * (cp * sr)
        )
        out_point.z = (
                inv_loc.x * (ch * sp * cr + sh * sr) + inv_loc.y * (sh * sp * cr - ch * sr) + inv_loc.z * (cp * cr)
        )
        return out_point

    def __create_cos_sin(self, angle) -> Tuple[float, float]:
        ca = math.cos(math.radians(angle))
        sa = math.sin(math.radians(angle))

        return ca, sa


@dataclass
class RelativeWorldPosition:
    # TODO add EntityRef
    dx: float = 0
    dy: float = 0
    dz: float = 0
    orientation: Orientation = Orientation()


@dataclass
class LanePosition:
    """Position of an entity along a Lane
    Attributes
    ----------
    laneId : str
        unique identifier of the lane
    offset : float
        lateral offset to the centerline of the current lane in [m]
    orientation : Orientation

    roadId : str
        unique identifier of the road the lane belongs to
    s : float
        current position along the lanes centerline in [m]
    """
    laneId: str = -1
    offset: float = 0
    orientation: Orientation = Orientation()
    roadId: str = -1
    s: float = 0


@dataclass()
class RelativeLanePosition:
    # TODO add entitiyRef
    dLane: int = -1
    ds: float = 0
    dsLane: float = 0
    offset: float = 0
    orientation: Orientation = Orientation()


@dataclass()
class RoadPosition:
    orientation: Orientation = Orientation()
    roadId: str = -1
    s: float = 0
    t: float = 0


@dataclass()
class RelativeRoadPosition:
    # TODO add entityRef
    ds: float = 0
    dt: float = 0
    orientation: Orientation = Orientation()


@dataclass()
class TrajectoryPosition:
    # TODO add trajectoryRef
    orientation: Orientation = Orientation()
    s: float = 0
    t: float = 0


@dataclass()
class GeoPosition:
    height: float = 0
    latitude: float = 0
    longitude: float = 0
    orientation: Orientation = Orientation()


@dataclass()
class RelativeObjectPosition:
    # TODO add entityRef
    dx: float = 0
    dy: float = 0
    dz: float = 0
    orientation: Orientation = Orientation()


@dataclass()
class RoutePosition:
    orientation: Orientation = Orientation()


@dataclass()
class PositionOfCurrentEntity:
    # TODO add entityRef
    pass


@dataclass()
class PositionInRoadCoordinates:
    pathS: float = 0
    t: float = 0


@dataclass()
class PositionInLaneCoordinates:
    laneId: str = 0
    laneOffset: float = 0
    pathS: float = 0


@dataclass()
class InRoutePosition:
    positionOfCurrentEntity: PositionOfCurrentEntity = PositionOfCurrentEntity()
    positionInRoadCoordinates: PositionInRoadCoordinates = PositionInRoadCoordinates()
    positionInLaneCoordinates: PositionInLaneCoordinates = PositionInLaneCoordinates()


@dataclass()
class Position:
    world_position: WorldPosition = WorldPosition()
    relative_world_position: Optional[RelativeWorldPosition] = None
    lane_position: Optional[LanePosition] = None
    relative_lane_position: Optional[RelativeLanePosition] = None
    road_position: Optional[RoadPosition] = None
    relative_road_position: Optional[RelativeRoadPosition] = None
    trajectory_position: Optional[TrajectoryPosition] = None
    route_position: Optional[RoutePosition] = None
    geo_position: Optional[GeoPosition] = None
    relative_object_position: Optional[RelativeObjectPosition] = None
