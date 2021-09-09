#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional, List
from enum import Enum
from data_model.positions import Position, Location, WorldPosition, Orientation

if TYPE_CHECKING:
    from data_model.vehicle import KinematicStatus


@dataclass()
class Dimension:
    """Dimensions for a three dimensional box. Width, length and height are the absolute extensions in the
    (x, y, z) coordinate system of the entity's local coordinate system

    Attributes
    ----------
    length : float
        Length of the entity's bounding box
    width : float
        Width of the entity's bounding box
    height : float
        Height of the entity's bounding box
    """
    length: float = 0
    width: float = 0
    height: float = 0


@dataclass()
class BoundingBox:
    """ Defines geometric properties of the entities as a simplified three dimensional bounding box.
    (Python dataclasses.dataclass object.)

    Attributes
    ----------
    center : Location
        Represents the geometrical center of the bounding expressed in coordinates that refer to the coordinate
        system of the entity
    dimension: Dimension
        Width, length and height of the bounding box
    """
    center: Location = Location()
    dimension: Dimension = Dimension()

    def get_middle_points(self, world_position: WorldPosition) -> List[Location]:
        """

        """
        world_vertices = self.get_world_vertices(world_position)
        front_mid_left: Location = self.__get_middle_point(world_vertices[5], world_vertices[4])
        front_mid_right: Location = self.__get_middle_point(world_vertices[7], world_vertices[6])
        front_mid_center: Location = self.__get_middle_point(front_mid_left, front_mid_right)
        rear_mid_left: Location = self.__get_middle_point(world_vertices[1], world_vertices[0])
        rear_mid_right: Location = self.__get_middle_point(world_vertices[3], world_vertices[2])
        rear_mid_center: Location = self.__get_middle_point(rear_mid_left, rear_mid_right)
        center_mid_left: Location = self.__get_middle_point(rear_mid_left, front_mid_left)
        center_mid_right: Location = self.__get_middle_point(rear_mid_right, front_mid_right)

        vehicle_middle_points = [
            rear_mid_left,
            rear_mid_center,
            rear_mid_right,
            center_mid_right,
            front_mid_left,
            front_mid_center,
            front_mid_right,
            center_mid_left
        ]
        return vehicle_middle_points

    @staticmethod
    def __get_middle_point(start: Location, end: Location):
        """

        """
        vector_between_points = end - start
        half_vector_between_points = vector_between_points * 0.5
        middle_point = half_vector_between_points + start
        return middle_point

    @staticmethod
    def __rotate_vector(location: Location, orientation: Orientation) -> Location:
        """Rotates a given location vector using the orientation

        Parameters
        ----------
        location : Location
            location of the point to rotate
        orientation : Orientation
            rotation vector used to rotate the location

        Returns
        -------
        Location
            Rotated location of the given location
        """
        ch: float = math.cos(math.radians(orientation.heading))
        sh: float = math.sin(math.radians(orientation.heading))
        cr: float = math.cos(math.radians(orientation.roll))
        sr: float = math.sin(math.radians(orientation.roll))
        cp: float = math.cos(math.radians(orientation.pitch))
        sp: float = math.sin(math.radians(orientation.pitch))

        new_loc_x: float = location.x * (cp * ch) + \
                           location.y * (ch * sp * sr - sh * cr) + \
                           location.z * (-ch * sp * cr - sh * sr)

        new_loc_y: float = location.x * (cp * sh) + \
                           location.y * (sh * sp * sr + ch * cr) + \
                           location.z * (-sh * sp * cr + ch * sr)

        new_lox_z: float = location.x * sp + \
                           location.y * (-cp * sr) + \
                           location.z * (cp * cr)

        new_loc = Location(new_loc_x, new_loc_y, new_lox_z)

        return new_loc

    def get_world_vertices(self, world_position: WorldPosition) -> List[Location]:
        """Returns the vertices of the bounding box using world coordinates

        Parameters
        ----------
        world_position : WorldPosition
            Rotation and Location of the respective entity in world coordinates.
        Returns
        -------
        List[Location]
            List of Location objects representing a vertice using world coordinates
        """
        local_vertices: List[Location] = self.get_local_vertices()
        world_vertices: List[Location] = []
        for vertex in local_vertices:
            vertex = self.__rotate_vector(vertex, world_position.orientation)  # rotate vecotr
            world_vertex = vertex +  world_position.location  # translate vector
            world_vertices.append(world_vertex)
        return world_vertices


    def get_local_vertices(self) -> List[Location]:
        """Return the vertices of the bounding box using the local coordinate system of the object

        Returns
        -------
        List[Location]
            List of Location objects representing a vertice using the objects local coordinates
        """
        rear_bottom_left: Location = Location(self.center.x - self.dimension.length/2, self.center.y + self.dimension.width/2, self.center.z - self.dimension.height/2)
        rear_top_left: Location = Location(self.center.x - self.dimension.length/2, self.center.y + self.dimension.width/2, self.center.z + self.dimension.height/2)
        rear_botton_right: Location = Location(self.center.x - self.dimension.length/2, self.center.y - self.dimension.width/2, self.center.z - self.dimension.height/2)
        rear_top_right: Location = Location(self.center.x - self.dimension.length/2, self.center.y - self.dimension.width/2, self.center.z + self.dimension.height/2)

        front_bottom_left: Location = Location(self.center.x + self.dimension.length/2, self.center.y + self.dimension.width/2, self.center.z - self.dimension.height/2)
        front_top_left: Location = Location(self.center.x + self.dimension.length/2, self.center.y + self.dimension.width/2, self.center.z + self.dimension.height/2)
        front_bottom_right: Location = Location(self.center.x + self.dimension.length/2, self.center.y - self.dimension.width/2, self.center.z - self.dimension.height/2)
        front_top_right: Location = Location(self.center.x + self.dimension.length/2, self.center.y - self.dimension.width/2, self.center.z + self.dimension.height/2)
        local_vertices = [
            rear_bottom_left,
            rear_top_left,
            rear_botton_right,
            rear_top_right,
            front_bottom_left,
            front_top_left,
            front_bottom_right,
            front_top_right
        ]

        return local_vertices


@dataclass()
class EntityObject:
    """A vehicle type, pedestrian type or miscellaneous object type

     Attributes
     ----------
     name : str
        Name of the Entity
    mass : Optional[float]
        The mass of the object in kg
    position : Position
        Position object of the Entity
    bounding_box : BoundingBox
        The three dimensional bounding box that encloses the vehicle
    """
    name: str = ""
    mass: Optional[float] = None
    position: Position = Position()
    bounding_box: BoundingBox = BoundingBox()

    def get_location(self) -> Location:
        """Returns the location of the entity in world coordinates.

        Returns
        -------
        Location
            Location of the entity in world coordinates
        """
        return self.position.world_position.location

    def get_orientation(self) -> Orientation:
        """Returns the orientation of the entity in world coordinates.

        Returns
        -------
        Orientation
            Orientation of the entity in world coordinates
        """
        return self.position.world_position.orientation

    def get_world_position(self) -> WorldPosition:
        """Returns the world position of the entity in world coordinates.

        Returns
        -------
        WorldPosition
            WorldPosition of the entity in world coordinates
        """
        return self.position.world_position

@dataclass()
class PedestrianCategory(Enum):
    PEDESTRIAN = 1
    WHEELCHAIR = 2
    ANIMAL = 3


@dataclass()
class Pedestrian(EntityObject):
    pedestrian_category: PedestrianCategory = None
    kinematic_status: "KinematicStatus" = None


@dataclass()
class MiscObjectCategory(Enum):
    BARRIER = 1
    BUILDING = 2
    CROSSWALK = 3
    GANTRY = 4
    OBSTACLE = 5
    PARKING_SPACE = 6
    PATCH = 7
    POLE = 8
    RAILING = 9
    ROAD_MARK = 10
    SOUND_BARRIER = 11
    STREET_LAMP = 12
    TRAFFIC_ISLAND = 13
    TREE = 14
    VEGETATION = 15
    NONE = 16


@dataclass()
class MiscObject(EntityObject):
    category: MiscObjectCategory = MiscObjectCategory.NONE
