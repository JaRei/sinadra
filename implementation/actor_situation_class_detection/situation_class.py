#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
from abc import ABC, abstractmethod
from enum import Enum
from shapely.geometry import Point, Polygon, LineString

from actor_situation_class_detection.vehicle_actor_wrapper import VehiclePointId, VehicleActorWrapper
from typing import List, Dict

import carla


class SituationClassType(Enum):
    FOLLOWING_LANE_1_LANE = 1
    FOLLOWING_LANE_2_LANES = 2
    MERGING_LANES = 3
    SIGNALIZED_T_CROSSING_2_LANES = 4
    UNSIGNALIZED_T_CROSSING_2_LANES = 5
    SIGNALIZED_4_WAY_CROSSING = 6
    UNSIGNALIZED_4_WAY_CROSSING = 7
    MISCELLANEOUS = 8


class SituationClassStageTypes(Enum):
    ENTER_SITUATION_CLASS = 1
    DEFAULT_STAGE = 2
    LEAVE_SITUATION_CLASS = 3
    NONE = 4


class RoadSegmentEnd(Enum):
    A = 1
    B = 2
    C = 3
    D = 4


class SituationClassStage(object):
    """Can be used used to further divide a situation class into several stages, where each of the stage knows the
    previous and next stages. """
    def __init__(self,
                 situation_class_stage_type: SituationClassType,
                 previous_stages: List["SituationClassStage"],
                 next_stages: List["SituationClassStage"]):
        self.type = situation_class_stage_type
        self.previous_stages = previous_stages
        self.next_stages = next_stages


class SituationClass(ABC):
    """Super class of all specified situation classes. A situation class instance is defined for a certain area in a
    town and knows which vehicles are currently in this area. A situation class can also be aware of which neighbor
    situation classes are specified as next and previous situation classes for each road segment end of this
    situation class

    Attributes
    ----------
    situation_type: SituationClassType
        Specifies the type of this situation class.
    id: str
        Identifier of the situation class
    wrapped_actors_in_situation_class: List[VehicleActorWrapper]
        A list of all Carla vehicles that are currently in the situation class wrapped into VehicleActorWrapper objects
    possible_next_situation_classes_by_road_segment_end: Dict[RoadSegmentEnd, SituationClass]
        Dictionary of possible next situation classes categorized by a road segment end object as the dictionary key
    previous_situation_classes_by_road_segment_end: Dict["RoadSegmentEnd", "SituationClass"]
        Dictionary of previous situation classes categorized by a road segment end object as the dictionary key
    """
    def __init__(self, id: str, coordinates: List[float], situation_type: SituationClassType):
        """Constructor for the base class SituationClass

        Parameters
        ----------
        id: str
            Identifier string for situation class
        coordinates: List[float]
            Corner point coordinates (X and Y coordinate) of the area the situation class shall be defined for.
        situation_type: SituationClassType
            A SituationClassType object that specifies the type of this situation class.
        """
        self._area = Polygon(coordinates)
        self.situation_type = situation_type
        self.id = id
        self.wrapped_actors_in_situation_class = []
        self.possible_next_situation_classes_by_road_segment_end: Dict["RoadSegmentEnd", "SituationClass"] = {}
        self.previous_situation_classes_by_road_segment_end: Dict["RoadSegmentEnd", "SituationClass"] = {}

    @abstractmethod
    def update_wrapped_actor_association(self, wrapped_actor: VehicleActorWrapper):
        """Associates the passed wrapped actor to this situation class. The association has to be implemented in the
        specific situation class class definition, as each situation class could associate the wrapped actor to it in
        a different way.

        Parameters
        ----------
        wrapped_actor: VehicleActorWrapper
            The carla vehicle actor wrapped into a VehicleActorWrapper object that shall be associated to the
            situation class.
        """
        pass

    @abstractmethod
    def clear_actor_associations(self):
        """Removes all associations to wrapped vehicle actors. This method is abstract and has to be implemented in the
        specific situation class class definition, as each situation class could handle the vehicle association
        differently."""
        pass

    @abstractmethod
    def print_information(self, wrapped_actor: VehicleActorWrapper):
        """Prints information to the console. This method is abstract and has to be implemented in teh specific
        situation class class definition for providing situation class type specific information. """
        pass

    def set_possible_next_situation_classes_by_road_segment_end(self,
                                                                possible_next_situation_classes_by_road_segment_end: Dict["RoadSegmentEnd", "SituationClass"]):
        """Sets the situations class public attribute possible_next_situation_classes_by_road_segment_end to passed
        parameter value

        Parameters
        ----------
        possible_next_situation_classes_by_road_segment_end: Dict[RoadSegmentEnd, SituationClass]
            Dictionary of possible next situation classes categorized by a road segment end object as the dictionary key
        """
        self.possible_next_situation_classes_by_road_segment_end = possible_next_situation_classes_by_road_segment_end

    def set_previous_situation_classes_by_road_segment_end(self,
                                                           previous_situation_classes_by_road_segment_end: Dict["RoadSegmentEnd", "SituationClass"]):
        """Sets the situations class public attribute previous_situation_classes_by_road_segment_end to passed
        parameter value

        Parameters
        ----------
        previous_situation_classes_by_road_segment_end: Dict["RoadSegmentEnd", "SituationClass"]
            Dictionary of previous situation classes categorized by a road segment end object as the dictionary key
        """
        self.previous_situation_classes_by_road_segment_end = previous_situation_classes_by_road_segment_end

    def contains_location(self, actor_location: carla.Vector3D) -> bool:
        """Determines if given actor_location is within the area of this situation class. Therefore the
        carla.Vector3D object is used to create a two dimensional point with the location's X and Y coordinates,
        which is further used to check if its within the situation class' area.

        Parameters
        ----------
        actor_location: carla.Vector3D
            A carla.Vector3D object that represents the location that is checked if it's in the situation class' area.
        Returns
        -------
        bool
            True if actor location is within situation class area.
            False if actor location is not within situation class area.
        """
        actor_location_point = Point(actor_location.x, actor_location.y)
        return actor_location_point.within(self._area)

    # add actor to self.actors_in_situation_class list if not already containing
    def _add_wrapped_actor_to_list(self, wrapped_actor: VehicleActorWrapper):
        if not any(wrapped_actor_in_list.vehicle.id == wrapped_actor.vehicle.id for wrapped_actor_in_list in
                   self.wrapped_actors_in_situation_class):
            self.wrapped_actors_in_situation_class.append(wrapped_actor)
        pass

    def _remove_all_wrapped_actors(self):
        self.wrapped_actors_in_situation_class.clear()
        pass


class LaneIdentifier(Enum):
    LEFT_LANE = 1
    RIGHT_LANE = 2
    NOT_SPECIFIED = 3


'''
    Road ends of 2 lane following situation classes

    Incoming end = RoadSegmentEnd.A
    Outgoing end = RoadSegmentEnd.B
'''


class TwoLaneFollowingSituationClass(SituationClass):
    """Situation class for a two lane following scenario that inherits from SituationClass. Additional to the super
    class' attributes, a TwoLaneFollowingSituationClass instance also provides information about which vehicle actors
    are located on the left and right lane.

    Attributes
    ----------
    wrapped_actors_on_left_lane: List[VehicleActorWrapper]
        Wrapped vehicle actors that are on the left lane
    wrapped_actors_on_right_lane: List[VehicleActorWrapper]
        Wrapped vehicle actors that are on the right lane
    """

    def __init__(
            self,
            id: str,
            situation_class_polygon_coordinates: List[float],
            left_lane_polygon_coordinates: List[float],
            right_lane_polygon_coordinates: List[float]):
        """Constructor for TwoLaneFollowingSituationClass

        Parameters
        ----------
        id: str
            Identifier of situation class. Parameter is passed to the constructor of the base class SituationClass
        situation_class_polygon_coordinates: List[float]
            Corner point coordinates (X and Y coordinate) of the area the situation class shall be defined for.
            Parameter is passed to the constructor of the base class SituationClass
        left_lane_polygon_coordinates: List[float]
            Corner point coordinates (X and Y coordinate) of the left lane area.
        right_lane_polygon_coordinates: List[float]
            Corner point coordinates (X and Y coordinate) of the right lane area.
        """

        self._left_lane_polygon = Polygon(left_lane_polygon_coordinates)
        self._right_lane_polygon = Polygon(right_lane_polygon_coordinates)

        self.wrapped_actors_on_left_lane = []
        self.wrapped_actors_on_right_lane = []

        sit_class_type = SituationClassType.FOLLOWING_LANE_2_LANES
        super().__init__(id, situation_class_polygon_coordinates, sit_class_type)

    def update_wrapped_actor_association(self, wrapped_actor: VehicleActorWrapper):
        """Implements method from base class SituationClass: Passed wrapped actor is put into list containing
        wrapped actors that are currently in the situation class. Additionally the wrapped actor is also added to the
        respective list according to which lane the actor is currently located on.

        Parameters
        ----------
        wrapped_actor: VehicleActorWrapper
            VehicleActorWrapper object that is associated to the two lane following situation class.
        """
        self._add_wrapped_actor_to_list(wrapped_actor)
        actor_lane = self.get_actor_lane(wrapped_actor)

        if actor_lane == LaneIdentifier.LEFT_LANE and not any(
                left_lane_actor.vehicle.id == wrapped_actor.vehicle.id for left_lane_actor in
                self.wrapped_actors_on_left_lane):
            self.wrapped_actors_on_left_lane.append(wrapped_actor)
        elif actor_lane == LaneIdentifier.RIGHT_LANE and not any(
                right_lane_actor.vehicle.id == wrapped_actor.vehicle.id for right_lane_actor in
                self.wrapped_actors_on_right_lane):
            self.wrapped_actors_on_right_lane.append(wrapped_actor)
        pass

    def clear_actor_associations(self):
        """Implements method from base class SituationClass: All wrapped actor associations are removed from this
        situation class. This affects the common list of all wrapped actors within the situation class as well as the
        lists that contain the wrapped actors on the left and the right lane.
        """
        self.wrapped_actors_in_situation_class.clear()
        self.wrapped_actors_on_left_lane.clear()
        self.wrapped_actors_on_right_lane.clear()
        pass

    def print_information(self, wrapped_actor: VehicleActorWrapper):
        """Implements method from base class SituationClass: Prints information related to the passed vehicle wrapper
        and the  two lane following situation class. If passed vehicle is not in this situation class, a respective
        information is printed on the console. If the passed vehicle is within the situation, this method prints
        information about the vehicles state within this situation class, i.e. on which lane the vehicle is located on.

        Parameters
        ----------
        wrapped_actor: VehicleActorWrapper
            VehicleActorWrapper object, which the method prints information for.
        """
        if not any(wa.id == wrapped_actor.id for wa in self.wrapped_actors_in_situation_class):
            print(
                f'Vehicle {wrapped_actor.id} with role name = {wrapped_actor.role_name} is NOT in situation class {self.situation_type.name}')
            return

        print(
            f'\nVehicle {wrapped_actor.id} with role name = {wrapped_actor.role_name} is in situation class {self.situation_type.name}')
        print(f'Vehicle is on lane {self.get_actor_lane(wrapped_actor)}')
        pass

    def get_actor_lane(self, wrapped_actor: VehicleActorWrapper) -> LaneIdentifier:
        """Checks, which lane the passed vehicle is on. If the vehicle is not in this situation class or it could not
        determined which lane the vehicle is on precisely, the method returns the lane identifier NOT_SPECIFIED.

        Parameters
        ----------
        wrapped_actor: VehicleActorWrapper
            VehicleActorWrapper object the lane check is performed for.

        Returns
        -------
        LaneIdentifier
            LaneIdentifier enum value. Possible values can be LEFT_LANE, RIGHT_LANE or NOT_SPECIFIED
        """
        actor_location = wrapped_actor.vehicle_points[VehiclePointId.FRONT_MID_CENTER]
        return self.get_lane_id_by_location(actor_location)

    def get_lane_id_by_location(self, location):
        location_2d_point = Point(location.x, location.y)
        if location_2d_point.within(self._left_lane_polygon):
            return LaneIdentifier.LEFT_LANE
        elif location_2d_point.within(self._right_lane_polygon):
            return LaneIdentifier.RIGHT_LANE
        else:
            return LaneIdentifier.NOT_SPECIFIED

    pass


'''
    Road ends of UnsignalizedFourWayJunction situation classes

     __|  A  |__
      D       B
     __       __
       |  C  |
'''


class UnsignalizedFourWayJunction(SituationClass):

    def __init__(self, id, situation_class_polygon_coordinates):
        situation_class_type = SituationClassType.UNSIGNALIZED_4_WAY_CROSSING
        super().__init__(id, situation_class_polygon_coordinates, situation_class_type)

    # implements method from class SituationClass
    def clear_actor_associations(self):
        self.wrapped_actors_in_situation_class.clear()
        pass

    # implements method from class SituationClass
    def update_wrapped_actor_association(self, wrapped_actor):
        self.wrapped_actors_in_situation_class.append(wrapped_actor)
        pass

    # implements method from class SituationClass
    def print_information(self, wrapped_actor):
        if not any(wa.id == wrapped_actor for wa in self.wrapped_actors_in_situation_class):
            print(
                f'Vehicle {wrapped_actor.id} with role name = {wrapped_actor.role_name} is NOT in situation class {self.situation_type.name}')
            return

        print(
            f'Vehicle {wrapped_actor.id} with role name = {wrapped_actor.role_name} is in situation class {self.situation_type.name}\n')
        pass
