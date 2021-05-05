#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
from typing import List

import numpy as np

from actor_situation_class_detection.situation_class import SituationClass
from actor_situation_class_detection.vehicle_actor_wrapper import VehicleActorWrapper, VehiclePointId


class SituationClassStateMachine(object):
    """State machine class that contains all existing situation classes for a town. It provides methods updating the
    situation classes regarding their actor associations, for clearing the already set actor associations from
    situation classes and for determining the situation class the hero vehicle is currently in
    """
    def __init__(self, situation_classes: List[SituationClass]):
        self.situation_classes = situation_classes
        self.ego_situation_class = None

    def clear_actor_associations_from_situation_classes(self):
        """Method to remove all actors from situation class objects."""
        for situation_class in self.situation_classes:
            situation_class.clear_actor_associations()
            situation_class.ego_is_in_situation_class = False
            pass

    def update_situation_classes(self, wrapped_vehicles: List["VehicleActorWrapper"]):
        """Method to allocate vehicles to their corresponding situation classes they are currently in. Therefore it is
        checked for each wrapped vehicle whether the center point of the wrapped vehicle's front bumper is within a
        situation class defined for this state machine or not. If a respective situation class has been determined, the
        wrapped vehicle is associated to it.

        Parameters
        ----------
        wrapped_vehicles : List[VehicleActorWrapper]
            All carla vehicles wrapped as VehicleActorWrapper objects
        """
        for wrapped_vehicle in wrapped_vehicles:
            actor_transform = wrapped_vehicle.vehicle.get_transform()
            actor_front_mid_location = wrapped_vehicle.vehicle_points[VehiclePointId.FRONT_MID_CENTER]
            actor_rotation = actor_transform.rotation
            situation_class_of_actor = self._get_active_situation_class(actor_front_mid_location, actor_rotation)

            if situation_class_of_actor is not None:
                situation_class_of_actor.update_wrapped_actor_association(wrapped_vehicle)

    def get_hero_situation_class(self) -> SituationClass:
        """Retrieves the situation class the hero vehicle is currently in.

        Returns
        -------
        SituationClass object, which the hero vehicle is currently allocated to. If hero vehicle can not be found in any
        situation class, None is returned instead.
        """
        for situation_class in self.situation_classes:
            wrapped_actors_in_situation_class = situation_class.wrapped_actors_in_situation_class
            if any(wrapped_actor.role_name == "hero" for wrapped_actor in wrapped_actors_in_situation_class):
                return situation_class
        return None

    def _get_active_situation_class(self, actor_location, actor_rotation):
        actor_forward_vector = actor_rotation.get_forward_vector()
        possible_situation_classes = self._get_possible_situation_classes(actor_location)

        if len(possible_situation_classes) == 1:
            return possible_situation_classes[0]

        # if current location is either in
        #   none situation class or in
        #   more than one situation class
        #   -> try a second time with point in future location
        possible_situation_classes.clear()
        next_location = np.add(actor_forward_vector, actor_location)
        possible_situation_classes = self._get_possible_situation_classes(next_location)

        if len(possible_situation_classes) == 1:
            return possible_situation_classes[0]

        return None

    def _get_possible_situation_classes(self, actor_location):
        possible_situation_classes = []
        for situation_class in self.situation_classes:
            if situation_class.contains_location(actor_location):
                possible_situation_classes.append(situation_class)
        return possible_situation_classes
