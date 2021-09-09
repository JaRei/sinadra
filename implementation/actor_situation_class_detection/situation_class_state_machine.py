#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################

from typing import List, Union, Optional, TYPE_CHECKING

import numpy as np

from actor_situation_class_detection.situation_class import SituationClass
from data_model.vehicle import EgoVehicle, VehiclePointId

if TYPE_CHECKING:
    from data_model.vehicle import OtherVehicle
    from data_model.positions import Location, Orientation


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

    def update_situation_classes(self, vehicles: List[Union["OtherVehicle", EgoVehicle]]):
        """Method to allocate vehicles to their corresponding situation classes they are currently in. Therefore it is
        checked for each vehicle whether the center point of the vehicle's front bumper is within a
        situation class defined for this state machine or not. If a respective situation class has been determined, the
        vehicle is associated to it.

        Parameters
        ----------
        vehicles : Union[EgoVehicle, OtherVehicle]
            All vehicles as EgoVehicle or OtherVehicle objects
        """

        for vehicle in vehicles:
            actor_transform = vehicle.position.world_position
            actor_front_mid_location = vehicle.vehicle_points[VehiclePointId.FRONT_MID_CENTER]
            actor_orientation = actor_transform.orientation
            situation_class_of_actor = self._get_active_situation_class(actor_front_mid_location, actor_orientation)

            if situation_class_of_actor is not None:
                situation_class_of_actor.update_actor_association(vehicle)

    def get_hero_situation_class(self) -> Optional[SituationClass]:
        """Retrieves the situation class the hero vehicle is currently in.

        Returns
        -------
        SituationClass object, which the hero vehicle is currently allocated to. If hero vehicle can not be found in any
        situation class, None is returned instead.
        """

        for situation_class in self.situation_classes:
            actors_in_situation_class = situation_class.actors_in_situation_class
            if any(isinstance(actor, EgoVehicle) for actor in actors_in_situation_class):
                return situation_class
        return None

    def _get_active_situation_class(self, actor_location: "Location", actor_orientation: "Orientation"):
        actor_forward_vector = actor_orientation.get_forward_vector()
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

    def _get_possible_situation_classes(self, actor_location: "Location"):
        possible_situation_classes = []
        for situation_class in self.situation_classes:
            if situation_class.contains_location(actor_location):
                possible_situation_classes.append(situation_class)
        return possible_situation_classes
