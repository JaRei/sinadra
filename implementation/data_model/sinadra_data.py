#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################

from dataclasses import dataclass
from typing import TYPE_CHECKING, List, Optional

if TYPE_CHECKING:
    from data_model.vehicle import OtherVehicle, EgoVehicle
    from data_model.environment import Environment
    from data_model.entity import Pedestrian, MiscObject


@dataclass()
class SinadraData():
    hero_vehicle: Optional["EgoVehicle"] = None
    other_vehicles: Optional[List["OtherVehicle"]] = None
    environment: Optional["Environment"] = None
    pedestrians: Optional[List["Pedestrian"]] = None
    misc_objects: Optional[List["MiscObject"]] = None
    map = None
