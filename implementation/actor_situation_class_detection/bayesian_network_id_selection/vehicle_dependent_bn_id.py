#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################

from typing import TYPE_CHECKING, Union

if TYPE_CHECKING:
    from data_model.vehicle import EgoVehicle, OtherVehicle


class VehicleDependentBNId(object):
    def __init__(self, vehicle: Union["EgoVehicle", "OtherVehicle"], bn_id):
        self.vehicle = vehicle
        self.bn_id = bn_id
