#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
class VehicleDependentBNId(object):
    def __init__(self, wrapped_vehicle, bn_id):
        self.wrapped_vehicle = wrapped_vehicle
        self.bn_id = bn_id
