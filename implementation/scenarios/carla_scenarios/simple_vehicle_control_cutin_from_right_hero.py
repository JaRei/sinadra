#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################

#!/usr/bin/env python

# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides an example control for vehicles which
does not use CARLA's vehicle engine.

Limitations:
- Does not respect any traffic regulation: speed limit, traffic light, priorities, etc.
- Can only consider obstacles in forward facing reaching (i.e. in tight corners obstacles may be ignored).
"""


import carla

from simple_vehicle_control_adapted import SimpleVehicleControlAdapted


class SimpleVehicleControlCutinFromRightHero(SimpleVehicleControlAdapted):

    def run_step(self):
        super(SimpleVehicleControlCutinFromRightHero, self).run_step()

        # Hardcoded location trigger for right indicator (lane following cutin hero scenario, Town03)
        if -60.0 < self._actor.get_location().y < -25.0:
            self._set_vehicle_light_state(carla.VehicleLightState.LeftBlinker)
