#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
from enum import Enum


class BayesianNetId(Enum):
    UNKNOWN = 1,
    TWO_LANE_FOLLOWING_EGO = 2,
    TWO_LANE_FOLLOWING_FRONT_VEHICLE = 3,
    TWO_LANE_FOLLOWING_SIDE_VEHICLE_ON_OTHER_LANE_LEFT = 4,
    TWO_LANE_FOLLOWING_SIDE_VEHICLE_ON_OTHER_LANE_RIGHT = 5,
    TWO_LANE_FOLLOWING_FRONT_VEHICLE_IN_LANE_CHANGE = 6
