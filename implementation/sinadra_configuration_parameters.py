#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
# This file contains parameters that can be changed to configure the sinadra risk sensor accordingly

from actor_situation_class_detection.bayesian_network_id_selection.bayesian_network_id import BayesianNetId

from enum import Enum
from typing import Dict, Tuple, Optional


####################################
# SINADRA client configuration
####################################

# Framerate of the risk plot refresh rate and the SINADRA client in
# general (base rate for the risk computation).
# Used for setting up the simulators as well.
FRAMERATE: int = 20
SAVE_EVALUATION_DATA = False

####################################
# Simulator client configuration
####################################

CARLA_SERVER_HOST: str = 'localhost'
CARLA_SERVER_PORT: int = 2000

####################################
# Simulator debugging
####################################

# DEBUG_MODE:
# Set DEBUG_MODE true to draw sensing areas if the respective simulator does support it.
# If value = False the sinadra risk sensor is still working but debug lines are not drawn.
DEBUG_MODE: bool = False

DEBUG_LINE_THICKNESS: float = 0.1  # float value in meters
DEBUG_DRAWING_LIFE_TIME: float = 1 / FRAMERATE  # value in seconds

Color = Tuple[int, int, int]
DEBUG_COLOR_SENSING_AREA: Color = (51, 255, 204)  # green
DEBUG_COLOR_SENSING_AREA_WITH_SENSED_OBJECT: Color = (255, 119, 0)  # orange
DEBUG_COLOR_DEBUG_LINE_VEHICLE_REAR_END: Color = (255, 0, 0)  # red

####################################
# Two lane following situation class
####################################

FRONT_SENSING_DISTANCE_TIME_GAP_IN_SECONDS: float = 2

# Angle between forward vectors of front vehicle and way point on
# same lane. If actual angle is greater than this parameter, front vehicle gets allocated "UNKNOWN" as BN_ID
ACCEPTABLE_DEVIATING_ANGLE_FOR_FRONT_VEHICLE: float = 10.0

# distance between way points for calculating sensing areas
SENSING_AREA_WAYPOINT_DISTANCE: float = 1.0

# length of side sensing area.
# SIDE_SENSING_AREA_LENGTH / 2 = length of sensing area to both the front and the back
SIDE_SENSING_AREA_LENGTH: float = 15.0

ACCEPTABLE_DISTANCE_FROM_LANE_CENTER_FOR_FRONT_VEHICLE: float = 1.75  # distance from lane center in meters

####################################
# BN Inference parameters
####################################

# Mapping of the ID/state of a vehicle in a situation class to the corresponding Bayesian network ID/name (= name of
# the folder with the BN files).
# If there shall be no Bayesian inference for a specific vehicle state, "None" as the Bayesian network ID is supported.
VEHICLE_SITUATION_STATE_TO_BAYESIAN_NETWORK: Dict[BayesianNetId, Optional[str]] = {
    BayesianNetId.TWO_LANE_FOLLOWING_FRONT_VEHICLE: "1LaneFollow_Simple",
    BayesianNetId.TWO_LANE_FOLLOWING_FRONT_VEHICLE_IN_LANE_CHANGE: "1LaneFollow_Simple",
    BayesianNetId.TWO_LANE_FOLLOWING_SIDE_VEHICLE_ON_OTHER_LANE_LEFT: "CutInFromLeft",
    BayesianNetId.TWO_LANE_FOLLOWING_SIDE_VEHICLE_ON_OTHER_LANE_RIGHT: "CutInFromRight",
    BayesianNetId.TWO_LANE_FOLLOWING_EGO: None,
    BayesianNetId.UNKNOWN: None
}

# Defines how many processes shall be used for the multiprocessing of the Bayesian network inference.
# (The default value is 2 (processes) if this variable is set to "None".)
NUMBER_OF_PROCESSES_FOR_THE_BN_INFERENCE: Optional[int] = None

# Number of vehicles with the state LANE_FOLLOWING_FRONT_VEHICLE in front of the ego vehicle that shall be considered
# in the Bayesian network inference.
# (In case of the value "None" all classified vehicles will be considered.)
NUM_INTERACTION_HOPS: Optional[int] = 1

####################################
# BN & Risk Computation parameters
####################################

class BehaviorType(Enum):
    Braking = 1
    LaneChangeToLeft = 2
    LaneChangeToRight = 3

# Behavior Type Mapping relates the output node names of the Bayesian Networks with a particular behavior type
# This is required so that the trajectory sampling algorithm knows, which behavior types to generate behaviors for
BEHAVIOR_TYPE_MAPPING = {
    "Predicted_FV_Braking_Behavior": BehaviorType.Braking,
    "Predicted_Left_SV_Cut_In_Behavior": BehaviorType.LaneChangeToRight,
    "Predicted_Right_SV_Cut_In_Behavior": BehaviorType.LaneChangeToLeft
}

SKIP_CYCLE_COUNT: int = 4  # Performance param: Only every n-th game loop cycle, the risk computation is executed
PREDICTION_HORIZON = 4  # [seconds]: Future time, until which the risk computation is performed
PREDICTION_TIMESTEP = 0.2  # [seconds]
NUM_TRAJECTORIES = 20  # Number of trajectories being sampled for each behavior in every time step

# Eggert Params
EGGERT_BETA = 1
EGGERT_RATE_MAX = 1

####################################
# Trajectory generation: Emergency Brake
####################################
# Mean/Standard deviation of constant acceleration of emergency braking behavior (Gaussian Distribution)
EMERGENCY_ACC_MEAN = -6.0  # [m/s²]
EMERGENCY_ACC_STD = 0.7  # [m/s²]

# Static position uncertainty to model measurement uncertainties of position
EMERGENCY_POS_STD = 0.2  # [m]

####################################
# Trajectory generation: Constant acceleration
####################################
# Mean/Standard deviation of constant acceleration (Gaussian Distribution)
CONST_ACCEL_MEAN = 0.0  # [m/s²]
CONST_ACCEL_STD = 0.01  # [m/s²]

# Static position uncertainty to model measurement uncertainties of position
CONST_ACCEL_POS_STD = 0.2  # [m]


####################################
# Trajectory generation: Target Brake
####################################
BRAKE_TARGET_SAFE_DISTANCE_MARGIN = 1.5  # meter before braking target to usually come to stop

# Static position uncertainty to model measurement uncertainties of position
TB_POS_STD = 0.2  # [m]

TB_MAX_DECELERATION = -8.0  # Maximum allowed deceleration [m/s²]

####################################
# Trajectory generation: IDM
####################################
# No vehicle in front of the front vehicle exists in groundtruth.
# Since the BN predicts a non-zero prob of follow vehicle behavior due to
# uncertainties, we assume a vehicle to be in front with a time gap of 1,5s
# and same speed like FV for the trajectory generation.
IDM_TIME_GAP_FRONT_VEHICLE = 1.0

# Standard deviation of time gap to model probabilistic differences in accepted time gap
IDM_TIMEGAP_STD = 0.05

# Static position uncertainty to model measurement uncertainties of position
IDM_POS_STD = 0.2  # [m]

# Intelligent Driver Model (IDM) formula, e.g. https://en.wikipedia.org/wiki/Intelligent_driver_model

# Minimum spacing: a minimum desired net distance.
# A car can't move if the distance from the car in the front is not at least S_0
S_0 = 2.0  # [m]
# desired velocity V_DESIRED: the velocity the vehicle would drive at in free traffic
# TODO to be made dynamic based on the traffic situation --> e.g. speed limit
V_DESIRED = 50 / 3.6  # [m/s]
DELTA = 4  # Typically chosen value according to Wikipedia
# acceleration A_MAX: the maximum vehicle acceleration
A_MAX = 0.75  # [m/s²]
# comfortable braking deceleration B_COMFORT: a positive number
B_COMFORT = 1.5  # [m/s²]

####################################
# Trajectory generation: Lane Change
####################################
# Assumed Standard Deviation for ego lateral position [m]
EGO_POS_LAT_STD = 0.2

# Assumption that cut-in will end at x m in front of ego (=end point of bezier curve)
LC_CUTIN_DISTANCE_FROM_EGO = 10

# Gaussian standard deviation to model variation in endpoints for lane change behavior
LC_ENDPOINT_VARIATION_STD = 1.0  # [m]

