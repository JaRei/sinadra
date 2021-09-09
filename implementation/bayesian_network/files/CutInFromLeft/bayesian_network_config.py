#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
class BayesianNetworkConfig:
    """Bayesian network config to change the node's outcomes dependent
    on the received DRA data.
    """

    # ===== Initialization methods =====================================

    def __init__(self) -> None:
        """Initialize the (simulator's) DRA data."""
        self.dra_data: "SINADRARiskSensorData" = None

    # ===== Update methods =============================================

    def update_dra_data(self, dra_data: "SINADRARiskSensorData") -> None:
        """Updates the saved DRA input feature data class instance.

        Parameters
        ----------
        dra_data : SINADRARiskSensorData
            Latest DRA input feature data class instance for the Bayesian network.
        """
        self.dra_data = dra_data

    # ===== Bayesian network's node methods ============================

    def node_Lane_Ends(self):
        """Method for the CPT node 'Lane_Ends' that allows setting the node as output node, 
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys 
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_small = 0.0
        outcome_medium = 0.0
        outcome_no_lane_end = 0.0
        '''----------Manual edit begin----------'''
        speed = self.dra_data.vehicle_speed
        lower_threshold_medium = 5 * speed
        upper_threshold_medium = 10 * speed

        if upper_threshold_medium <= self.dra_data.distance_to_lane_end:
            outcome_no_lane_end = 1.0
        elif lower_threshold_medium <= self.dra_data.distance_to_lane_end < upper_threshold_medium:
            outcome_medium = 1.0
        elif self.dra_data.distance_to_lane_end < lower_threshold_medium:
            outcome_small = 1.0
        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'small': outcome_small,
            'medium': outcome_medium,
            'no_lane_end': outcome_no_lane_end}

    def node_Lane_Change_Intent(self):
        """Method for the CPT node 'Lane_Change_Intent' that allows setting the node as output node, 
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys 
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_LC_Required = 0.0
        outcome_LC_Not_Required = 0.0
        '''----------Manual edit begin----------'''

        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'LC_Required': outcome_LC_Required,
            'LC_Not_Required': outcome_LC_Not_Required}

    def node_Gap_Availability(self):
        """Method for the CPT node 'Gap_Availability' that allows setting the node as output node, 
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys 
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_lt_SV_length = 0.0
        outcome_gt_SV_length = 0.0
        outcome_gt_4_secs = 0.0
        '''----------Manual edit begin----------'''
        # only one vehicle on the right lane at the location
        if self.dra_data.right_cut_in_gap_size == -1:
            outcome_gt_4_secs = 1.0
        # vehicle directly besides
        elif self.dra_data.right_cut_in_gap_size == 0:
            outcome_lt_SV_length = 1.0
        # two vehicles with a gap at the location
        elif self.dra_data.vehicle_speed > 0:
            vehicle_length_gap = self.dra_data.vehicle_length / self.dra_data.vehicle_speed
            current_gap = self.dra_data.right_cut_in_gap_size / self.dra_data.vehicle_speed
            threshold_4_sec_gap = 4 * self.dra_data.vehicle_speed

            if current_gap <= vehicle_length_gap:
                outcome_lt_SV_length = 1.0
            elif vehicle_length_gap < current_gap <= threshold_4_sec_gap:
                outcome_gt_SV_length = 1.0
            elif threshold_4_sec_gap < current_gap:
                outcome_gt_4_secs = 1.0
        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'lt_SV_length': outcome_lt_SV_length,
            'gt_SV_length': outcome_gt_SV_length,
            'gt_4_secs': outcome_gt_4_secs}

    def node_Gap_Acceptance(self):
        """Method for the CPT node 'Gap_Acceptance' that allows setting the node as output node, 
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys 
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_Acceptable = 0.0
        outcome_Unacceptable = 0.0
        '''----------Manual edit begin----------'''

        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'Acceptable': outcome_Acceptable,
            'Unacceptable': outcome_Unacceptable}

    def node_HeavyRain(self):
        """Method for the CPT node 'HeavyRain' that allows setting the node as output node, 
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys 
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_Yes = 0.0
        outcome_No = 0.0
        '''----------Manual edit begin----------'''
        if self.dra_data.is_raining_heavy:
            outcome_Yes = 1.0
        else:
            outcome_No = 1.0
        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'Yes': outcome_Yes,
            'No': outcome_No}

    def node_FVViewRange(self):
        """Method for the CPT node 'FVViewRange' that allows setting the node as output node, 
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys 
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_Restricted = 0.0
        outcome_Clear = 0.0
        '''----------Manual edit begin----------'''

        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'Restricted': outcome_Restricted,
            'Clear': outcome_Clear}

    def node_SV_Situation_Perception(self):
        """Method for the CPT node 'SV_Situation_Perception' that allows setting the node as output node, 
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys 
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_Incorrect = 0.0
        outcome_Correct = 0.0
        '''----------Manual edit begin----------'''

        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'Incorrect': outcome_Incorrect,
            'Correct': outcome_Correct}

    def node_SV_Situation_Assessment(self):
        """Method for the CPT node 'SV_Situation_Assessment' that allows setting the node as output node, 
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys 
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_Incorrect = 0.0
        outcome_Correct = 0.0
        '''----------Manual edit begin----------'''

        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'Incorrect': outcome_Incorrect,
            'Correct': outcome_Correct}

    def node_SV_Maneuver_Decision(self):
        """Method for the CPT node 'SV_Maneuver_Decision' that allows setting the node as output node, 
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys 
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_Safe_Cutin = 0.0
        outcome_Unsafe_Cutin = 0.0
        outcome_No_Cutin = 0.0
        '''----------Manual edit begin----------'''

        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'Safe_Cutin': outcome_Safe_Cutin,
            'Unsafe_Cutin': outcome_Unsafe_Cutin,
            'No_Cutin': outcome_No_Cutin}

    def node_SV_Cutin_Behavior(self):
        """Method for the CPT node 'SV_Cutin_Behavior' that allows setting the node as output node, 
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys 
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_CutIn = 0.0
        outcome_NoCutIn = 0.0
        '''----------Manual edit begin----------'''

        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'CutIn': outcome_CutIn,
            'NoCutIn': outcome_NoCutIn}

    def node_Ego_FV_Perception(self):
        """Method for the CPT node 'Ego_FV_Perception' that allows setting the node as output node, 
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys 
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_Obstructed = 0.0
        outcome_Unobstructed = 0.0
        '''----------Manual edit begin----------'''

        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'Obstructed': outcome_Obstructed,
            'Unobstructed': outcome_Unobstructed}

    def node_Predicted_Left_SV_Cut_In_Behavior(self):
        """Method for the CPT node 'Predicted_Left_SV_Cut_In_Behavior' that allows setting the node as output node,
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys 
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_CutIn = 0.0
        outcome_NoCutIn = 0.0
        '''----------Manual edit begin----------'''
        is_bn_output = True
        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'CutIn': outcome_CutIn,
            'NoCutIn': outcome_NoCutIn}

    def node_Steering_Angle(self):
        """Method for the CPT node 'Steering_Angle' that allows setting the node as output node, 
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys 
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_lt_10_deg = 0.0
        outcome_gt_10_deg = 0.0
        '''----------Manual edit begin----------'''
        if self.dra_data.steering_angle_from_lane_following < 10:
            outcome_lt_10_deg = 1.0
        elif 10 <= self.dra_data.steering_angle_from_lane_following:
            outcome_gt_10_deg = 1.0
        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'lt_10_deg': outcome_lt_10_deg,
            'gt_10_deg': outcome_gt_10_deg}

    def node_Turn_Indicator(self):
        """Method for the CPT node 'Turn_Indicator' that allows setting the node as output node, 
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys 
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_Yes = 0.0
        outcome_No = 0.0
        '''----------Manual edit begin----------'''
        if self.dra_data.is_indicating_lane_change:
            outcome_Yes = 1.0
        else:
            outcome_No = 1.0
        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'Yes': outcome_Yes,
            'No': outcome_No}

    def node_Distance_Center(self):
        """Method for the CPT node 'Distance_Center' that allows setting the node as output node, 
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys 
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_MidLane = 0.0
        outcome_BetweenLaneAndCrossing = 0.0
        outcome_RightBeforeCrossing = 0.0
        '''----------Manual edit begin----------'''
        distance_right_lane_line = self.dra_data.distance_from_right_lane_line
        distance_lane_center = self.dra_data.distance_from_lane_center

        # near to the right lane
        if distance_right_lane_line <= 0.5:
            outcome_RightBeforeCrossing = 1.0
        # normal driving
        elif self.dra_data.steering_angle_from_lane_following < 1.5 and distance_lane_center < 0.2:
            outcome_MidLane = 1.0
        # everything in between
        else:
            outcome_BetweenLaneAndCrossing = 1.0
        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'MidLane': outcome_MidLane,
            'BetweenLaneAndCrossing': outcome_BetweenLaneAndCrossing,
            'RightBeforeCrossing': outcome_RightBeforeCrossing}
