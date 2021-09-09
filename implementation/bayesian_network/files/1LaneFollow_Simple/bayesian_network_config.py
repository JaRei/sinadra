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

    def node_TTC(self):
        """Method for the CPT node 'TTC' that allows setting the node as output node,
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_Critical = 0.0
        outcome_Medium = 0.0
        outcome_High = 0.0
        '''----------Manual edit begin----------'''

        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'Critical': outcome_Critical,
            'Medium': outcome_Medium,
            'High': outcome_High}

    def node_TTC_Measurement_Uncertainty(self):
        """Method for the CPT node 'TTC_Measurement_Uncertainty' that allows setting the node as output node,
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_High = 0.0
        outcome_Low = 0.0
        '''----------Manual edit begin----------'''

        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'High': outcome_High,
            'Low': outcome_Low}

    def node_Measured_TTC(self):
        """Method for the CPT node 'Measured_TTC' that allows setting the node as output node,
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_Critical = 0.0
        outcome_Medium = 0.0
        outcome_High = 0.0
        '''----------Manual edit begin----------'''
        is_bn_output = True

        if self.dra_data.lead_speed:
            relative_speed = self.dra_data.speed - self.dra_data.lead_speed
            if relative_speed <= 0.0:
                outcome_High = 1.0
            else:
                ttc = self.dra_data.distance_between_vehicles / relative_speed
                if ttc >= 1.5:
                    outcome_Medium = 1.0
                else:
                    outcome_Critical = 1.0
        else:
            outcome_High = 1.0
        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'Critical': outcome_Critical,
            'Medium': outcome_Medium,
            'High': outcome_High}

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
            outcome_No = 0.0
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

    def node_FV_Situation_Perception(self):
        """Method for the CPT node 'FV_Situation_Perception' that allows setting the node as output node,
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

    def node_FV_Situation_Assessment(self):
        """Method for the CPT node 'FV_Situation_Assessment' that allows setting the node as output node,
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

    def node_FV_Maneuver_Decision(self):
        """Method for the CPT node 'FV_Maneuver_Decision' that allows setting the node as output node,
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_Correct = 0.0
        outcome_Incorrect = 0.0
        '''----------Manual edit begin----------'''

        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'Correct': outcome_Correct,
            'Incorrect': outcome_Incorrect}

    def node_FV_Front_Existence(self):
        """Method for the CPT node 'FV_Front_Existence' that allows setting the node as output node,
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
        if self.dra_data.lead_vehicle_exists:
            outcome_Yes = 1.0
        else:
            outcome_No = 1.0
        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'Yes': outcome_Yes,
            'No': outcome_No}

    def node_FV_Braking_Behavior(self):
        """Method for the CPT node 'FV_Braking_Behavior' that allows setting the node as output node,
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_Emergency = 0.0
        outcome_TargetBrake = 0.0
        outcome_FollowVehicle = 0.0
        outcome_NoBrake = 0.0
        '''----------Manual edit begin----------'''

        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'Emergency': outcome_Emergency,
            'TargetBrake': outcome_TargetBrake,
            'FollowVehicle': outcome_FollowVehicle,
            'NoBrake': outcome_NoBrake}

    def node_FVType(self):
        """Method for the CPT node 'FVType' that allows setting the node as output node,
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_Smaller_than_ego = 0.0
        outcome_Taller_than_ego = 0.0
        '''----------Manual edit begin----------'''
        if self.dra_data.leading_vehicle_is_larger:
            outcome_Taller_than_ego = 1.0
        else:
            outcome_Smaller_than_ego = 1.0
        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'Smaller_than_ego': outcome_Smaller_than_ego,
            'Taller_than_ego': outcome_Taller_than_ego}

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

    def node_Predicted_FV_Braking_Behavior(self):
        """Method for the CPT node 'Predicted_FV_Braking_Behavior' that allows setting the node as output node,
        and allows changing this nodes outcomes for the Bayesian network inference.

        Returns
        -------
        Tuple[bool, Dict[str, float]]
            Flag whether this node is an output node, and dictionary with the node outcome IDs as keys
            and the corresponding probabilities as values.
        """
        is_bn_output = False
        outcome_Emergency = 0.0
        outcome_TargetBrake = 0.0
        outcome_FollowVehicle = 0.0
        outcome_NoBrake = 0.0
        '''----------Manual edit begin----------'''
        is_bn_output = True
        '''----------Manual edit end----------'''
        return is_bn_output, \
            {'Emergency': outcome_Emergency,
            'TargetBrake': outcome_TargetBrake,
            'FollowVehicle': outcome_FollowVehicle,
            'NoBrake': outcome_NoBrake}
