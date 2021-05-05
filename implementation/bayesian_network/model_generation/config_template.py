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
