#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
from sinadra_configuration_parameters import VEHICLE_SITUATION_STATE_TO_BAYESIAN_NETWORK
from typing import Dict, List, TYPE_CHECKING, Union
import importlib

if TYPE_CHECKING:
    from bayesian_network.inference.interfaces import BayesianNetworkData, BayesianNetworkInputFeatureData
    from bayesian_network.inference.sinadra_risk_sensor_data import SINADRARiskSensorData
    from data_model.vehicle import EgoVehicle, OtherVehicle


class RiskSensorDataBuilder:
    """Provides usability to build the Bayesian network input feature interfaces for several vehicles based on input
    from the CARLA simulator.
    """

    def __init__(self) -> None:
        """Initializes the object and sets the package location of the SINADRA risk sensor data interface classes."""
        self._risk_data_module = importlib.import_module(".sinadra_risk_sensor_data",
                                                         package="bayesian_network.inference")

    def collect_carla_data_and_build_risk_sensor_data(self, bayesian_network_data: List["BayesianNetworkData"],
                                                      carla_input_feature_data:
                                                      List["BayesianNetworkInputFeatureData"],
                                                      all_vehicles: Union["EgoVehicle", "OtherVehicle"]
                                                      ) -> List["BayesianNetworkData"]:
        """Builds the correct SINADRA risk sensor data input feature data interface for each Bayesian network for each 
        vehicle. Uses the CARLA input feature data for the feature extraction and adds the built and updated SINADRA 
        risk sensor data input feature instances to the Bayesian network data instance of the corresponding vehicle.
        
        Parameters
        ----------
        bayesian_network_data : List[BayesianNetworkData]
            List of the Bayesian network data for all vehicles. Used for the mapping from vehicle states to Bayesian 
            networks each.
        carla_input_feature_data : List[BayesianNetworkInputFeatureData]
            CARLA input data for each vehicle which is used for extracting the situation-specific Bayesian network input 
            features.

        Returns
        -------
        List[BayesianNetworkData]
            List of the updated Bayesian network data instances for each vehicle. In case of a recognized Bayesian 
            network for a given vehicle state (in a situation class) the SINADRA risk sensor input feature data 
            instance is added. 
        """
        bayesian_network_data = self._map_vehicle_situation_states_to_bayesian_network_ids(bayesian_network_data)
        carla_input_feature_data = self._add_bayesian_network_ids(carla_input_feature_data, bayesian_network_data)
        vehicle_id_to_risk_data = self._build_risk_sensor_data_instances(carla_input_feature_data, all_vehicles)
        bayesian_network_data = self._add_risk_sensor_data(bayesian_network_data, vehicle_id_to_risk_data)

        return bayesian_network_data

    @staticmethod
    def _map_vehicle_situation_states_to_bayesian_network_ids(bayesian_network_data: List["BayesianNetworkData"]
                                                              ) -> List["BayesianNetworkData"]:
        """Maps the vehicle situation class states to a corresponding Bayesian network identifier.
        Further, extends the given input Bayesian network data objects by the identifier (None value as ID is legit).

        Parameters
        ----------
        bayesian_network_data : List[BayesianNetworkData]
            Input Bayesian network data objects that should be extended by the Bayesian network identifier.

        Returns
        -------
        List[BayesianNetworkData]
            Updated Bayesian network data objects.
        """
        for specific_bn_data in bayesian_network_data:
            vehicle_situation_state_id = specific_bn_data.vehicle_situation_state_id

            if vehicle_situation_state_id in VEHICLE_SITUATION_STATE_TO_BAYESIAN_NETWORK:
                bayesian_network_id = VEHICLE_SITUATION_STATE_TO_BAYESIAN_NETWORK[vehicle_situation_state_id]
            else:
                bayesian_network_id = None

            specific_bn_data.bayesian_network_id = bayesian_network_id

        return bayesian_network_data

    @staticmethod
    def _add_bayesian_network_ids(carla_input_feature_data: List["BayesianNetworkInputFeatureData"],
                                  bayesian_network_data: List["BayesianNetworkData"]
                                  ) -> List["BayesianNetworkInputFeatureData"]:
        """Extracts the Bayesian network IDs for each vehicle from the Bayesian network data objects and adds it to the
        given input CARLA Bayesian network input feature data objects.

        Parameters
        ----------
        carla_input_feature_data : List[BayesianNetworkInputFeatureData]
            CARLA Bayesian network input feature data objects to which the Bayesian network identifier shall be added.
        bayesian_network_data : List[BayesianNetworkData]
            Bayesian network data objects from which the Bayesian network identifier for each vehicle is extracted from.

        Returns
        -------
        List[CARLABayesianNetworkInputFeatureData]
            Given input CARLA Bayesian network input feature data objects to which the Bayesian network identifier is
            added.
        """
        vehicle_id_to_bn_id: Dict[str, str] = {}

        for specific_bn_data in bayesian_network_data:
            vehicle_id = specific_bn_data.vehicle_id
            bayesian_network_id = specific_bn_data.bayesian_network_id
            vehicle_id_to_bn_id[vehicle_id] = bayesian_network_id

        for specific_carla_data in carla_input_feature_data:
            vehicle_id = specific_carla_data.vehicle_id
            bayesian_network_id = vehicle_id_to_bn_id[vehicle_id]
            specific_carla_data.bayesian_network_id = bayesian_network_id

        return carla_input_feature_data

    def _build_risk_sensor_data_instances(self, carla_input_feature_data: List["BayesianNetworkInputFeatureData"],
                                          all_vehicles: List[Union["EgoVehicle", "OtherVehicle"]]
                                          ) -> Dict[str, "SINADRARiskSensorData"]:
        """Builds the SINADRA risk sensor data for each vehicle (that has a Bayesian network associated) in the given
        CARLA Bayesian network input feature data objects.

        Parameters
        ----------
        carla_input_feature_data : List[BayesianNetworkInputFeatureData]
            CARLA Bayesian network input feature data objects for which the risk data is collected and built.

        Returns
        -------
        Dict[str, SINADRARiskSensorData]
            Dictionary with the vehicle ID as key and the corresponding SINADRA risk sensor data object as value.
        """
        vehicle_id_to_risk_data: Dict[str, "SINADRARiskSensorData"] = {}

        for specific_carla_data in carla_input_feature_data:
            bayesian_network_id = specific_carla_data.bayesian_network_id

            if bayesian_network_id:
                starts_with_number = str.isnumeric(bayesian_network_id[0])
                corrected_bn_id = f"Data{bayesian_network_id}" if starts_with_number else bayesian_network_id
                corrected_bn_id = corrected_bn_id.replace(" ", "")
                risk_data_class = getattr(self._risk_data_module, corrected_bn_id)
                risk_data_instance = risk_data_class()
                risk_data_instance.collect_data_from_carla(specific_carla_data, all_vehicles)
                vehicle_id = specific_carla_data.vehicle_id
                vehicle_id_to_risk_data[vehicle_id] = risk_data_instance

        return vehicle_id_to_risk_data

    @staticmethod
    def _add_risk_sensor_data(bayesian_network_data: List["BayesianNetworkData"],
                              vehicle_id_to_risk_data: Dict[str, "SINADRARiskSensorData"]
                              ) -> List["BayesianNetworkData"]:
        """Extends the given input Bayesian network data objects by the SINADRA risk sensor data for these vehicles
        that have a Bayesian network associated.

        Parameters
        ----------
        bayesian_network_data : List[BayesianNetworkData]
            Input Bayesian network data objects that should be extended by the corresponding SINADRA risk sensor data.
        vehicle_id_to_risk_data : Dict[str, SINADRARiskSensorData]
            Dictionary with the vehicle ID as key and the corresponding SINADRA risk sensor data object as value.

        Returns
        -------
        List[BayesianNetworkData]
            Updated Bayesian network data objects.
        """
        for specific_bn_data in bayesian_network_data:
            vehicle_id = specific_bn_data.vehicle_id

            if vehicle_id in vehicle_id_to_risk_data:
                risk_data = vehicle_id_to_risk_data[vehicle_id]
                specific_bn_data.sinadra_risk_sensor_data = risk_data

        return bayesian_network_data
