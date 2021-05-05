#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
from pgmpy.readwrite.XMLBIF import XMLBIFReader
from sinadra_configuration_parameters import VEHICLE_SITUATION_STATE_TO_BAYESIAN_NETWORK
from typing import Type, Dict, Tuple, List, TYPE_CHECKING
import importlib
import sys
import os

if TYPE_CHECKING:
    from pgmpy.models import BayesianModel
    from bayesian_network.model_generation.config_template import BayesianNetworkConfig
    from bayesian_network.inference.interfaces import BayesianNetworkData


class BayesianNetworkFileExtractor:
    """Initially extracts all Bayesian network related files from the disk, instantiates them and provides them to the
    Bayesian network data instances.

    Attributes
    ----------
    file_directory_path : str
        (Class attribute) Absolute path to the files location to easily locate the saved Bayesian network files in the
        "../files/" directory. Extracted using Python's os package.
    """

    file_directory_path = os.path.dirname(os.path.abspath(__file__))

    def __init__(self) -> None:
        """Loads all Bayesian network files on startup and stores them."""
        self._bn_instance_for_network_id: Dict[str, "BayesianModel"] = {}
        self._bn_config_class_for_network_id: Dict[str, Type["BayesianNetworkConfig"]] = {}

        self._path_to_files = f"{BayesianNetworkFileExtractor.file_directory_path}/../files/"
        self._package_to_files = "bayesian_network.files"
        sys.path.append(self._path_to_files)

        self._config_file_name = "bayesian_network_config"
        self._config_class_name = "BayesianNetworkConfig"
        self._bayesian_network_file_name = "bayesian_network.xmlbif"

        self._initial_loading_of_all_networks()

    def get_all_network_instances_for_situation(self, bayesian_network_data: List["BayesianNetworkData"]
                                                ) -> List["BayesianNetworkData"]:
        """Extracts for each Bayesian network data instance, respectively for each vehicle, the corresponding Bayesian
        network model (pgmpy object) and its model configuration Python class.
        The model and its configuration are added to the data instance that is passed through the method.

        Parameters
        ----------
        bayesian_network_data : List[BayesianNetworkData]
            List of all data instances, one for each vehicle.

        Returns
        -------
        List[BayesianNetworkData]
            List of all data instances, one for each vehicle.

        """
        for specific_bn_data in bayesian_network_data:
            bayesian_network_id = specific_bn_data.bayesian_network_id

            if bayesian_network_id:
                bn_model, bn_config = self._create_bn_instances(bayesian_network_id)
                specific_bn_data.bayesian_network_model = bn_model
                specific_bn_data.bayesian_network_config = bn_config

        return bayesian_network_data

    def _create_bn_instances(self, bayesian_network_id: str) -> Tuple["BayesianModel", "BayesianNetworkConfig"]:
        """Extracts the pgmpy Bayesian model and the Bayesian network configuration for the given Bayesian network
        identifier.

        Parameters
        ----------
        bayesian_network_id : str
            Identifier of the input Bayesian network.

        Returns
        -------
        Tuple[BayesianModel, BayesianNetworkConfig]
            Pgmpy Bayesian model and the instantiated Bayesian network configuration for the given Bayesian network.
        """
        bn_type = self._bn_instance_for_network_id[bayesian_network_id]
        bn_instance = bn_type.copy()
        bn_config_class = self._bn_config_class_for_network_id[bayesian_network_id]
        bn_config_instance = bn_config_class()

        return bn_instance, bn_config_instance

    def _initial_loading_of_all_networks(self) -> None:
        """Loads the Bayesian network configuration class and the Bayesian network pgmpy model for all the provided
        Bayesian networks. Further, stores them in the instance dictionaries for the Bayesian network objects.
        """
        for bayesian_network_id in VEHICLE_SITUATION_STATE_TO_BAYESIAN_NETWORK.values():
            if bayesian_network_id:
                self._load_bn_files_for_network_id(bayesian_network_id)

    def _load_bn_files_for_network_id(self, bayesian_network_id: str) -> None:
        """Loads the Bayesian network configuration class and the Bayesian network pgmpy model.
        Further, stores them in the instance dictionaries for the Bayesian network objects.

        Parameters
        ----------
        bayesian_network_id : str
            Name/identifier for the Bayesian network.
        """
        self._bn_config_class_for_network_id[bayesian_network_id] = self._load_bn_config_class(bayesian_network_id)
        self._bn_instance_for_network_id[bayesian_network_id] = self._load_bn(bayesian_network_id)

    def _load_bn_config_class(self, bayesian_network_id: str) -> Type["BayesianNetworkConfig"]:
        """Loads the Bayesian network configuration Python class.

        Parameters
        ----------
        bayesian_network_id : str
            Name/identifier for the Bayesian network.

        Returns
        -------
        Type[BayesianNetworkConfig]
            Type of the Bayesian network configuration for the given Bayesian network identifier that can be
            instantiated later on.
        """
        bn_config_pkg = importlib.import_module(f"{self._package_to_files}.{bayesian_network_id}"
                                                f".{self._config_file_name}")
        bn_config_class = bn_config_pkg.BayesianNetworkConfig  # type:ignore

        return bn_config_class

    def _load_bn(self, bayesian_network_id: str) -> "BayesianModel":
        """Loads the .xmlbif file and generates the corresponding pgmpy Bayesian model object.

        Parameters
        ----------
        bayesian_network_id : str
            Name/identifier for the Bayesian network.

        Returns
        -------
        BayesianModel
            Instantiated pgmpy Bayesian model corresponding to the given Bayesian network identifier.
        """
        path = (self._path_to_files + f"{bayesian_network_id}/{self._bayesian_network_file_name}")
        reader = XMLBIFReader(path=path)
        bayesian_network = reader.get_model()

        return bayesian_network
