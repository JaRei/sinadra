#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
from pgmpy.models import BayesianModel
from typing import IO
import os


class BayesianNetworkFilesAlreadyExistException(Exception):
    pass


class BayesianNetworkConfigCreator:
    """Creates a Python script including the required functionality for customization of the Bayesian network inference.

    Attributes
    ----------
    file_directory_path : str
        (Class attribute) Absolute path to the files location to easily locate the saved Bayesian network files in the
        "../files/" directory. Extracted using Python's os package.
    """

    file_directory_path = os.path.dirname(os.path.abspath(__file__))

    # ========== Initialization methods ================================

    def __init__(self, bayesian_network: BayesianModel, bayesian_network_id: str) -> None:
        """Instantiates the config creator object and creates the config script. Further, the Bayesian network files
        are moved as well.

        Parameters
        ----------
        bayesian_network : BayesianModel
            Bayesian network for which the configuration is created.
        bayesian_network_id : str
            Name / identifier of the Bayesian network. Defines the name of the folder that will contain the Bayesian
            network's files.

        Raises
        ------
        BayesianNetworkFilesAlreadyExistException
            Raised in case that a folder with the given Bayesian network name/id already exists.
        """
        self._bayesian_network_id = bayesian_network_id
        self._bayesian_network = bayesian_network
        self._INDENT = "    "
        self._config_file: IO

        self._create_config_script()

    # ========== File level methods ====================================

    def _create_config_script(self) -> None:
        """Creates new empty file and includes the Bayesian network config template file content. Further, adds the
        header and all the node methods to the configuration script. Finally, writes it to the hard drive and moves it
        to the correct folder.

        Raises
        ------
        BayesianNetworkFilesAlreadyExistException
            Raised in case that a folder with the given Bayesian network name/id already exists.
        """
        self._create_empty_file()
        template_file = self._load_template_file()

        for line in template_file:
            self._config_file.write(line)

        self._add_node_methods()

        self._config_file.close()
        self._copy_bayesian_network_file()

    def _create_empty_file(self) -> None:
        """Creates an empty file (if the file is already existent, empties it).

        Raises
        ------
        BayesianNetworkFilesAlreadyExistException
            Raised in case that a folder with the given Bayesian network name/id already exists.
        """
        path = (f'{BayesianNetworkConfigCreator.file_directory_path}/../files/{self._bayesian_network_id}'
                f'/bayesian_network_config.py')

        if os.path.exists(os.path.dirname(path)):
            raise BayesianNetworkFilesAlreadyExistException(f"Directory for this Bayesian network ID already exists. "
                                                            f"Please remove the bayesian network files folder "
                                                            f"\"{self._bayesian_network_id}\" manually.")
        else:
            os.makedirs(os.path.dirname(path))

        open(path, 'w').close()  # empty the file if already existent
        self._config_file = open(path, 'w')

    @staticmethod
    def _load_template_file() -> str:
        """Loads the content of the Bayesian network config template file.

        Returns
        -------
        str
            Content of the Bayesian network config template file.
        """
        path = f'{BayesianNetworkConfigCreator.file_directory_path}/config_template.py'
        template_file = open(path, "r")

        return template_file.read()

    # ========== Node method creation methods ==========================

    def _add_node_methods(self) -> None:
        """Adds the Bayesian network's node methods to the script."""
        nodes = self._bayesian_network.nodes

        for node in nodes:
            self._add_cpt_node_method(node)

    def _add_cpt_node_method(self, node: str) -> None:
        """Adds a CPT node's method to the script.

        Parameters
        ----------
        node : str
            Identifier of the Bayesian network's node.
        """
        method = "\n" + self._INDENT + "def node_" + node + "(self):\n"
        method += 2 * self._INDENT + f'"""Method for the CPT node \'{node}\' that allows setting the node as output node, \n'
        method += f'{2 * self._INDENT}and allows changing this nodes outcomes for the Bayesian network inference.\n'
        method += (f"\n{2 * self._INDENT}Returns\n{2 * self._INDENT}-------\n"
                   f"{2 * self._INDENT}Tuple[bool, Dict[str, float]]\n")
        method += (f"{3 * self._INDENT}Flag whether this node is an output node, and dictionary with the node outcome "
                   f"IDs as keys \n{3 * self._INDENT}and the corresponding probabilities as values.\n")
        method += f'{2 * self._INDENT}"""\n'
        method += 2 * self._INDENT + "is_bn_output = False\n"

        cpds = self._bayesian_network.get_cpds()
        cpd = [cpd for cpd in cpds if cpd.variable == node][0]
        outcomes = list(cpd.state_names.values())[0]

        for outcome in outcomes:
            method += (2 * self._INDENT + f"outcome_{outcome}" + " = 0.0\n")

        method += (2 * self._INDENT + "'''----------Manual edit begin----------'''\n" + "\n" + 2 * self._INDENT
                   + "'''----------Manual edit end----------'''\n")

        method += 2 * self._INDENT + "return is_bn_output, \\\n"
        method += 3 * self._INDENT + "{"

        first = True
        for outcome in outcomes:
            if first:
                first = False
                method += f"'{outcome}': outcome_{outcome},\n"
            else:
                method += 3 * self._INDENT + f"'{outcome}': outcome_{outcome},\n"

        method = method[:-2]
        method += "}\n"

        self._config_file.write(method)

    def _copy_bayesian_network_file(self) -> None:
        """Moves the Bayesian network .xmlbif (+.xdsl) files to the target location (../files/)."""
        destination = f"{BayesianNetworkConfigCreator.file_directory_path}/../files/{self._bayesian_network_id}/"
        os.rename(f"{BayesianNetworkConfigCreator.file_directory_path}/{self._bayesian_network_id}.xmlbif",
                  f"{destination}/bayesian_network.xmlbif")

        path_to_xdsl = f"{BayesianNetworkConfigCreator.file_directory_path}/{self._bayesian_network_id}.xdsl"
        if os.path.exists(path_to_xdsl):
            os.rename(path_to_xdsl, f"{destination}/{self._bayesian_network_id}.xdsl")
