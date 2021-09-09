#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
import argparse
import os

# Required to run from PyCharm as well as independently from terminal
try:
    from bayesian_network.model_generation.config_creator import BayesianNetworkConfigCreator
except ModuleNotFoundError:
    from config_creator import BayesianNetworkConfigCreator

from pgmpy.models import BayesianModel
from pgmpy.factors.discrete import TabularCPD
from pgmpy.readwrite.XMLBIF import XMLBIFReader, XMLBIFWriter


PATH = os.path.dirname(os.path.abspath(__file__))
"""str : Absolute directory path to this file's directory."""


def main() -> None:
    """Starts the Bayesian network configuration creation process for the given Bayesian network name.
    The .xmlbif file of the Bayesian network must be located in the same directory as this script.
    In case there is no Bayeaian network file but rather the Bayesian network is defined using pgmpy on a code level,
    it is possible to build the Bayesian network in the `create_code_based_bayesian_network()` method (using the
    --codeBased console argument).

    Note
    ----
    Example start command for the Bayesian network with the name TestSituationClassOne that is defined on the code
    level: python file_creator_initial.py -bn TestSituationClassOne -cb
    Type `python file_creator_initial.py --help` for more details on the console arguments.
    """
    parser = set_up_argparser()
    bayesian_network_id = parser.parse_args().bayesian_network_id
    code_based = parser.parse_args().bn_defined_code_based

    if code_based:
        bayesian_network = create_code_based_bayesian_network()
        write_model(bayesian_network, bayesian_network_id)
    else:
        bayesian_network = load_bayesian_network_from_xmlbif_file(bayesian_network_id)

    BayesianNetworkConfigCreator(bayesian_network, bayesian_network_id)


def set_up_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Bayesian network config file creation. If using a .xmlbif file to "
                                                 "represent the Bayesian network it is required to copy the file in "
                                                 "the directory of this script. The file will be moved to the target "
                                                 "location afterwards.")
    parser.add_argument("-bn", "--bayesianNetworkID", help="ID of the associated Bayesian network (= file name (without"
                                                           " file extension) for loading a .xmlbif file as well)",
                        action="store", dest="bayesian_network_id", required=True, type=str)
    parser.add_argument("-cb", "--codeBased", help="Flag to load the code-based Bayesian network instead of loading a "
                                                   ".xmlbif file",
                        action="store_true", dest="bn_defined_code_based")

    return parser


def create_code_based_bayesian_network() -> BayesianModel:
    """Fill this out with your specific Bayesian network in case that you do not want to load a .xmlbif file but rather
    want to define your network code-based.

    Note
    ----
    Currently:
    Example BN extracted from the official pgmpy examples.

    Returns
    -------
    BayesianModel
        Created Bayesian network.
    """
    cancer_model = BayesianModel([('Pollution', 'Cancer'), ('Smoker', 'Cancer'), ('Cancer', 'Xray'),
                                  ('Cancer', 'Dyspnoea')])

    cpd_poll = TabularCPD(variable='Pollution', variable_card=2, values=[[0.9], [0.1]],
                          state_names={"Pollution": ["Yes", "No"]})
    cpd_smoke = TabularCPD(variable='Smoker', variable_card=2, values=[[0.3], [0.7]])
    cpd_cancer = TabularCPD(variable='Cancer', variable_card=2, values=[[0.03, 0.05, 0.001, 0.02],
                                                                        [0.97, 0.95, 0.999, 0.98]],
                            evidence=['Smoker', 'Pollution'], evidence_card=[2, 2])
    cpd_xray = TabularCPD(variable='Xray', variable_card=2, values=[[0.9, 0.2], [0.1, 0.8]], evidence=['Cancer'],
                          evidence_card=[2])
    cpd_dysp = TabularCPD(variable='Dyspnoea', variable_card=2, values=[[0.65, 0.3], [0.35, 0.7]], evidence=['Cancer'],
                          evidence_card=[2])

    # Associating the parameters with the model structure.
    cancer_model.add_cpds(cpd_poll, cpd_smoke, cpd_cancer, cpd_xray, cpd_dysp)

    # Checking if the cpds are valid for the model.
    cancer_model.check_model()

    return cancer_model


def write_model(model: BayesianModel, file_name: str) -> None:
    writer = XMLBIFWriter(model)
    writer.write_xmlbif(f"{PATH}/{file_name}.xmlbif")


def load_bayesian_network_from_xmlbif_file(bayesian_network_id: str) -> BayesianModel:
    reader = XMLBIFReader(f"{PATH}/{bayesian_network_id}.xmlbif")
    bayesian_network = reader.get_model()

    return bayesian_network


if __name__ == "__main__":
    main()
