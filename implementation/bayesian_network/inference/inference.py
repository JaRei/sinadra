#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
from bayesian_network.inference.interfaces import Outcome, Node, BayesianNetworkOutput
from sinadra_configuration_parameters import NUM_INTERACTION_HOPS
from actor_situation_class_detection.bayesian_network_id_selection.bayesian_network_id import BayesianNetId
from sys import float_info
from pgmpy.inference.ExactInference import VariableElimination
from typing import TYPE_CHECKING, List, Dict, Tuple

if TYPE_CHECKING:
    from bayesian_network.model_generation.config_template import BayesianNetworkConfig
    from bayesian_network.inference.interfaces import BayesianNetworkData, VehicleLocation
    from pgmpy.models import BayesianModel
    from pgmpy.factors.discrete import DiscreteFactor
    import multiprocessing


class BayesianNetworkInference:
    """Performs the Bayesian network inference for all given Bayesian models and input features."""

    def bn_inferences_for_vehicles(self, bayesian_network_data: List["BayesianNetworkData"],
                                   multiprocessing_pool: "multiprocessing.Pool") -> List[BayesianNetworkOutput]:
        """Performs the Bayesian network inference for the given Bayesian network data objects, respectively vehicles,
        and builds the output data instances with the infered values for the defined output nodes in each Bayesian
        network.

        Parameters
        ----------
        bayesian_network_data : List[BayesianNetworkData]
            List of the data objects, respectively vehicles, for which the Bayesian network inference shall be
            performed.
        multiprocessing_pool : multiprocessing.Pool
            Processing pool that is used for the parallel Bayesian network inference. (Initialized outside once each
            run for performance reasons.)

        Returns
        -------
        List[BayesianNetworkOutput]
            List of the data objects that hold alle relevant information about the vehicle after the Bayesian inference
            including the infered values of the defined output nodes.
        """
        bn_outputs = []

        # filtering to the target number of front vehicles
        if NUM_INTERACTION_HOPS:
            inference_objects = self._filter_number_of_front_vehicles(bayesian_network_data)

        # filter only objects that have a BN to infer assigned to
        inference_objects = list(filter(lambda x: bool(x.bayesian_network_id), inference_objects))

        if len(inference_objects) >= 1:
            bn_outputs = multiprocessing_pool.map(self._worker_inference, inference_objects)

        return bn_outputs

    def _filter_number_of_front_vehicles(self, inference_objects: List["BayesianNetworkData"]
                                         ) -> List["BayesianNetworkData"]:
        """Limits the received front vehicles from the situation class detection to the maximum number of supported
        front vehicles (equals the NUM_INTERACTION_HOPS that is provided in the SINADRA configuration script).
        The front vehicles are sorted based on the distance to the ego vehicle and the nearest front vehicles are kept.

        Parameters
        ----------
        inference_objects : List[BayesianNetworkData]
            Inference objects including any detected front vehicle.

        Returns
        -------
        List[BayesianNetworkData]
            Inference objects with only the maximum number of front vehicles.
        """
        filter_function_no_front_vehicle = (lambda data: data.vehicle_situation_state_id !=
                                                         BayesianNetId.TWO_LANE_FOLLOWING_FRONT_VEHICLE)
        filter_function_only_front_vehicle = (lambda data: data.vehicle_situation_state_id ==
                                                           BayesianNetId.TWO_LANE_FOLLOWING_FRONT_VEHICLE)
        filter_function_ego_vehicle = (lambda data: data.vehicle_situation_state_id ==
                                                    BayesianNetId.TWO_LANE_FOLLOWING_EGO)

        data_without_front_vehicles: List["BayesianNetworkData"]
        data_without_front_vehicles = list(filter(filter_function_no_front_vehicle, inference_objects))
        front_vehicles: List["BayesianNetworkData"]
        front_vehicles = list(filter(filter_function_only_front_vehicle, inference_objects))
        ego_vehicle: "BayesianNetworkData"
        ego_vehicle = next(filter(filter_function_ego_vehicle, data_without_front_vehicles), None)

        if ego_vehicle and front_vehicles:
            front_vehicles = self._sort_by_distance_ascending(ego_vehicle, front_vehicles)
            if len(front_vehicles) > NUM_INTERACTION_HOPS:
                front_vehicles = front_vehicles[:NUM_INTERACTION_HOPS]

            inference_objects = front_vehicles + data_without_front_vehicles

        return inference_objects

    @staticmethod
    def _sort_by_distance_ascending(reference_vehicle: "BayesianNetworkData",
                                    vehicles_to_check: List["BayesianNetworkData"]) -> List["BayesianNetworkData"]:
        """Sorts the given Bayesian network data objects by their vehicle's distance to the given reference vehicle's
        location in ascending order.

        Parameters
        ----------
        reference_vehicle : BayesianNetworkData
            Bayesian network data of the reference vehicle (typically the ego vehicle).
        vehicles_to_check : List[BayesianNetworkData]
            Given Bayesian network data objects for the vehicles that shall be sorted by distance.

        Returns
        -------
        List[BayesianNetworkData]
            Sorted list of the given input Bayesian network data objects.
        """
        reference_location: "VehicleLocation" = reference_vehicle.vehicle_location
        vehicles_to_check.sort(key=lambda data: reference_location.get_2d_distance(data.vehicle_location))
        return vehicles_to_check

    def _worker_inference(self, specific_bn_data: "BayesianNetworkData") -> BayesianNetworkOutput:
        """Does the inference for the given Bayesian network data. This method is used as worker method in the
        multiprocessing of the Bayesian network inference.

        Parameters
        ----------
        specific_bn_data : BayesianNetworkData
            Bayesian network data for which the Bayesian network shall be inferred.

        Returns
        -------
        BayesianNetworkOutput
            Bayesian network output object for the given input data after inference.
        """
        bn_network_output = self._bn_inference(specific_bn_data)
        bn_network_output = self._add_ids_to_output(bn_network_output, specific_bn_data)
        bn_network_output.vehicle_situation_state_id = specific_bn_data.vehicle_situation_state_id

        return bn_network_output

    def _bn_inference(self, specific_bn_data: "BayesianNetworkData") -> BayesianNetworkOutput:
        """Does the inference for the given Bayesian network data.

        Parameters
        ----------
        specific_bn_data : BayesianNetworkData
            Bayesian network data for which the Bayesian network shall be inferred.

        Returns
        -------
        BayesianNetworkOutput
            Bayesian network output object for the given input data after inference without the corresponding
            vehicle ID and Bayesian network ID.
        """
        bn_instance = specific_bn_data.bayesian_network_model
        bn_config_instance = specific_bn_data.bayesian_network_config
        risk_sensor_data = specific_bn_data.sinadra_risk_sensor_data
        vehicle_id = specific_bn_data.vehicle_id
        bn_id = specific_bn_data.bayesian_network_id

        bn_config_instance.update_dra_data(risk_sensor_data)

        output_nodes, evidence_nodes = self._extract_node_values_from_bn_config(bn_instance, bn_config_instance)
        evidence_nodes = self._filter_supported_evidence_nodes(evidence_nodes)
        evidence_query = self._create_evidence_query(evidence_nodes)
        output_network = self._inference_query_and_build_infered_bn_output(bn_instance, output_nodes, evidence_query,
                                                                           vehicle_id, bn_id)

        return output_network

    @staticmethod
    def _extract_node_values_from_bn_config(bn_instance: "BayesianModel", bn_config_instance: "BayesianNetworkConfig"
                                            ) -> Tuple[List[str], Dict[str, List[Outcome]]]:
        nodes = bn_instance.nodes
        output_nodes = []
        inference_nodes = {}

        for node_id in nodes:
            method = getattr(bn_config_instance, f"node_{node_id}")
            is_output, extracted_outcomes = method()

            if is_output:
                output_nodes.append(node_id)

            outcomes = []
            for name, value in extracted_outcomes.items():
                outcome = Outcome(name, value)
                outcomes.append(outcome)
            inference_nodes[node_id] = outcomes

        return output_nodes, inference_nodes

    @staticmethod
    def _filter_supported_evidence_nodes(evidence_nodes: Dict[str, List[Outcome]]) -> Dict[str, List[Outcome]]:
        """Filters illegal nodes. This is based on the fact that all state probabilities of a node must sum up to 1.0
        and that pgmpy does not support virtual evidences as of now.

        Parameters
        ----------
        evidence_nodes : Dict[str, List[Outcome]]
            Input dictionary of node IDs to their Outcomes.

        Returns
        -------
        Dict[str, List[Outcome]]
            Filtered dictionary of node IDs to their Outcomes.
        """
        supported_evidence_nodes = {}

        for node_id, outcomes in evidence_nodes.items():
            outcome_values = [outcome.value for outcome in outcomes]
            sum_of_outcome_values = sum(outcome_values)

            # sum of p(outcome) for all outcomes = 100%
            if 1.0 - float_info.epsilon < sum_of_outcome_values < 1.0 + float_info.epsilon:
                max_of_outcome_values = max(outcome_values)

                # only one outcome is "set" = only one with 100%
                if 1.0 - float_info.epsilon < max_of_outcome_values < 1.0 + float_info.epsilon:
                    max_index = outcome_values.index(max_of_outcome_values)
                    # assure that there is no rounding issue
                    outcome_values[max_index] = 1.0

                    supported_evidence_nodes[node_id] = outcomes

        return supported_evidence_nodes

    @staticmethod
    def _create_evidence_query(node_outcomes: Dict[str, List[Outcome]]) -> Dict[str, str]:
        evidence_query = {}

        for node_id, outcomes in node_outcomes.items():
            outcome_values = [outcome.value for outcome in outcomes]
            outcome_names = [outcome.name for outcome in outcomes]
            index_of_evidence = outcome_values.index(max(outcome_values))
            evidence_query[node_id] = outcome_names[index_of_evidence]

        return evidence_query

    @staticmethod
    def _inference_query_and_build_infered_bn_output(bn_instance: "BayesianModel", output_nodes: List[str],
                                                     evidence_query: Dict[str, str], vehicle_id: str, bn_id: str
                                                     ) -> BayesianNetworkOutput:
        infered_output_nodes = []
        inference_algorithm = VariableElimination(bn_instance)
        output_discrete_factors = inference_algorithm.query(variables=output_nodes, evidence=evidence_query,
                                                            joint=False, show_progress=False)

        if output_nodes:
            for node_id, discrete_factor in output_discrete_factors.items():
                node_output = BayesianNetworkInference._extract_outcomes_and_build_node_output(discrete_factor, node_id)
                infered_output_nodes.append(node_output)

        network_output = BayesianNetworkOutput(vehicle_id, bn_id, infered_output_nodes)

        return network_output

    @staticmethod
    def _extract_outcomes_and_build_node_output(discrete_factor: "DiscreteFactor", node_id: str) -> Node:
        title = node_id
        outcomes = [list(discrete_factor.state_names.values())[0], list(discrete_factor.values)]
        outcomes = list(zip(*outcomes))
        outcomes_output = []

        for outcome_name, outcome_value in outcomes:
            name = outcome_name
            value = outcome_value
            outcome = Outcome(name, value)
            outcomes_output.append(outcome)

        node_output = Node(title, outcomes_output)

        return node_output

    @staticmethod
    def _add_ids_to_output(bn_network_output: BayesianNetworkOutput, specific_bn_data: "BayesianNetworkData"
                           ) -> BayesianNetworkOutput:
        """Extracts the vehicle ID and the Bayesian network ID from the given Bayesian network input data and extends
        the Bayesian network output by these IDs.

        Parameters
        ----------
        bn_network_output : BayesianNetworkOutput
            Bayesian network output in which the vehicle ID and Bayesian network ID is still missing.
        specific_bn_data : BayesianNetworkData
            Given input Bayesian network data on which the Bayesian network output is based on.

        Returns
        -------
        BayesianNetworkOutput
            Bayesian network output with the corresponding vehicle ID and Bayesian network ID.
        """
        bn_network_output.vehicle_id = specific_bn_data.vehicle_id
        bn_network_output.bayesian_network_id = specific_bn_data.bayesian_network_id

        return bn_network_output
