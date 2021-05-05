#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
from typing import List, Optional, ClassVar, TYPE_CHECKING
from dataclasses import dataclass
import math

if TYPE_CHECKING:
    from bayesian_network.inference.sinadra_risk_sensor_data import SINADRARiskSensorData
    from bayesian_network.model_generation.config_template import BayesianNetworkConfig
    from actor_situation_class_detection.bayesian_network_id_selection.bayesian_network_id import BayesianNetId
    from pgmpy.models.BayesianModel import BayesianModel
    import carla


@dataclass
class Outcome:
    """Data class that describes a specific state with its value of a node in a Bayesian network.
    (Python dataclasses.dataclass object.)

    Attributes
    ----------
    name : str
        Name of this specific state/outcome.
    value : float
        Value/probability for this specific state/outcome.
    """

    name: str
    value: float


@dataclass
class Node:
    """Data class that describes a Bayesian network node.
    (Python dataclasses.dataclass object.)

    Attributes
    ----------
    title : str
        The title of the node.
    outcomes : List[Outcome]
        List of the outcomes of the nodes with their state names and values/probabilities.
    """
    title: str
    outcomes: List[Outcome]


@dataclass()
class BayesianNetworkOutput:
    """This data class describes the output after a Bayesian network inference for a specific vehicle. For given output
    nodes the infered values and their states are collected in here.
    (Python dataclasses.dataclass object.)

    Attributes
    ----------
    vehicle_id : str
        Identifier of the respective vehicle.
    vehicle_situation_state_id : Optional[BayesianNetId]
        State of the vehicle relative to the ego vehicle and its situation class.  (The default value is None.)
        (E.g. indicating the state of being a front vehicle in a lane following situation.)
    bayesian_network_id : Optional[str]
        Identifier for the Bayesian network that was used for the inference.
    output_nodes : List[Node]
        List of the defined output nodes for this Bayesian network. The infered node values and states are saved in
        here.
    """

    vehicle_id: str
    bayesian_network_id: str
    output_nodes: List[Node]
    vehicle_situation_state_id: Optional["BayesianNetId"] = None

    def __str__(self) -> str:
        output_string = f"{self.bayesian_network_id}:\n"
        for node in self.output_nodes:
            output_string += f"- {node.title}\n"
            for outcome in node.outcomes:
                output_string += f"    {outcome.name}: \t\t{outcome.value}\n"
        return output_string


@dataclass()
class VehicleLocation:
    """Vehicle location as a 3D vector.

    Attributes
    ----------
    x : float
        Location on the x-axis.
    y : float
        Location on the y-axis.
    z : float
        Location on the z-axis.
    """

    x: float
    y: float
    z: float

    def get_2d_distance(self, other_location: "VehicleLocation") -> float:
        """Calculates the 2-dimensional distance of the object location and the given other location.

        Parameters
        ----------
        other_location : VehicleLocation
            Second location to calculate the distance to.

        Returns
        -------
        float
            Distance between the two locations.
        """
        distance = math.hypot(self.x - other_location.x, self.y - other_location.y)
        return distance


@dataclass
class BayesianNetworkData:
    """Data class that is typically used as input for the methods that are part of the Bayesian network inference
    module. Each vehicle got its own data instance. If there is no Bayesian network ID set there will be no inference
    for the respective vehicle.
    (Python dataclasses.dataclass object.)

    Attributes
    ----------
    vehicle_id : str
        Identifier of the respective vehicle.
    vehicle_situation_state_id : BayesianNetId
        State of the vehicle relative to the ego vehicle and its situation class.
        (E.g. indicating the state of being a front vehicle in a lane following situation.)
    vehicle_location : VehicleLocation
        Position of the vehicle in the world coordinate system. Required for the filtering of vehicles, e.g., the
        filtering of the nearest front vehicle to the ego in case of several front vehicles.
    bayesian_network_id : Optional[str]
        Identifier for the Bayesian network that shall be used for the inference. Will only be set if the inference
        will be performed for this specific vehicle. (The default value is None.)
    sinadra_risk_sensor_data : Optional[SINADRARiskSensorData]
        Input data class comprising the situation input features that will be fed to the Bayesian network configuration
        for situation-specific runtime updates of the evidences. Will only be set if the inference will be performed
        for this specific vehicle. (The default value is None.)
    bayesian_network_config : Optional[BayesianNetworkConfig]
        Configuration object of this specific Bayesian network that allows situation-specific runtime updates of the
        evidences of the Bayesian network. Will only be set if the inference will be performed for this specific
        vehicle. (The default value is None.)
    bayesian_network_model : Optional[BayesianModel]
        Bayesian network for this specific vehicle as pgmpy Bayesian model object. Will only be set if the inference
        will be performed for this specific vehicle. (The default value is None.)
    """

    vehicle_id: str
    vehicle_situation_state_id: "BayesianNetId"
    vehicle_location: VehicleLocation
    bayesian_network_id: Optional[str] = None
    sinadra_risk_sensor_data: Optional["SINADRARiskSensorData"] = None
    bayesian_network_config: Optional["BayesianNetworkConfig"] = None
    bayesian_network_model: Optional["BayesianModel"] = None


@dataclass()
class CARLABayesianNetworkInputFeatureData:
    """Data class that is used as input for the creation of the Bayesian network's specific input feature data classes
    that are used in the Bayesian network configurations.
    (Python dataclasses.dataclass object.)

    Attributes
    ----------
    vehicle_id : str
        Identifier of the respective vehicle.
    carla_vehicle : carla.Vehicle
        Vehicle object for the given vehicle from the CARLA simulator.
    bayesian_network_id : Optional[str]
        Identifier for the Bayesian network that shall be used for the inference. Will only be set if the inference
        will be performed for this specific vehicle. (The default value is None.)

    carla_world : carla.World
        (Class attribute) World object for the current CARLA simulator world which is the access point for the vehicle's
        environment.
    carla_map : carla.Map
        (Class attribute) Map object for the current CARLA simulator world which is the access point for the topology
    """

    vehicle_id: str
    carla_vehicle: "carla.Vehicle"
    bayesian_network_id: Optional[str] = None

    carla_world: ClassVar["carla.World"]
    carla_map: ClassVar["carla.Map"]

    @staticmethod
    def set_carla_world(carla_world: "carla.World") -> None:
        """Sets the given CARLA world object.

        Parameters
        ----------
        carla_world : carla.World
        """
        CARLABayesianNetworkInputFeatureData.carla_world = carla_world

    @staticmethod
    def set_carla_map(carla_map: "carla.Map") -> None:
        """Sets the given CARLA map object.

        Parameters
        ----------
        carla_map : carla.Map
        """
        CARLABayesianNetworkInputFeatureData.carla_map = carla_map
