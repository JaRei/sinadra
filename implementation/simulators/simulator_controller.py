#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################

from typing import Optional, Tuple, TYPE_CHECKING

if TYPE_CHECKING:
    from data_model.sinadra_data import SinadraData
    from data_model.map import Map
    from data_model.positions import Vector3D
    import numpy as np


class SimulatorController:
    def connect_simulator(self) -> None:
        """Connects the simulator client to the running simulator server instance."""
        raise NotImplementedError

    def setup_simulator(self) -> None:
        """Sets up the simulator client for running evaluation scenarios."""
        raise NotImplementedError

    def run_simulator_game_loop_step(self) -> None:
        """Runs one simulation tick of the simulator. Must be called every computation cycle (must align with the
        FRAMERATE configuration parameter).
        """
        raise NotImplementedError

    def get_sinadra_data(self) -> Optional["SinadraData"]:
        """Extracts and Returns the latest SINADRA data (environment and all entities) of the current simulation
        step.

        Returns
        -------
        Optional[SinadraData]
            SINADRA data object if the vehicles are already instantiated. If the vehicles are not instantiated yet,
            e.g., if a scenario is not launched already, None is returned instead.
        """
        raise NotImplementedError

    def get_map(self) -> "Map":
        """Extracts and Returns the map of the simulator. Typically, done once in the initialization due to the map
        being invariant and slow to load.

        Returns
        -------
        Map
            Map SINADRA data object of the simulator.
        """
        raise NotImplementedError

    def get_scenario_image(self) -> Optional["np.ndarray"]:
        """Extracts and Returns a top view image of the area of interest.

        Returns
        -------
        Optional[np.ndarray]
            Image as a numpy ndarray if the vehicles are already instantiated. If the vehicles are not instantiated yet,
            e.g., if a scenario is not launched already, None is returned instead.
        """
        raise NotImplementedError

    def get_subject_vehicle_image(self) -> Optional["np.ndarray"]:
        """Extracts and Returns an image of the subject vehicle's perspective.

        Returns
        -------
        Optional[np.ndarray]
            Image as a numpy ndarray if the vehicles are already instantiated. If the vehicles are not instantiated yet,
            e.g., if a scenario is not launched already, None is returned instead.
        """
        raise NotImplementedError

    def draw_line(self, start: "Vector3D", end: "Vector3D", color: Tuple[int, int, int]) -> None:
        """Draws a line in the simulation if it is supported by the simulator.
        If the simulator does not support drawing lines: Do not implement this method.

        Parameters
        ----------
        start : Vector3D
            Start point of the line to draw
        end : Vector3D
            End point of the line to draw
        color : Tuple[int, int, int]
            Color of the lin to draw as (r, g, b) tuple.
        """
        pass
