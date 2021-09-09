#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################

# Custom Python Packages
from simulators.simulator_controller import SimulatorController
from sinadra_configuration_parameters import CARLA_SERVER_HOST, CARLA_SERVER_PORT, FRAMERATE, DEBUG_LINE_THICKNESS, \
    DEBUG_DRAWING_LIFE_TIME
from data_model.sinadra_data import SinadraData
from data_model.map import Map
from simulators.carla_sinadra_data_handler import CarlaSinadraDataHandler

# Standard Python Packages
import atexit
import carla
import cv2
import weakref
import numpy as np
from typing import Optional, Tuple, List, TYPE_CHECKING

if TYPE_CHECKING:
    from data_model.positions import Vector3D


class CarlaSimulatorController(SimulatorController):
    def __init__(self) -> None:
        self._town_name = "Town03"
        self._carla_client: Optional[carla.Client] = None
        self._carla_world: Optional[carla.World] = None
        self._carla_debug_helper: Optional[carla.DebugHelper] = None
        self._carla_spectator: Optional[carla.Spectator] = None
        self._carla_hero_vehicle: Optional[carla.Vehicle] = None
        self._map: Optional[Map] = None
        self._scenario_camera: Optional[carla.Sensor] = None
        self._hero_camera: Optional[carla.Sensor] = None
        self._sinadra_data_handler: CarlaSinadraDataHandler = CarlaSinadraDataHandler()
        self._scenario_cv_image: Optional[np.ndarray] = None
        self._hero_cv_image: Optional[np.ndarray] = None

        # 0 -> Do not set the spectator
        # 1 -> Top View on 1st Lane Following Scenarios
        # 2 -> Angled Side View on 1st Lane Following Scenarios (observing indicator for left to right lane change)
        # 3 -> Angled Side View on 1st Lane Following Scenarios (observing indicator for right to left lane change)
        # 4 -> Top View on 2nd Lane Following Scenarios
        self._spectator_view_choice: int = 1

        self._print_spectator_positions: bool = False

    def connect_simulator(self) -> None:
        self._carla_client = carla.Client(CARLA_SERVER_HOST, CARLA_SERVER_PORT)
        self._carla_world = self._carla_client.get_world()
        self._carla_debug_helper = self._carla_world.debug

    def setup_simulator(self) -> None:
        self._set_up_synchronization()
        self._set_up_town()
        self._set_up_spectator_for_lane_following()

    def run_simulator_game_loop_step(self) -> None:
        self._carla_world.tick()
        if self._print_spectator_positions:
            self._output_spectator_data()
        if not self._hero_camera and self._carla_hero_vehicle:
            self._set_up_scenario_camera()
            self._set_up_hero_camera(self._carla_hero_vehicle)
        elif not self._carla_hero_vehicle:
            if self._hero_camera:
                self._hero_camera.stop()
                self._hero_camera.destroy()
                self._hero_camera = None
            if self._scenario_camera:
                self._scenario_camera.stop()
                self._scenario_camera.destroy()
                self._scenario_camera = None

    def get_sinadra_data(self) -> Optional[SinadraData]:
        other_vehicles, self._carla_hero_vehicle = self._get_carla_vehicles()
        if self._carla_hero_vehicle:
            sinadra_data = self._sinadra_data_handler.populate_data_model(self._carla_hero_vehicle, other_vehicles,
                                                                          self._carla_world)
            return sinadra_data
        else:
            return None

    def get_map(self) -> Map:
        return self._map

    def get_scenario_image(self) -> Optional[np.ndarray]:
        return self._scenario_cv_image

    def get_subject_vehicle_image(self) -> Optional[np.ndarray]:
        return self._hero_cv_image

    def draw_line(self, start: "Vector3D", end: "Vector3D", color: Tuple[int, int, int]) -> None:
        # The y-axis is directed in the other direction in CARLA (in comparison to the OpenDrive map)
        carla_start = carla.Vector3D(start.x, -start.y, start.z)
        carla_end = carla.Vector3D(end.x, -end.y, end.z)
        carla_color = carla.Color(*color)
        print(f"drawing from \n{carla_start}\nto\n{carla_end}\nin Color\n{carla_color}")
        self._carla_debug_helper.draw_line(carla_start, carla_end, DEBUG_LINE_THICKNESS, carla_color,
                                           DEBUG_DRAWING_LIFE_TIME)

    def _get_carla_vehicles(self) -> Tuple[List[carla.Vehicle], carla.Vehicle]:
        all_vehicles = self._carla_world.get_actors().filter("vehicle.*")
        hero_vehicle = None

        for vehicle in all_vehicles:
            if vehicle.attributes["role_name"] == "hero":
                hero_vehicle = vehicle
                break

        other_vehicles = list(filter(lambda v: v.attributes["role_name"] != "hero", all_vehicles))
        return other_vehicles, hero_vehicle

    def _set_up_synchronization(self) -> None:
        self._set_up_synchronous_carla_mode()
        atexit.register(self._set_up_asynchronous_carla_mode)

    def _set_up_synchronous_carla_mode(self) -> None:
        settings = self._carla_world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1 / FRAMERATE
        self._carla_world.apply_settings(settings)

    def _set_up_asynchronous_carla_mode(self) -> None:
        settings = self._carla_world.get_settings()
        settings.synchronous_mode = False
        self._carla_world.apply_settings(settings)

    def _set_up_town(self) -> None:
        if self._carla_world.get_map().name != self._town_name:
            self._carla_client.load_world(self._town_name)
        self._map = Map(self._town_name)

    def _set_up_spectator_for_lane_following(self) -> None:
        self._carla_spectator = self._carla_world.get_spectator()

        if self._spectator_view_choice == 1:  # above the 1st lane following scenario
            target_location = carla.Location(x=-81.5, y=-66.5, z=70.0)
            target_rotation = carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0)
        elif self._spectator_view_choice == 2:  # side angle to view the indicator (left to right LC) (1st LF scenario)
            target_location = carla.Location(x=-73.534294, y=-41.217567, z=3.980349)
            target_rotation = carla.Rotation(pitch=-16.670803, yaw=-105.755310, roll=0.000033)
        elif self._spectator_view_choice == 3:  # side angle to view the indicator (right to left LC) (1st LF scenario)
            target_location = carla.Location(x=-76.0, y=-36.217567, z=3.980349)
            target_rotation = carla.Rotation(pitch=-15.0, yaw=-90.0, roll=0.000033)
        elif self._spectator_view_choice == 4:  # above the 2nd lane following scenario
            # TODO: adapt the locations for the location that shall be used for the 2nd lane following scenario
            target_location = carla.Location(x=-40.867714, y=-132.761581, z=35.147583)
            target_rotation = carla.Rotation(pitch=-82.317413, yaw=-88.617477, roll=-0.000881)
        else:  # ignore the spectator
            return

        target_transform = carla.Transform(target_location, target_rotation)
        self._carla_spectator.set_transform(target_transform)

    def _output_spectator_data(self) -> None:
        spectator_transform = self._carla_spectator.get_transform()
        spectator_location = spectator_transform.location
        spectator_rotation = spectator_transform.rotation
        print(f'Spectator location = {spectator_location} \nSpectator rotation = {spectator_rotation}')

    def _set_up_scenario_camera(self) -> None:
        camera_actor_bp = self._carla_world.get_blueprint_library().find("sensor.camera.rgb")
        camera_actor_bp.set_attribute("image_size_x", "1920")
        camera_actor_bp.set_attribute("image_size_y", "1080")
        target_location = carla.Location(x=-81.5, y=-66.5, z=70.0)  # above the lane following scenarios
        target_roation = carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0)
        target_transform = carla.Transform(target_location, target_roation)
        self._scenario_camera = self._carla_world.spawn_actor(camera_actor_bp, target_transform)
        weak_self = weakref.ref(self)
        self._scenario_camera.listen(lambda image: self._scenario_camera_callback(weak_self, image))

    def _set_up_hero_camera(self, carla_hero_vehicle: carla.Vehicle) -> None:
        camera_actor_bp = self._carla_world.get_blueprint_library().find("sensor.camera.rgb")
        camera_actor_bp.set_attribute("image_size_x", "1920")
        camera_actor_bp.set_attribute("image_size_y", "1080")
        camera_transform = carla.Transform(carla.Location(x=-7, z=3), carla.Rotation(pitch=-15))
        self._hero_camera = self._carla_world.spawn_actor(camera_actor_bp, camera_transform,
                                                   attach_to=carla_hero_vehicle)
        weak_self = weakref.ref(self)
        self._hero_camera.listen(lambda image: self._hero_camera_callback(weak_self, image))

    @staticmethod
    def _scenario_camera_callback(weak_self: weakref.ReferenceType, sensor_data_image: carla.Image) -> None:
        self = weak_self()
        np_image = self._extract_camera_image(sensor_data_image)
        self._scenario_cv_image = cv2.cvtColor(np_image, cv2.COLOR_BGR2RGB)

    @staticmethod
    def _hero_camera_callback(weak_self: weakref.ReferenceType, sensor_data_image: carla.Image) -> None:
        self = weak_self()
        np_image = self._extract_camera_image(sensor_data_image)
        self._hero_cv_image = cv2.cvtColor(np_image, cv2.COLOR_BGR2RGB)

    @staticmethod
    def _extract_camera_image(sensor_data_image: carla.Image) -> np.ndarray:
        image_data = np.frombuffer(sensor_data_image.raw_data, dtype=np.dtype("uint8"))
        np_image = np.reshape(image_data, (sensor_data_image.height, sensor_data_image.width, 4))
        np_image = np_image[:, :, :3]
        np_image = np_image[:, :, ::-1]
        return np_image
