#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################

from data_model.sinadra_data import SinadraData
from data_model.vehicle import EgoVehicle, OtherVehicle, VehicleLightState, SurroundingVehicles, VehicleControl, \
    KinematicStatus
from data_model.positions import Position, Location, WorldPosition, Orientation, Vector3D
from data_model.environment import Environment, Weather, Precipitation
from data_model.entity import BoundingBox, Dimension
from data_model.map import Map

from typing import TYPE_CHECKING, List
import math
from carla import VehicleLightState as CarlaVehicleLightState

if TYPE_CHECKING:
    from carla import Vehicle as CarlaVehicle
    from carla import VehicleControl as CarlaVehicleControl
    from carla import BoundingBox as CarlaBoundingBox
    from carla import World as CarlaWorld
    from carla import WeatherParameters as CarlaWeatherParameters
    from carla import Actor as CarlaActor


class CarlaSinadraDataHandler(object):

    def __init__(self):
        pass

    def populate_data_model(self, hero_vehicle: "CarlaVehicle", other_vehicles: List["CarlaVehicle"],
                            carla_world: "CarlaWorld") -> SinadraData:
        """Creates the Sinadra Data model using CARLA data

        Parameters
        ----------
        hero_vehicle : CarlaVehicle
            hero vehicle for the sinadra scenario
        other_vehicles : List[CarlaVehicle]
            Other vehicles present in the sindra scenario
        carla_world : CarlaWorld
            Refernce to the currently active carla world object

        Returns
        -------
        SinadraData
            data object containing all relevant information used by the SINADRA code
        """

        hero_vehicle = self.__create_hero_vehicle(hero_vehicle)
        data: SinadraData = SinadraData(hero_vehicle)
        data.other_vehicles = self.__create_other_vehicles(other_vehicles)
        data.environment = self.__create_environment(carla_world.get_weather())

        return data

    def __create_hero_vehicle(self, carla_vehicle: "CarlaVehicle") -> EgoVehicle:
        hero_vehicle = EgoVehicle()
        hero_vehicle.id = carla_vehicle.id
        hero_vehicle.role_name = carla_vehicle.attributes["role_name"]
        hero_vehicle.length = carla_vehicle.bounding_box.extent.x * 2
        hero_vehicle.kinematics = self.__create_kinematics(carla_vehicle)
        hero_vehicle.surrounding_vehicles = self.__create_surrounding_vehicles(carla_vehicle)
        hero_vehicle.bounding_box = self.__create_bounding_box(carla_vehicle.bounding_box)
        hero_vehicle.light_state = self.__create_vehicle_light_state(carla_vehicle.get_light_state())
        hero_vehicle.vehicle_control = self.__create_vehicle_control(carla_vehicle.get_control())
        hero_vehicle.position = self.__create_position(carla_vehicle)
        hero_vehicle.vehicle_points = hero_vehicle.get_middle_points_dict(hero_vehicle.position.world_position)

        return hero_vehicle

    def __create_other_vehicles(self, other_carla_vehicles: List["CarlaVehicle"]) -> List[OtherVehicle]:
        other_vehicles = []
        for carla_vehicle in other_carla_vehicles:
            other_vehicle = OtherVehicle()
            other_vehicle.id = carla_vehicle.id
            other_vehicle.role_name = carla_vehicle.attributes["role_name"]
            other_vehicle.length = carla_vehicle.bounding_box.extent.x * 2
            other_vehicle.kinematics = self.__create_kinematics(carla_vehicle)
            other_vehicle.surrounding_vehicles = self.__create_surrounding_vehicles(carla_vehicle)
            other_vehicle.bounding_box = self.__create_bounding_box(carla_vehicle.bounding_box)
            other_vehicle.light_state = self.__create_vehicle_light_state(carla_vehicle.get_light_state())
            other_vehicle.vehicle_control = self.__create_vehicle_control(carla_vehicle.get_control())
            other_vehicle.position = self.__create_position(carla_vehicle)
            other_vehicle.vehicle_points = other_vehicle.get_middle_points_dict(
                other_vehicle.position.world_position)

            other_vehicles.append(other_vehicle)

        return other_vehicles

    def __create_kinematics(self, carla_actor: "CarlaActor") -> KinematicStatus:
        kinematics: KinematicStatus = KinematicStatus()
        carla_velocity = carla_actor.get_velocity()
        kinematics.velocity = Vector3D(carla_velocity.x, -carla_velocity.y, carla_velocity.z)
        kinematics.speed = math.sqrt(
            kinematics.velocity.x ** 2 + kinematics.velocity.y ** 2 + kinematics.velocity.z ** 2)
        carla_acceleration = carla_actor.get_acceleration()
        kinematics.acceleration_vector = Vector3D(carla_acceleration.x, -carla_acceleration.y, carla_acceleration.z)
        kinematics.acceleration = math.sqrt(kinematics.acceleration_vector.x ** 2 +
                                            kinematics.acceleration_vector.y ** 2 + kinematics.acceleration_vector.z ** 2)

        return kinematics

    def __create_surrounding_vehicles(self, carla_vehicle: "CarlaVehicle") -> SurroundingVehicles:

        surrounding_vehicles: SurroundingVehicles = SurroundingVehicles()

        return surrounding_vehicles

    def __create_bounding_box(self, carla_bounding_box: "CarlaBoundingBox") -> BoundingBox:

        bounding_box: BoundingBox = BoundingBox()

        # Transforming from carlas left handed to opendrives right handed coordinate system
        x = carla_bounding_box.location.x
        y = -carla_bounding_box.location.y
        z = carla_bounding_box.location.z

        center: Location = Location(x, y, z)

        length: float = carla_bounding_box.extent.x * 2
        width: float = carla_bounding_box.extent.y * 2
        height: float = carla_bounding_box.extent.z * 2

        dimension: Dimension = Dimension(length, width, height)

        bounding_box.center = center
        bounding_box.dimension = dimension

        return bounding_box

    def __create_vehicle_light_state(self, carla_vehicle_lightstate: CarlaVehicleLightState) -> VehicleLightState:
        light_state: VehicleLightState = VehicleLightState()
        if carla_vehicle_lightstate & CarlaVehicleLightState.NONE:
            light_state.NONE = True
            return light_state
        if carla_vehicle_lightstate & CarlaVehicleLightState.Position:
            light_state.position = True
        if carla_vehicle_lightstate & CarlaVehicleLightState.LowBeam:
            light_state.lowbeam = True
        if carla_vehicle_lightstate & CarlaVehicleLightState.HighBeam:
            light_state.highbeam = True
        if carla_vehicle_lightstate & CarlaVehicleLightState.RightBlinker:
            light_state.right_blinker = True
        if carla_vehicle_lightstate & CarlaVehicleLightState.LeftBlinker:
            light_state.left_blinker = True
        if carla_vehicle_lightstate & CarlaVehicleLightState.Reverse:
            light_state.reverse = True
        if carla_vehicle_lightstate & CarlaVehicleLightState.Fog:
            light_state.fog = True
        if carla_vehicle_lightstate & CarlaVehicleLightState.Interior:
            light_state.interior = True
        if carla_vehicle_lightstate & CarlaVehicleLightState.Special1:
            light_state.special_1 = True
        if carla_vehicle_lightstate & CarlaVehicleLightState.Special2:
            light_state.special_2 = True
        if carla_vehicle_lightstate & CarlaVehicleLightState.All:
            light_state.all = True

        return light_state

    def __create_vehicle_control(self, carla_control: "CarlaVehicleControl") -> VehicleControl:
        vehicle_control: VehicleControl = VehicleControl()
        vehicle_control.throttle = carla_control.throttle
        vehicle_control.steer = carla_control.steer
        vehicle_control.brake = carla_control.brake
        vehicle_control.hand_brake = carla_control.hand_brake
        vehicle_control.reverse = carla_control.reverse
        vehicle_control.gear = carla_control.gear

        return vehicle_control

    def __create_position(self, carla_actor: "CarlaActor") -> Position:
        position: Position = Position()

        carla_transform = carla_actor.get_transform()

        location: Location = Location()
        orientation: Orientation = Orientation()
        location.x = carla_transform.location.x
        location.y = -carla_transform.location.y
        location.z = carla_transform.location.z

        orientation.pitch = carla_transform.rotation.pitch
        orientation.heading = -carla_transform.rotation.yaw
        orientation.roll = -carla_transform.rotation.roll

        world_position: WorldPosition = WorldPosition()
        world_position.location = location
        world_position.orientation = orientation
        position.world_position = world_position

        return position

    def __create_environment(self, carla_weather: "CarlaWeatherParameters") -> Environment:
        # TODO add all values
        environment = Environment()
        environment.weather = self.__create_weather(carla_weather)

        return environment

    def __create_weather(self, carla_weather: "CarlaWeatherParameters") -> Weather:
        weather: Weather = Weather()
        weather.precipitation = Precipitation()
        weather.precipitation.intensity = carla_weather.precipitation

        return weather