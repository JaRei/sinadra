#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
#!/usr/bin/env python

# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides an example control for vehicles which
does not use CARLA's vehicle engine.

Limitations:
- Does not respect any traffic regulation: speed limit, traffic light, priorities, etc.
- Can only consider obstacles in forward facing reaching (i.e. in tight corners obstacles may be ignored).
"""

from functools import reduce
import math
import cv2

import carla

from srunner.scenariomanager.actorcontrols.simple_vehicle_control import SimpleVehicleControl
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime


class SimpleVehicleControlAdapted(SimpleVehicleControl):

    """
    Controller class for vehicles derived from BasicControl.

    The controller directly sets velocities in CARLA, therefore bypassing
    CARLA's vehicle engine. This allows a very precise speed control, but comes
    with limitations during cornering.

    In addition, the controller can consider blocking obstacles, which are
    classified as dynamic (i.e. vehicles, bikes, pedestrians). Activation of this
    features is controlled by passing proper arguments to the class constructor.
    The collision detection uses CARLA's obstacle sensor (sensor.other.obstacle),
    which checks for obstacles in the direct forward channel of the vehicle, i.e.
    there are limitation with sideways obstacles and while cornering.

    Args:
        actor (carla.Actor): Vehicle actor that should be controlled.
        args (dictionary): Dictonary of (key, value) arguments to be used by the controller.
                           May include: (consider_obstacles, true/false) - Enable consideration of obstacles
                                        (proximity_threshold, distance)  - Distance in front of actor in which
                                                                           obstacles are considered
                                        (attach_camera, true/false)      - Attach OpenCV display to actor
                                                                           (useful for debugging)

    Attributes:

        _generated_waypoint_list (list of carla.Transform): List of target waypoints the actor
            should travel along. A waypoint here is of type carla.Transform!
            Defaults to [].
        _last_update (float): Last time step the update function (tick()) was called.
            Defaults to None.
        _consider_obstacles (boolean): Enable/Disable consideration of obstacles
            Defaults to False.
        _proximity_threshold (float): Distance in front of actor in which obstacles are considered
            Defaults to infinity.
        _cv_image (CV Image): Contains the OpenCV image, in case a debug camera is attached to the actor
            Defaults to None.
        _camera (sensor.camera.rgb): Debug camera attached to actor
            Defaults to None.
        _obstacle_sensor (sensor.other.obstacle): Obstacle sensor attached to actor
            Defaults to None.
        _obstacle_distance (float): Distance of the closest obstacle returned by the obstacle sensor
            Defaults to infinity.
        _obstacle_actor (carla.Actor): Closest obstacle returned by the obstacle sensor
            Defaults to None.
    """

    def __init__(self, actor, args=None):
        super(SimpleVehicleControlAdapted, self).__init__(actor, args)
        self._active_vehicle_light_states = []

    def run_step(self):
        """
        Execute on tick of the controller's control loop

        If _waypoints are provided, the vehicle moves towards the next waypoint
        with the given _target_speed, until reaching the final waypoint. Upon reaching
        the final waypoint, _reached_goal is set to True.

        If _waypoints is empty, the vehicle moves in its current direction with
        the given _target_speed.

        For further details see :func:`_set_new_velocity`
        """
        # Reset the vehicle light state (shut off the braking lights)
        self._set_vehicle_light_state(carla.VehicleLightState.NONE)

        if self._reached_goal:
            # Reached the goal, so stop
            self._set_vehicle_light_state(carla.VehicleLightState.Brake)
            self._actor.apply_control(carla.VehicleControl(brake=1.0))
            return

        if self._visualizer:
            self._visualizer.render()

        self._reached_goal = False

        if not self._waypoints:
            # No waypoints are provided, so we have to create a list of waypoints internally
            # get next waypoints from map, to avoid leaving the road
            self._reached_goal = False

            map_wp = None
            if not self._generated_waypoint_list:
                map_wp = CarlaDataProvider.get_map().get_waypoint(CarlaDataProvider.get_location(self._actor))
            else:
                map_wp = CarlaDataProvider.get_map().get_waypoint(self._generated_waypoint_list[-1].location)
            while len(self._generated_waypoint_list) < 50:
                map_wps = map_wp.next(2.0)
                if map_wps:
                    self._generated_waypoint_list.append(map_wps[0].transform)
                    map_wp = map_wps[0]
                else:
                    break

            # Remove all waypoints that are too close to the vehicle
            while (self._generated_waypoint_list and
                   self._generated_waypoint_list[0].location.distance(self._actor.get_location()) < 0.5):
                self._generated_waypoint_list = self._generated_waypoint_list[1:]

            direction_norm = self._set_new_velocity(self._offset_waypoint(self._generated_waypoint_list[0]))
            if direction_norm < 2.0:
                self._generated_waypoint_list = self._generated_waypoint_list[1:]
        else:
            # When changing from "free" driving without pre-defined waypoints to a defined route with waypoints
            # it may happen that the first few waypoints are too close to the ego vehicle for obtaining a
            # reasonable control command. Therefore, we drop these waypoints first.
            while self._waypoints and self._waypoints[0].location.distance(self._actor.get_location()) < 0.5:
                self._waypoints = self._waypoints[1:]

            self._reached_goal = False
            if not self._waypoints:
                self._reached_goal = True
            else:
                direction_norm = self._set_new_velocity(self._offset_waypoint(self._waypoints[0]))
                if direction_norm < 4.0:
                    self._waypoints = self._waypoints[1:]
                    if not self._waypoints:
                        self._reached_goal = True

    def _set_new_velocity(self, next_location):
        """
        Calculate and set the new actor veloctiy given the current actor
        location and the _next_location_

        If _consider_obstacles is true, the speed is adapted according to the closest
        obstacle in front of the actor, if it is within the _proximity_threshold distance.
        If _consider_trafficlights is true, the vehicle will enforce a stop at a red
        traffic light.
        If _max_deceleration is set, the vehicle will reduce its speed according to the
        given deceleration value.
        If the vehicle reduces its speed, braking lights will be activated.

        Args:
            next_location (carla.Location): Next target location of the actor

        returns:
            direction (carla.Vector3D): Length of direction vector of the actor
        """

        current_time = GameTime.get_time()
        target_speed = self._target_speed

        if not self._last_update:
            self._last_update = current_time

        current_speed = math.sqrt(self._actor.get_velocity().x**2 + self._actor.get_velocity().y**2)

        if self._consider_obstacles:
            # If distance is less than the proximity threshold, adapt velocity
            if self._obstacle_distance < self._proximity_threshold:
                distance = max(self._obstacle_distance, 0)
                if distance > 0:
                    current_speed_other = math.sqrt(
                        self._obstacle_actor.get_velocity().x**2 + self._obstacle_actor.get_velocity().y**2)
                    if current_speed_other < current_speed:
                        acceleration = -0.5 * (current_speed - current_speed_other)**2 / distance
                        target_speed = max(acceleration * (current_time - self._last_update) + current_speed, 0)
                else:
                    target_speed = 0

        if self._consider_traffic_lights:
            if (self._actor.is_at_traffic_light() and
                    self._actor.get_traffic_light_state() == carla.TrafficLightState.Red):
                target_speed = 0

        if target_speed < current_speed:
            self._set_vehicle_light_state(carla.VehicleLightState.Brake)
            if self._max_deceleration is not None:
                target_speed = max(target_speed, current_speed - (current_time -
                                                                  self._last_update) * self._max_deceleration)
        else:
            if self._max_acceleration is not None:
                tmp_speed = min(target_speed, current_speed + (current_time -
                                                               self._last_update) * self._max_acceleration)
                # If the tmp_speed is < 0.5 the vehicle may not properly accelerate.
                # Therefore, we bump the speed to 0.5 m/s if target_speed allows.
                target_speed = max(tmp_speed, min(0.5, target_speed))

        # set new linear velocity
        velocity = carla.Vector3D(0, 0, 0)
        direction = next_location - CarlaDataProvider.get_location(self._actor)
        direction_norm = math.sqrt(direction.x**2 + direction.y**2)
        velocity.x = direction.x / direction_norm * target_speed
        velocity.y = direction.y / direction_norm * target_speed

        self._actor.set_target_velocity(velocity)

        # set new angular velocity
        current_yaw = CarlaDataProvider.get_transform(self._actor).rotation.yaw
        # When we have a waypoint list, use the direction between the waypoints to calculate the heading (change)
        # otherwise use the waypoint heading directly
        if self._waypoints:
            delta_yaw = math.degrees(math.atan2(direction.y, direction.x)) - current_yaw
        else:
            new_yaw = CarlaDataProvider.get_map().get_waypoint(next_location).transform.rotation.yaw
            delta_yaw = new_yaw - current_yaw

        if math.fabs(delta_yaw) > 360:
            delta_yaw = delta_yaw % 360

        if delta_yaw > 180:
            delta_yaw = delta_yaw - 360
        elif delta_yaw < -180:
            delta_yaw = delta_yaw + 360

        angular_velocity = carla.Vector3D(0, 0, 0)
        if target_speed == 0:
            angular_velocity.z = 0
        else:
            angular_velocity.z = delta_yaw / (direction_norm / target_speed)
        self._actor.set_target_angular_velocity(angular_velocity)

        self._last_update = current_time

        return direction_norm

    def _set_vehicle_light_state(self, light_state: carla.VehicleLightState) -> None:
        if light_state == carla.VehicleLightState.NONE:
            self._active_vehicle_light_states = []
        else:
            if carla.VehicleLightState.NONE in self._active_vehicle_light_states:
                self._active_vehicle_light_states.remove(carla.VehicleLightState.NONE)

        if light_state not in self._active_vehicle_light_states:
            self._active_vehicle_light_states.append(light_state)

        final_light_state = reduce(lambda state_one, state_two: state_one | state_two,
                                   self._active_vehicle_light_states)
        self._actor.set_light_state(carla.VehicleLightState(final_light_state))
