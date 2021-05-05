#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
import multiprocessing
import signal
import cv2
import weakref
import atexit
import matplotlib.pyplot as plt
import math
from time import sleep
from typing import TYPE_CHECKING
from pygame.time import Clock

from sinadra import *
from actor_situation_class_detection.town_data.town03_sinadra_data import SituationClassStateMachineTown03
from actor_situation_class_detection.bayesian_network_id_selection.bayesian_network_id_selector import \
    BayesianNetworkIdSelector
from actor_situation_class_detection.vehicle_actor_wrapper import get_vehicles, \
    get_wrapped_vehicles
from sinadra import evaluate_bayesian_networks, compute_total_risk
from sinadra_configuration_parameters import *
from trajectory_gen.long_traj_generator import gen_constant_accel
from util.kinematic_transform import *

import carla

if TYPE_CHECKING:
    from bayesian_network.inference.interfaces import BayesianNetworkOutput


class SinadraClient:

    def __init__(self):
        '''
        :return:
        '''

        self.cycle_counter = SKIP_CYCLE_COUNT

        # Initialize Carla Client

        carla_client = carla.Client(CARLA_SERVER_HOST, CARLA_SERVER_PORT)
        carla_world = carla_client.get_world()

        synchronization_clock = self._set_up_synchronization(carla_world)

        if carla_world.get_map().name != "Town03":
            carla_client.load_world("Town03")

        self._set_carla_spectator_for_lane_following(carla_world)
        self.hero_camera: Optional[carla.Sensor] = None
        self._cv_image: Optional["numpy.ndarray"] = None
        self._cv_window_name = "SINADRA Risk Sensor"
        cv2.namedWindow(self._cv_window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._cv_window_name, (800, 1200))

        town03_state_machine = SituationClassStateMachineTown03()

        carla_map = carla_world.get_map()

        # Dynamic risk plot initialization

        self.fig_risk, self.ax_risk = plt.subplots()
        self.ax_risk.axis([0, PREDICTION_HORIZON, 0, 1])
        self.fig_risk.suptitle('Total Risk')
        self.ax_risk.set_xlabel("Time")
        self.ax_risk.set_ylabel("Collision Probability")

        self.ax_risk.set_ylim([-0.1,1.1])

        self.x = np.arange(0, PREDICTION_HORIZON + PREDICTION_TIMESTEP, PREDICTION_TIMESTEP)
        self.fv_risk_x_value = np.zeros(len(self.x))
        self.sv_risk_x_value = np.zeros(len(self.x))
        self.sv_risk_y_value = np.zeros(len(self.x))

        self.fv_risk_line, = self.ax_risk.plot(self.x, self.fv_risk_x_value, label="FV Braking Risk")
        self.sv_risk_x_line, = self.ax_risk.plot(self.x, self.sv_risk_x_value, label="SV Cut-In Long Risk")
        self.sv_risk_y_line, = self.ax_risk.plot(self.x, self.sv_risk_y_value, label="SV Cut-In Lat Risk")
        self.ax_risk.legend(loc="best")

        processes_number_bn_inference = NUMBER_OF_PROCESSES_FOR_THE_BN_INFERENCE
        processes_number_bn_inference = processes_number_bn_inference if processes_number_bn_inference else 2
        self.bn_inference_multiprocessing_pool = multiprocessing.Pool(processes=processes_number_bn_inference,
                                                                      initializer=self._worker_init)

        # This while loop is the execution loop of the SINADRA client (!= the execution loop of CARLA)
        # Therefore it only controls the update of the risk plot. Carla-related work is triggered over the game loop.
        while True:
            carla_world.tick()

            self.game_loop(carla_world, carla_map, town03_state_machine)

            if self._cv_image is not None:
                # Update dynamic risk plot with currently stored total risk array
                self.fig_risk.canvas.flush_events()
                # Prediction horizon and timestep are constant over scenario -> not necessary to reset x axis
                # self.risk_line.set_xdata(self.x)
                self.fv_risk_line.set_ydata(self.fv_risk_x_value)
                self.sv_risk_x_line.set_ydata(self.sv_risk_x_value)
                self.sv_risk_y_line.set_ydata(self.sv_risk_y_value)
                self.fig_risk.canvas.draw()
                img_risk = np.fromstring(self.fig_risk.canvas.tostring_rgb(), dtype=np.uint8, sep='')
                img_risk = img_risk.reshape(self.fig_risk.canvas.get_width_height()[::-1] + (3,))
                img_risk = cv2.resize(img_risk, (800, 600), interpolation=cv2.INTER_AREA)
                merged_cv_image = cv2.vconcat([self._cv_image, img_risk])
                cv2.imshow(self._cv_window_name, merged_cv_image)
                cv2.waitKey(1)

            synchronization_clock.tick(FRAMERATE)

    def _set_up_synchronization(self, carla_world: carla.World) -> Clock:
        synchronization_clock = Clock()

        self._set_up_synchronous_carla_mode(carla_world)
        atexit.register(self._set_up_asynchronous_carla_mode, carla_world)

        return synchronization_clock

    @staticmethod
    def _set_up_synchronous_carla_mode(carla_world: carla.World) -> None:
        settings = carla_world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1 / FRAMERATE
        carla_world.apply_settings(settings)

    @staticmethod
    def _set_up_asynchronous_carla_mode(carla_world: carla.World) -> None:
        settings = carla_world.get_settings()
        settings.synchronous_mode = False
        carla_world.apply_settings(settings)

    def __del__(self):
        # Release multiprocessing resources
        self.bn_inference_multiprocessing_pool.close()
        del self.bn_inference_multiprocessing_pool

        cv2.destroyAllWindows()

    def _worker_init(self):
        # This method is needed for parallel processing based on pools
        # SIGINT (=CTRL+C) will be handled by SIG_IGN (=Handler that ignores the signal)
        signal.signal(signal.SIGINT, signal.SIG_IGN)

    @staticmethod
    def _set_carla_spectator_for_lane_following(carla_world):
        carla_spectator = carla_world.get_spectator()

        # 0 -> Top View on Lane Following Scenarios
        # 1 -> Angled Side View on Lane Following Scenarios (observing indicator)
        view_choice = 1
        if view_choice:  # side angle to view the indicator
            target_location = carla.Location(x=-73.534294, y=-41.217567, z=3.980349)
            target_rotation = carla.Rotation(pitch=-16.670803, yaw=-105.755310, roll=0.000033)
        else:  # above the lane following scenario
            target_location = carla.Location(x=-81.5, y=-66.5, z=70.0)
            target_rotation = carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0)

        target_transform = carla.Transform(target_location, target_rotation)
        carla_spectator.set_transform(target_transform)

    def set_up_carla_camera(self, carla_world):
        camera_actor_bp = carla_world.get_blueprint_library().find("sensor.camera.rgb")
        # camera_transform = carla.Transform(carla.Location(x=-10, z=5), carla.Rotation(pitch=-20))
        target_location = carla.Location(x=-81.5, y=-66.5, z=70.0)  # above the lane following scenarios
        target_roation = carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0)
        target_transform = carla.Transform(target_location, target_roation)
        # self.hero_camera = carla_world.spawn_actor(camera_actor_bp, camera_transform,
        #                                            attach_to=hero_vehicle)
        self.hero_camera = carla_world.spawn_actor(camera_actor_bp, target_transform)
        weak_self = weakref.ref(self)
        self.hero_camera.listen(lambda image: self.camera_callback(weak_self, image))

    @staticmethod
    def camera_callback(weak_self, sensor_data_image: carla.Image) -> None:
        self = weak_self()
        image_data = np.frombuffer(sensor_data_image.raw_data, dtype=np.dtype("uint8"))
        np_image = np.reshape(image_data, (sensor_data_image.height, sensor_data_image.width, 4))
        np_image = np_image[:, :, :3]
        np_image = np_image[:, :, ::-1]
        self._cv_image = cv2.cvtColor(np_image, cv2.COLOR_BGR2RGB)

        # =============  Game Loop
    
    def game_loop(self, carla_world, carla_map, state_machine):
        spectator_mode = False

        if self.cycle_counter % SKIP_CYCLE_COUNT > 0:
            self.cycle_counter += 1
            return

        self.cycle_counter += 1

        print("====================================================\nStart Tick\n\n")
        start = time.time()

        carla_debug_helper = carla_world.debug

        if spectator_mode:
            # spectator output
            spectator = carla_world.get_spectator()
            spectator_transform = spectator.get_transform()
            spectator_location = spectator_transform.location
            spectator_rotation = spectator_transform.rotation
            print(f'Spectator location = {spectator_location} \nSpectator rotation = {spectator_rotation}')

        else:
            (other_vehicles, hero_vehicle) = get_vehicles(carla_world)

            if hero_vehicle is not None:

                # Set up camera
                if not self.hero_camera:
                    self.set_up_carla_camera(carla_world)

                ####################################
                # Classify actors in situation class
                ####################################

                # remove all actors from situation classes
                state_machine.clear_actor_associations_from_situation_classes()

                # wrap vehicles
                all_wrapped_vehicles, wrapped_hero_vehicle, wrapped_other_vehicles = (
                    get_wrapped_vehicles(hero_vehicle, other_vehicles))

                # update situation classes with given vehicles
                state_machine.update_situation_classes(all_wrapped_vehicles)

                # retrieve situation class the hero vehicle is currently in
                hero_situation_class = state_machine.get_hero_situation_class()
                self.log_hero_sit_class_info(wrapped_hero_vehicle, hero_situation_class)

                #########################################################
                # Bayesian Network Inference for actors relevant for ego
                #########################################################

                # based on situation class and other vehicles, determine, which BN shall be evaluated
                bn_selector = BayesianNetworkIdSelector(wrapped_hero_vehicle, hero_situation_class, 
                                                        wrapped_other_vehicles, carla_map, carla_debug_helper)
                vehicle_dependent_bn_ids = bn_selector.get_vehicle_dependent_bn_ids()
                self.log_active_bn_info(vehicle_dependent_bn_ids)

                if not any(vehicle_dependent_bn_ids):
                    print("no relevant vehicles, for which risk can be computed")
                    return

                # Infer behavior likelihoods in Bayesian Networks
                infered_bn_outputs = evaluate_bayesian_networks(self.bn_inference_multiprocessing_pool,
                                                                vehicle_dependent_bn_ids, carla_world, carla_map)
                self.log_bayesian_network_inference_outputs(infered_bn_outputs)

                ##########################################################################################
                # Trajectory sampling for relevant behaviors of other actors and pair-wise risk assessment
                ##########################################################################################

                for network_output in infered_bn_outputs:
                    for node in network_output.output_nodes:
                        behavior_type = BEHAVIOR_TYPE_MAPPING[node.title]

                        #############################
                        # Ego Trajectory Generation
                        #############################

                        # Generate ego trajectory prediction (Constant Acceleration)

                        # Positions of the following are relative to the ego vehicle center point, i.e. pos = (0,0)
                        ego_pos_center, ego_speed, ego_accel, ego_np_vec_center = transform_ego_kinematics_to_ego_frame(
                            hero_vehicle)

                        # Add half of the ego vehicle length to move the position to the ego's front end
                        # This is relevant to get the "correct" distances for risk analysis
                        ego_np_vec_front = ego_np_vec_center
                        ego_half_length = hero_vehicle.bounding_box.extent.x  # is already half of the vehicle length
                        ego_np_vec_front[0] += ego_half_length
                        ego_pos_front = ego_pos_center
                        ego_pos_front.x += ego_half_length

                        ego_pos_x_mean, ego_pos_x_std = gen_constant_accel(ego_np_vec_front, NUM_TRAJECTORIES,
                                                                       PREDICTION_HORIZON, PREDICTION_TIMESTEP)

                        #############################################
                        # Front Vehicle Brake Behavior Prediction
                        #############################################
                        if behavior_type == BehaviorType.Braking:
                            # Relative braking behavior likelihoods
                            emergency_prob = node.outcomes[0].value
                            targetbrake_prob = node.outcomes[1].value
                            followvehicle_prob = node.outcomes[2].value
                            nobrake_prob = node.outcomes[3].value

                            # Get actor reference of front vehicle
                            vehicle_id = network_output.vehicle_id
                            front_vehicle = filter(lambda v: v.id == vehicle_id, other_vehicles)
                            front_vehicle = next(front_vehicle, None)

                            if not front_vehicle:
                                return

                            brake_behavior_risks = list()

                            # Position of the following call is the position of the front vehicle's center point
                            # relative to the ego vehicle's center point
                            fv_pos_center, fv_speed, fv_accel, fv_np_vec_center = transform_actor_kinematics_to_ego_frame(hero_vehicle, front_vehicle)

                            # We want to get the x location of the front vehicle's rear end relative to the ego
                            # vehicle's front end to get the correct distance. Subtract half of the lengths of
                            # both vehicles.
                            fv_half_length = front_vehicle.bounding_box.extent.x  # is already half of the vehicle length
                            fv_np_vec_rear = fv_np_vec_center
                            fv_pos_rear = fv_pos_center
                            fv_np_vec_rear[0] -= fv_half_length + ego_half_length
                            fv_pos_rear.x -= fv_half_length + ego_half_length

                            # Adapt Front Vehicle Position to align with the ego vehicle
                            # (if it is executing a lane change)
                            if (network_output.vehicle_situation_state_id ==
                                    BayesianNetId.TWO_LANE_FOLLOWING_FRONT_VEHICLE_IN_LANE_CHANGE):
                                fv_pos_rear.y = 0
                                fv_speed.lat = 0
                                fv_accel.lat = 0
                                fv_np_vec_rear[1], fv_np_vec_rear[3], fv_np_vec_rear[5] = 0, 0, 0

                            ###########################
                            # Emergency Brake Behavior Risk
                            ###########################
                            if emergency_prob > 0:
                                emergency_brake_eggert_prob = emergency_brake_risk(ego_pos_x_mean, ego_pos_x_std,
                                                                                   fv_np_vec_rear)
                                brake_behavior_risks.append((emergency_brake_eggert_prob, emergency_prob))

                            ###########################
                            # Target Brake Behavior Risk
                            ###########################
                            if targetbrake_prob > 0:
                                # This is the global coordinate of the concrete stop line used in the Town03 scenario
                                # TODO needs to be made dynamic by using the waypoint API
                                stop_line_pos_g: carla.Location = carla.Location(-74.8, -127, -0.1)
                                # The following position is relative to the ego vehicle's center point
                                stop_line_pos_e: Pose = transform_global_pos_to_ego_frame(hero_vehicle, stop_line_pos_g)

                                # The target distance should be the distance between front vehicle front end
                                # and the stop line
                                fv_frontend_x = fv_np_vec_center[0] + fv_half_length
                                target_distance = stop_line_pos_e.x - fv_frontend_x

                                target_brake_eggert_prob = target_brake_risk(ego_pos_x_mean, ego_pos_x_std,
                                                                             fv_np_vec_rear, target_distance,
                                                                             BRAKE_TARGET_SAFE_DISTANCE_MARGIN)
                                brake_behavior_risks.append((target_brake_eggert_prob, targetbrake_prob))


                            ###############################################################
                            # Intelligent Driver Model Behavior Risk (Follow Vehicle and Follow Road)
                            ###############################################################
                            if followvehicle_prob > 0 or nobrake_prob > 0:

                                # Role = adversary or adversary1 -> Vehicle that spawned directly in front of the ego
                                # vehicle (specific for our scenarios)
                                # Role = adversary2 -> Vehicle that spawned either in front of the adversary(1) vehicle
                                # or on the neighboring lane
                                front_role = front_vehicle.attributes["role_name"]

                                if (front_role == "adversary1") or (front_role == "adversary"):
                                    # in the case that the adversary(1) vehicle is directly in front of the ego vehicle
                                    # the adversary2 vehicle can either be in front of the adversary(1) vehicle or
                                    # on the neighboring lane
                                    fv_front = filter(lambda v: v.attributes["role_name"] == "adversary2",
                                                      other_vehicles)
                                    front_front_vehicle = next(fv_front, None) if fv_front else None
                                    if front_front_vehicle:
                                        # Check whether the vehicles are on the same lane
                                        # (specific for our lane following scenario location)
                                        # TODO Coordinate transformation
                                        if math.fabs(front_vehicle.get_location().x
                                                     - front_front_vehicle.get_location().x) >= 1.5:
                                            front_front_vehicle = None
                                elif front_role == "adversary2":
                                    # in the case that the adversary2 vehicle is directly in front of the ego vehicle
                                    # the adversary1 vehicle is always in front of the adversary2 vehicle
                                    fv_front = filter(lambda v: v.attributes["role_name"] == "adversary1",
                                                      other_vehicles)
                                    front_front_vehicle = next(fv_front, None) if fv_front else None

                                if not front_front_vehicle:
                                    # print("No vehicle in front of the considered front vehicle exists")

                                    # No vehicle in front of the front vehicle exists in groundtruth.
                                    # Since the BN predicts a non-zero prob of follow vehicle behavior due to
                                    # uncertainties, we assume a vehicle to be in front with a time gap of 1,5s
                                    # and same speed like FV for the trajectory generation.

                                    # Time gap needs to be determined between front vehicle front end and
                                    # front front vehicle rear end

                                    fv_front_pos_center = fv_np_vec_center[0] + fv_np_vec_center[2] * IDM_TIME_GAP_FRONT_VEHICLE
                                    fv_front_pos_rear_x = fv_front_pos_center - (fv_half_length + ego_half_length)
                                    fv_front_pos_rear = Pose(x=fv_front_pos_rear_x, y=fv_np_vec_center[1])
                                    fv_front_speed = fv_speed
                                    fv_front_np_vec_rear = np.array([fv_front_pos_rear_x, fv_np_vec_center[1], fv_front_speed.long, fv_front_speed.lat, 0.0, 0.0])
                                else:
                                    # Position of the front front vehicle's center point relative to the ego
                                    # vehicle's center point
                                    fv_front_pos_center, fv_front_speed, _, fv_front_np_vec_center = transform_actor_kinematics_to_ego_frame(hero_vehicle, front_front_vehicle)

                                    # What we need for the IDM is the position of front front vehicle's rear end
                                    # relative to the ego vehicle's front end
                                    fv_front_pos_rear = fv_front_pos_center
                                    fv_front_np_vec_rear = fv_front_np_vec_center

                                    fv_front_half_length = front_front_vehicle.bounding_box.extent.x  # is already half of the vehicle length
                                    fv_front_np_vec_rear[0] -= fv_front_half_length + ego_half_length
                                    fv_front_pos_rear.x -= fv_front_half_length + ego_half_length

                                # self.log_kinematic_info(ego_pos, ego_speed, fv_front_pos, fv_front_speed, fv_pos, fv_speed)

                                idm_eggert_prob = idm_risk(ego_pos_x_mean, ego_pos_x_std, fv_np_vec_rear, 2 * fv_half_length, fv_front_np_vec_rear)
                                brake_behavior_risks.append((idm_eggert_prob, followvehicle_prob+nobrake_prob))

                            # Weight the individual behavior risk scores based on BN output likelihoods
                            weighted_total_risk = compute_total_risk(brake_behavior_risks)

                            # Update synced datastructure being used for updating the dynamic risk plot
                            self.fv_risk_x_value = weighted_total_risk

                        #############################################
                        # Side Vehicle Cutin From Right Behavior Prediction
                        #############################################
                        elif behavior_type == BehaviorType.LaneChangeToLeft:
                            # TODO When scenarios are considered, where lane changes from right to left are done
                            # Basically the code from LaneChangeToRight branch needs to be slightly adapted
                            pass
                        #############################################
                        # Side Vehicle Cutin From Left Behavior Prediction
                        #############################################
                        elif behavior_type == BehaviorType.LaneChangeToRight:
                            # Relative cutin behavior likelihoods
                            cutin_prob = node.outcomes[0].value
                            nocutin_prob = 1.0 - cutin_prob

                            # Get actor reference of side vehicle
                            vehicle_id = network_output.vehicle_id
                            side_vehicle = filter(lambda v: v.id == vehicle_id, other_vehicles)
                            side_vehicle = next(side_vehicle, None)

                            if not side_vehicle:
                                return

                            # Note that all computations are performed in the local ego coordinate frame
                            # TODO Think about whether to use rear end of side vehicle
                            sv_pos, sv_speed, sv_accel, sv_np_vec = transform_actor_kinematics_to_ego_frame(hero_vehicle, side_vehicle)

                            # End Point of lane change behavior [x,y]
                            lc_target = [ego_np_vec_front[0] + LC_CUTIN_DISTANCE_FROM_EGO, ego_np_vec_front[1]]

                            # print("Side Vehicle")
                            # print(sv_np_vec)
                            # print("Ego Vehicle")
                            # print(ego_np_vec_front)
                            # print("LC Target")
                            # print(lc_target)

                            # Generate ego lateral positions for a straight drive
                            num_positions = int((PREDICTION_HORIZON / PREDICTION_TIMESTEP + 1))
                            ego_pos_y_mean = np.zeros(num_positions)
                            ego_pos_y_std = np.full(num_positions, EGO_POS_LAT_STD)  # Assumed standard dev of ego lat pos = 0.2

                            collision_prob_x, collision_prob_y = \
                                lc_right_risk(ego_pos_x_mean, ego_pos_x_std, ego_pos_y_mean, ego_pos_y_std, sv_np_vec, lc_target)
                            # print("Collision Probability X")
                            # print(collision_prob_x)
                            # print(collision_prob_y)

                            lc_behavior_long_risk = list()
                            lc_behavior_lat_risk = list()
                            # Assuming that only cut-in of side vehicle into ego lane is critical --> Zero Risk
                            # This is important for computing the weighted total risk
                            lc_behavior_long_risk.append((list(np.zeros(len(self.x))), nocutin_prob))
                            lc_behavior_long_risk.append((collision_prob_x, cutin_prob))
                            # Weight the individual behavior risk scores based on BN output likelihoods
                            weighted_total_long_risk = compute_total_risk(lc_behavior_long_risk)

                            # Assuming that only cut-in of side vehicle into ego lane is critical --> Zero Risk
                            # This is important for computing the weighted total risk
                            lc_behavior_lat_risk.append((list(np.zeros(len(self.x))), nocutin_prob))
                            lc_behavior_lat_risk.append((collision_prob_y, cutin_prob))
                            # Weight the individual behavior risk scores based on BN output likelihoods
                            weighted_total_lat_risk = compute_total_risk(lc_behavior_lat_risk)

                            # Update synced datastructure being used for updating the dynamic risk plot
                            self.sv_risk_x_value = weighted_total_long_risk
                            self.sv_risk_y_value = weighted_total_lat_risk
                        else:
                            pass

            else:
                self.hero_camera = None
                print("No hero vehicle detected")
        end = time.time()
        print("\n\nEnd Tick (Exec Time="+str(end-start)+"\n====================================================")

    ###########
    # Logging
    ###########

    def log_hero_sit_class_info(self, wrapped_hero_vehicle, hero_situation_class):
        if hero_situation_class is None:
            print("Hero situation class = None")
        else:
            hero_situation_class.print_information(wrapped_hero_vehicle)


    def log_active_bn_info(self, vehicle_dependent_bn_ids):
        if any(vehicle_dependent_bn_ids):
            print('BN_IDs by vehicle:')
            for vehicle_dependent_bn_id in vehicle_dependent_bn_ids:
                print(f'Vehicle {vehicle_dependent_bn_id.wrapped_vehicle.vehicle} : BN_ID = {vehicle_dependent_bn_id.bn_id}')


    def log_bayesian_network_inference_outputs(self, infered_bn_outputs: List["BayesianNetworkOutput"]) -> None:
        print("\n---------------\nBayesian Network(s) Inference Outputs:\n")
        for bn_output in infered_bn_outputs:
            print(f"Vehicle (ID: {bn_output.vehicle_id}):\n{bn_output}\n")
            print("---------------\n")

    def log_kinematic_info(self, ego_pos, ego_speed, fv_front_pos, fv_front_speed, fv_pos, fv_speed):
        print("Ego Vehicle")
        print(ego_pos)
        print(ego_speed)
        print("Front Vehicle")
        print(fv_pos)
        print(fv_speed)
        print("Front Front Vehicle")
        print(fv_front_pos)
        print(fv_front_speed)


if __name__ == "__main__":
    SinadraClient()
