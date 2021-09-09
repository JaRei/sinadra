#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
from actor_situation_class_detection.situation_class import RoadSegmentEnd, TwoLaneFollowingSituationClass, \
    UnsignalizedFourWayJunction
from actor_situation_class_detection.situation_class_state_machine import SituationClassStateMachine


class SituationClassStateMachineTown03(SituationClassStateMachine):
    """
    This class represents an initialized situation class state machine for CARLA Town03.
    """

    def __init__(self):
        possible_situation_classes = self._create_situation_classes()
        super().__init__(possible_situation_classes)

    def _create_situation_classes(self):
        situation_classes = []

        # create all situation classes
        sit_class_two_lane_following_sc1 = self._create_two_lane_following_sc1()
        sit_class_unsignalized_four_way_crossing_sc3 = self._create_unsignalized_four_way_crossing_sc3()
        sit_class_two_lane_following_sc4 = self._create_two_lane_following_sc4()

        # link situation classes with as next and previous classes

        # SC1 2 lane following situation class: south entry point to unsignalized 4 way crossing (SC3)
        sc1_next_situation_classes_by_road_segment_end = {
            RoadSegmentEnd.B: [sit_class_unsignalized_four_way_crossing_sc3]
        }
        sit_class_two_lane_following_sc1.set_possible_next_situation_classes_by_road_segment_end(
            sc1_next_situation_classes_by_road_segment_end)

        # SC3 unsignalized 4 way crossing situation class
        sc3_next_situation_classes_by_road_segment_end = {
            RoadSegmentEnd.C: [sit_class_two_lane_following_sc1]
        }
        sit_class_unsignalized_four_way_crossing_sc3.set_possible_next_situation_classes_by_road_segment_end(
            sc3_next_situation_classes_by_road_segment_end)

        sc3_previous_situation_classes_by_road_segment_end = {
            RoadSegmentEnd.C: [sit_class_two_lane_following_sc1],
            RoadSegmentEnd.B: [sit_class_two_lane_following_sc4]
        }
        sit_class_unsignalized_four_way_crossing_sc3.set_previous_situation_classes_by_road_segment_end(
            sc3_previous_situation_classes_by_road_segment_end)

        # SC4 2 lane following situation class: east entry point to unsignalized 4 way crossing (SC3)
        sc4_next_situation_classes_by_road_segment_end = {
            RoadSegmentEnd.B: [sit_class_unsignalized_four_way_crossing_sc3]
        }

        sit_class_two_lane_following_sc4.set_possible_next_situation_classes_by_road_segment_end(
            sc4_next_situation_classes_by_road_segment_end)

        # add created situation classes to list
        situation_classes.append(sit_class_two_lane_following_sc1)
        situation_classes.append(sit_class_unsignalized_four_way_crossing_sc3)
        situation_classes.append(sit_class_two_lane_following_sc4)
        return situation_classes

    def _create_two_lane_following_sc1(self):

        """ Carla coordinates
        sc1_coordinates = [
            (-59.677048, -117.748764),  # top right
            (-82.056221, -117.778320),  # top between north and south way
            (-82.056221, -8.005962),  # bottom between north and south way
            (-59.844963, -8.005962)  # bottom right
        ]
        left_lane_coordinates = [
            (-79.846626, -117.748764),  # top left
            (-79.550591, -8.005962),  # bottom left
            (-76.228622, -8.005962),  # bottom right
            (-76.392159, -117.748764)  # top right
        ]
        right_lane_coordinates = [
            (-76.392159, -117.748764),  # top left
            (-76.228622, -8.005962),  # bottom left
            (-72.610268, -8.005962),  # bottom right
            (-72.976341, -117.748764)  # top right
        ]"""
        # OpenDrive coordinates
        sc1_coordinates = [
            (-59.677048, 117.748764),  # top right
            (-82.056221, 117.778320),  # top between north and south way
            (-82.056221, 8.005962),  # bottom between north and south way
            (-59.844963, 8.005962)  # bottom right
        ]
        left_lane_coordinates = [
            (-79.846626, 117.748764),  # top left
            (-79.550591, 8.005962),  # bottom left
            (-76.228622, 8.005962),  # bottom right
            (-76.392159, 117.748764)  # top right
        ]
        right_lane_coordinates = [
            (-76.392159, 117.748764),  # top left
            (-76.228622, 8.005962),  # bottom left
            (-72.610268, 8.005962),  # bottom right
            (-72.976341, 117.748764)  # top right
        ]
        sit_class_sc1 = TwoLaneFollowingSituationClass("SC1",
                                                       sc1_coordinates,
                                                       left_lane_coordinates,
                                                       right_lane_coordinates)
        return sit_class_sc1

    def _create_unsignalized_four_way_crossing_sc3(self):
        """Carla coordinates
        sc3_coordinates = [
            (-59.677048, -117.748764),  # bottom right
            (-103.239853, -117.778320),  # bottom left
            (-103.239853, -155.401031),  # top left
            (-59.391304, -154.834015)]  # top right"""
        #OpenDrive coordinates
        sc3_coordinates = [
            (-59.677048, 117.748764),  # bottom right
            (-103.239853, 117.778320),  # bottom left
            (-103.239853, 155.401031),  # top left
            (-59.391304, 154.834015)]  # top right

        sit_class_sc3 = UnsignalizedFourWayJunction("SC3", sc3_coordinates)
        return sit_class_sc3

    def _create_two_lane_following_sc4(self):
        """Carla coordinates
        sc4_coordinates = [
            (-15.799465, -136.851822),  # bottom right
            (-59.391304, -137.856964),  # bottom left
            (-59.391304, -154.834015),  # top left
            (-15.799465, -154.834015)]  # top right

        right_lane_coordinates = [
            (-59.391304, -144.838501),  # top left
            (-59.391304, -141.465286),  # bottom left -> top left of left lane
            (-15.799465, -140.587616),  # bottom right -> top right of left lane
            (-15.799465, -143.862487)  # top right
        ]

        left_lane_coordinates = [
            (-59.391304, -141.465286),  # top left -> bottom left of right lane
            (-59.391304, -137.803009),  # bottom left
            (-15.799465, -136.860580),  # bottom right
            (-15.799465, -140.587616)  # top right -> bottom right of right lane
        ]"""
        # OpenDrive coordinates
        sc4_coordinates = [
            (-15.799465, 136.851822),  # bottom right
            (-59.391304, 137.856964),  # bottom left
            (-59.391304, 154.834015),  # top left
            (-15.799465, 154.834015)]  # top right

        right_lane_coordinates = [
            (-59.391304, 144.838501),  # top left
            (-59.391304, 141.465286),  # bottom left -> top left of left lane
            (-15.799465, 140.587616),  # bottom right -> top right of left lane
            (-15.799465, 143.862487)  # top right
        ]

        left_lane_coordinates = [
            (-59.391304, 141.465286),  # top left -> bottom left of right lane
            (-59.391304, 137.803009),  # bottom left
            (-15.799465, 136.860580),  # bottom right
            (-15.799465, 140.587616)  # top right -> bottom right of right lane
        ]

        sit_class_sc4 = TwoLaneFollowingSituationClass("SC4",
                                                       sc4_coordinates,
                                                       left_lane_coordinates,
                                                       right_lane_coordinates)
        return sit_class_sc4
