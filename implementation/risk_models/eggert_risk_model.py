#################### BEGIN LICENSE BLOCK ###############################
#
# Copyright (C) 2021 Fraunhofer IESE
#
# SPDX-License-Identifier: LGPL-2.1-only
#
#################### END LICENSE BLOCK #################################
import math
import numpy as np
from scipy.integrate import simps

from sinadra_configuration_parameters import EGGERT_BETA, EGGERT_RATE_MAX

##########################################################################################
# Eggert Risk Model
##########################################################################################


def eggert_risk(ego_pos_mean, ego_pos_std, fv_pos_mean, fv_pos_std, time_inc):
    mean_d_t = fv_pos_mean - ego_pos_mean
    sig_d_t = np.sqrt((ego_pos_std * ego_pos_std) + (fv_pos_std * fv_pos_std))
    # print(mean_d_t)
    # print(sig_d_t)
    risk_ind = np.empty(len(mean_d_t))
    event_prob = np.empty(len(mean_d_t))
    for i in range(0, len(mean_d_t)):
        risk_ind[i] = risk_indicator(mean_d_t[i], sig_d_t[i])
        event_prob[i] = event_rate(risk_ind[i])
    # print(risk_ind)
    # print(event_prob)
    cumulative_survival = np.empty(len(event_prob))
    survival_prob = np.empty(len(event_prob))
    collision_prob = np.empty(len(event_prob))
    for i in range(0, len(event_prob)):
        x = np.linspace(0, i * time_inc, i + 1)
        cumulative_survival[i] = simps(event_prob[:(i + 1)], x)
        survival_prob[i] = math.exp(-cumulative_survival[i])
        collision_prob[i] = event_prob[i] * survival_prob[i]
    return collision_prob


def risk_indicator(mean_d_t, sig_d_t):
    thresh_crit = 0  # All distances below 0m are deemed critical as this means a collision occured
    return 0.5 * (math.erf((thresh_crit - mean_d_t) / math.sqrt(2.0 * sig_d_t ** 2.0)) + 1)


def event_rate(risk_ind):
    return (1 / EGGERT_RATE_MAX) * ((1 - math.exp(-EGGERT_BETA * risk_ind)) / (1 - math.exp(-EGGERT_BETA)))
