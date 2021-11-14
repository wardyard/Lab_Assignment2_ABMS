"""
File used to analyse the obtained output data
"""
import itertools as it

from bisect import bisect_left
from typing import List

import numpy as np
import pandas as pd
import scipy.stats as ss

from pandas import Categorical

def VD_A(treatment: List[float], control: List[float]):
    """
    Computes Vargha and Delaney A index
    A. Vargha and H. D. Delaney.
    A critique and improvement of the CL common language
    effect size statistics of McGraw and Wong.
    Journal of Educational and Behavioral Statistics, 25(2):101-132, 2000
    The formula to compute A has been transformed to minimize accuracy errors
    See: http://mtorchiano.wordpress.com/2014/05/19/effect-size-of-r-precision/
    :param treatment: a numeric list
    :param control: another numeric list
    :returns the value estimate and the magnitude
    """
    m = len(treatment)
    n = len(control)

    if m != n:
        raise ValueError("Data d and f must have the same length")

    r = ss.rankdata(treatment + control)
    r1 = sum(r[0:m])

    # Compute the measure
    # A = (r1/m - (m+1)/2)/n # formula (14) in Vargha and Delaney, 2000
    A = (2 * r1 - m * (m + 1)) / (2 * n * m)  # equivalent formula to avoid accuracy errors

    levels = [0.147, 0.33, 0.474]  # effect sizes from Hess and Kromrey, 2004
    magnitude = ["negligible", "small", "medium", "large"]
    scaled_A = (A - 0.5) * 2

    magnitude = magnitude[bisect_left(levels, abs(scaled_A))]
    estimate = A

    return estimate, magnitude


def VD_A_DF(data, val_col: str = None, group_col: str = None, sort=True):
    """
    :param data: pandas DataFrame object
        An array, any object exposing the array interface or a pandas DataFrame.
        Array must be two-dimensional. Second dimension may vary,
        i.e. groups may have different lengths.
    :param val_col: str, optional
        Must be specified if `a` is a pandas DataFrame object.
        Name of the column that contains values.
    :param group_col: str, optional
        Must be specified if `a` is a pandas DataFrame object.
        Name of the column that contains group names.
    :param sort : bool, optional
        Specifies whether to sort DataFrame by group_col or not. Recommended
        unless you sort your data manually.
    :return: stats : pandas DataFrame of effect sizes
    Stats summary ::
    'A' : Name of first measurement
    'B' : Name of second measurement
    'estimate' : effect sizes
    'magnitude' : magnitude
    """

    x = data.copy()
    if sort:
        x[group_col] = Categorical(x[group_col], categories=x[group_col].unique(), ordered=True)
        x.sort_values(by=[group_col, val_col], ascending=True, inplace=True)

    groups = x[group_col].unique()

    # Pairwise combinations
    g1, g2 = np.array(list(it.combinations(np.arange(groups.size), 2))).T

    # Compute effect size for each combination
    ef = np.array([VD_A(list(x[val_col][x[group_col] == groups[i]].values),
                        list(x[val_col][x[group_col] == groups[j]].values)) for i, j in zip(g1, g2)])

    return pd.DataFrame({
        'A': np.unique(data[group_col])[g1],
        'B': np.unique(data[group_col])[g2],
        'estimate': ef[:, 0],
        'magnitude': ef[:, 1]
    })




if __name__ == '__main__':

    # load data
    res_cbs_hi = pd.read_csv('Simulation results/results_cbs_hi.csv')
    res_cbs_lo = pd.read_csv('Simulation results/results_cbs_lo.csv')
    res_ind_hi = pd.read_csv('Simulation results/results_ind_hi.csv')
    res_ind_lo = pd.read_csv('Simulation results/results_ind_lo.csv')
    res_prio_hi = pd.read_csv('Simulation results/results_prio_hi.csv')
    res_prio_lo = pd.read_csv('Simulation results/results_prio_lo.csv')

    # load sensitivity analysis data for changes in observation size (Individual planning)
    res_obs_2_hi = res_ind_hi
    res_obs_3_hi = pd.read_csv('Sensitivity analysis/Observation size/results_ind_hi_obs_3.csv')
    res_obs_4_hi = pd.read_csv('Sensitivity analysis/Observation size/results_ind_hi_obs_4.csv')

    # load sensitivity analysis data for change in bidding decay factor (Individual planning)
    res_bid_11_hi = res_ind_hi
    res_bid_15_hi = pd.read_csv('Sensitivity analysis/Bidding factor/results_ind_hi_bid_15.csv')
    res_bid_20_hi = pd.read_csv('Sensitivity analysis/Bidding factor/results_ind_hi_bid_20.csv')
    res_bid_full_hi = pd.read_csv('Sensitivity analysis/Bidding factor/results_ind_hi_bid_full.csv')

    ####################################################################################################################
    # hypothesis 1: average travel time for CBS is higher than for Individual planning with a high arrival rate
    ####################################################################################################################
    A_travel_t_cbs_ind_hi = VD_A(list(res_cbs_hi['travel_t_avg']), list(res_ind_hi['travel_t_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average travel time: CBS vs Individual and high demand: '
          + str(A_travel_t_cbs_ind_hi))

    ####################################################################################################################
    # hypothesis 2: average travel time for CBS is higher than for Prioritised planning with a high arrival rate
    ####################################################################################################################
    A_travel_t_cbs_prio_hi = VD_A(list(res_cbs_hi['travel_t_avg']), list(res_prio_hi['travel_t_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average travel time: CBS vs Prioritised and high demand: '
          + str(A_travel_t_cbs_prio_hi))

    ####################################################################################################################
    # hypothesis 3: average travel time for Individual is higher than for Prioritised planning with a high arrival rate
    ####################################################################################################################
    A_travel_t_ind_prio_hi = VD_A(list(res_ind_hi['travel_t_avg']), list(res_prio_hi['travel_t_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average travel time: Individual vs Prioritised and high demand: '
          + str(A_travel_t_ind_prio_hi))

    ####################################################################################################################
    # hypothesis 4: average travel time for CBS is higher than for Idnividual planning with a low arrival rate
    ####################################################################################################################
    A_travel_t_cbs_ind_lo = VD_A(list(res_cbs_lo['travel_t_avg']), list(res_ind_lo['travel_t_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average travel time: CBS vs Individual and low demand: '
          + str(A_travel_t_cbs_ind_lo))

    ####################################################################################################################
    # hypothesis 5: average travel time for CBS is higher than for Prioritised planning with a low arrival rate
    ####################################################################################################################
    A_travel_t_cbs_prio_lo = VD_A(list(res_cbs_lo['travel_t_avg']), list(res_prio_lo['travel_t_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average travel time: CBS vs Prioritised and low demand: '
          + str(A_travel_t_cbs_prio_lo))

    ####################################################################################################################
    # hypothesis 6: average travel time for Individual is higher than for Prioritised planning with a low arrival rate
    ####################################################################################################################
    A_travel_t_ind_prio_lo = VD_A(list(res_ind_lo['travel_t_avg']), list(res_prio_lo['travel_t_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average travel time: Individual vs Prioritised and low demand: '
          + str(A_travel_t_ind_prio_lo))

    ####################################################################################################################
    # hypothesis 7: average travel distance for CBS is higher than for Individual planning with a high arrival rate
    ####################################################################################################################
    A_travel_d_cbs_ind_hi = VD_A(list(res_cbs_hi['travel_d_avg']), list(res_ind_hi['travel_d_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average travel distance: CBS vs Individual and high demand: '
          + str(A_travel_d_cbs_ind_hi))

    ####################################################################################################################
    # hypothesis 8: average travel distance for CBS is higher than for Prioritised planning with a high arrival rate
    ####################################################################################################################
    A_travel_d_cbs_prio_hi = VD_A(list(res_cbs_hi['travel_d_avg']), list(res_prio_hi['travel_d_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average travel distance: CBS vs Prioritised and high demand: '
          + str(A_travel_d_cbs_prio_hi))

    ####################################################################################################################
    # hypothesis 9: average travel distance for Individual is higher than for Prioritised planning with a high arrival rate
    ####################################################################################################################
    A_travel_d_ind_prio_hi = VD_A(list(res_ind_hi['travel_d_avg']), list(res_prio_hi['travel_d_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average travel distance: Individual vs Prioritised and high demand: '
          + str(A_travel_d_ind_prio_hi))

    ####################################################################################################################
    # hypothesis 10: average travel distance for CBS is higher than for Individual planning with a low arrival rate
    ####################################################################################################################
    A_travel_d_cbs_ind_lo = VD_A(list(res_cbs_lo['travel_d_avg']), list(res_ind_lo['travel_d_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average travel distance: CBS vs Individual and low demand: '
          + str(A_travel_d_cbs_ind_lo))

    ####################################################################################################################
    # hypothesis 11: average travel distance for CBS is higher than for Prioritised planning with a low arrival rate
    ####################################################################################################################
    A_travel_d_cbs_prio_lo = VD_A(list(res_cbs_lo['travel_d_avg']), list(res_prio_lo['travel_d_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average travel distance: CBS vs Prioritised and low demand: '
          + str(A_travel_d_cbs_prio_lo))

    ####################################################################################################################
    # hypothesis 12: average travel distance for Individual is higher than for Prioritised planning with a low arrival rate
    ####################################################################################################################
    A_travel_d_ind_prio_lo = VD_A(list(res_ind_lo['travel_d_avg']), list(res_prio_lo['travel_d_avg']))
    print('----------------')
    print(
        'Vargha and Delaney A index for comparing average travel distance: Individual vs Prioritised and low demand: '
        + str(A_travel_d_ind_prio_lo))

    ####################################################################################################################
    # hypothesis 13: avg time/distance for CBS higher than Individual for high arrival rate
    ####################################################################################################################
    A_travel_td_cbs_ind_hi = VD_A(list(res_cbs_hi['travel_td_avg']), list(res_ind_hi['travel_td_avg']))
    print('----------------')
    print(
        'Vargha and Delaney A index for comparing travel time/distance: CBS vs Individual and high demand: '
        + str(A_travel_td_cbs_ind_hi))

    ####################################################################################################################
    # hypothesis 14: avg time/distance for CBS higher than Prioritised for high arrival rate
    ####################################################################################################################
    A_travel_td_cbs_prio_hi = VD_A(list(res_cbs_hi['travel_td_avg']), list(res_prio_hi['travel_td_avg']))
    print('----------------')
    print(
        'Vargha and Delaney A index for comparing travel time/distance: CBS vs Prioritised and high demand: '
        + str(A_travel_td_cbs_prio_hi))

    ####################################################################################################################
    # hypothesis 15: avg time/distance for Individual higher than Prioritised for high arrival rate
    ####################################################################################################################
    A_travel_td_ind_prio_hi = VD_A(list(res_ind_hi['travel_td_avg']), list(res_prio_hi['travel_td_avg']))
    print('----------------')
    print(
        'Vargha and Delaney A index for comparing travel time/distance: Individual vs Prioritised and high demand: '
        + str(A_travel_td_ind_prio_hi))

    ####################################################################################################################
    # hypothesis 16: avg time/distance for CBS higher than Individual for low arrival rate
    ####################################################################################################################
    A_travel_td_cbs_ind_lo = VD_A(list(res_cbs_lo['travel_td_avg']), list(res_ind_lo['travel_td_avg']))
    print('----------------')
    print(
        'Vargha and Delaney A index for comparing travel time/distance: CBS vs Individual and low demand: '
        + str(A_travel_td_cbs_ind_lo))

    ####################################################################################################################
    # hypothesis 17: avg time/distance for CBS higher than Prioritised for low arrival rate
    ####################################################################################################################
    A_travel_td_cbs_prio_lo = VD_A(list(res_cbs_lo['travel_td_avg']), list(res_prio_lo['travel_td_avg']))
    print('----------------')
    print(
        'Vargha and Delaney A index for comparing travel time/distance: CBS vs Prioritised and low demand: '
        + str(A_travel_td_cbs_prio_lo))

    ####################################################################################################################
    # hypothesis 18: avg time/distance for Individual higher than Prioritised for high arrival rate
    ####################################################################################################################
    A_travel_td_ind_prio_lo = VD_A(list(res_ind_lo['travel_td_avg']), list(res_prio_lo['travel_td_avg']))
    print('----------------')
    print(
        'Vargha and Delaney A index for comparing travel time/distance: Individual vs Prioritised and low demand: '
        + str(A_travel_td_ind_prio_lo))

    ####################################################################################################################
    # hypothesis 19: avg computation time for CBS higher than Individual for high arrival rate
    ####################################################################################################################
    A_comp_cbs_ind_hi = VD_A(list(res_cbs_hi['comput_t_avg']), list(res_ind_hi['comput_t_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average computation time: CBS vs Individual and high demand: ' +
          str(A_comp_cbs_ind_hi))

    ####################################################################################################################
    # hypothesis 20: avg computation time for CBS higher than Prioritised for high arrival rate
    ####################################################################################################################
    A_comp_cbs_prio_hi = VD_A(list(res_cbs_hi['comput_t_avg']), list(res_prio_hi['comput_t_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average computation time: CBS vs Prioritised and high demand: ' +
          str(A_comp_cbs_prio_hi))

    ####################################################################################################################
    # hypothesis 21: avg computation time for Individual higher than Prioritised for high arrival rate
    ####################################################################################################################
    A_comp_ind_prio_hi = VD_A(list(res_ind_hi['comput_t_avg']), list(res_prio_hi['comput_t_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average computation time: Individual vs Prioritised and high demand: '
          + str(A_comp_ind_prio_hi))

    ####################################################################################################################
    # hypothesis 19: avg computation time for CBS higher than Individual for low arrival rate
    ####################################################################################################################
    A_comp_cbs_ind_lo = VD_A(list(res_cbs_lo['comput_t_avg']), list(res_ind_lo['comput_t_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average computation time: CBS vs Individual and low demand: ' +
          str(A_comp_cbs_ind_lo))

    ####################################################################################################################
    # hypothesis 20: avg computation time for CBS higher than Prioritised for low arrival rate
    ####################################################################################################################
    A_comp_cbs_prio_lo = VD_A(list(res_cbs_lo['comput_t_avg']), list(res_prio_lo['comput_t_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average computation time: CBS vs Prioritised and low demand: ' +
          str(A_comp_cbs_prio_lo))

    ####################################################################################################################
    # hypothesis 21: avg computation time for Individual higher than Prioritised for high arrival rate
    ####################################################################################################################
    A_comp_ind_prio_lo = VD_A(list(res_ind_lo['comput_t_avg']), list(res_prio_lo['comput_t_avg']))
    print('----------------')
    print(
        'Vargha and Delaney A index for comparing average computation time: Individual vs Prioritised and low demand: '
        + str(A_comp_ind_prio_lo))

    ####################################################################################################################
    # hypothesis 22: avg throughput for CBS higher than Individual for high arrival rate
    ####################################################################################################################
    A_thru_cbs_ind_hi = VD_A(list(res_cbs_hi['throughput_avg']), list(res_ind_hi['throughput_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing average throughput: CBS vs Individual and high demand: '
          + str(A_thru_cbs_ind_hi))

    ####################################################################################################################
    # hypothesis 23: avg throughput for CBS higher than Prioritised for high arrival rate
    ####################################################################################################################
    A_thru_cbs_prio_hi = VD_A(list(res_cbs_hi['throughput_avg']), list(res_prio_hi['throughput_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing average throughput: CBS vs Prioritised and high demand: '
          + str(A_thru_cbs_prio_hi))

    ####################################################################################################################
    # hypothesis 24: avg throughput for Individual higher than Prioritised for high arrival rate
    ####################################################################################################################
    A_thru_ind_prio_hi = VD_A(list(res_ind_hi['throughput_avg']), list(res_prio_hi['throughput_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing average throughput: Individual vs Prioritised and high demand: '
          + str(A_thru_ind_prio_hi))

    ####################################################################################################################
    # hypothesis 25: avg throughput for CBS higher than Individual for low arrival rate
    ####################################################################################################################
    A_thru_cbs_ind_lo = VD_A(list(res_cbs_lo['throughput_avg']), list(res_ind_lo['throughput_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing average throughput: CBS vs Individual and low demand: '
          + str(A_thru_cbs_ind_lo))

    ####################################################################################################################
    # hypothesis 26: avg throughput for CBS higher than Prioritised for low arrival rate
    ####################################################################################################################
    A_thru_cbs_prio_lo = VD_A(list(res_cbs_lo['throughput_avg']), list(res_prio_lo['throughput_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing average throughput: CBS vs Prioritised and low demand: '
          + str(A_thru_cbs_prio_lo))

    ####################################################################################################################
    # hypothesis 27: avg throughput for Individual higher than Prioritised for low arrival rate
    ####################################################################################################################
    A_thru_ind_prio_lo = VD_A(list(res_ind_lo['throughput_avg']), list(res_prio_lo['throughput_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing average throughput: Individual vs Prioritised and low demand: '
          + str(A_thru_ind_prio_lo))

    ####################################################################################################################
    # hypothesis 28: avg expanded nodes higher for CBS than Individual and high demand
    ####################################################################################################################
    A_exp_cbs_ind_hi = VD_A(list(res_cbs_hi['exp_nodes']), list(res_ind_hi['exp_nodes']))
    print('---------------')
    print('Vargha and Delaney A index for comparing expanded nodes: CBS vs Individual and high demand: '
          + str(A_exp_cbs_ind_hi))

    ####################################################################################################################
    # hypothesis 29: avg expanded nodes higher for CBS than Prioritised and high demand
    ####################################################################################################################
    A_exp_cbs_prio_hi = VD_A(list(res_cbs_hi['exp_nodes']), list(res_prio_hi['exp_nodes']))
    print('---------------')
    print('Vargha and Delaney A index for comparing expanded nodes: CBS vs Prioritised and high demand: '
          + str(A_exp_cbs_prio_hi))

    ####################################################################################################################
    # hypothesis 30: avg expanded nodes higher for Individual than Prioritised and high demand
    ####################################################################################################################
    A_exp_ind_prio_hi = VD_A(list(res_ind_hi['exp_nodes']), list(res_prio_hi['exp_nodes']))
    print('---------------')
    print('Vargha and Delaney A index for comparing expanded nodes: Individual vs Prioritised and high demand: '
          + str(A_exp_ind_prio_hi))

    ####################################################################################################################
    # hypothesis 31: avg expanded nodes higher for CBS than Individual and low demand
    ####################################################################################################################
    A_exp_cbs_ind_lo = VD_A(list(res_cbs_lo['exp_nodes']), list(res_ind_lo['exp_nodes']))
    print('---------------')
    print('Vargha and Delaney A index for comparing expanded nodes: CBS vs Individual and low demand: '
          + str(A_exp_cbs_ind_lo))

    ####################################################################################################################
    # hypothesis 32: avg expanded nodes higher for CBS than Prioritised and low demand
    ####################################################################################################################
    A_exp_cbs_prio_lo = VD_A(list(res_cbs_lo['exp_nodes']), list(res_prio_lo['exp_nodes']))
    print('---------------')
    print('Vargha and Delaney A index for comparing expanded nodes: CBS vs Prioritised and low demand: '
          + str(A_exp_cbs_prio_lo))

    ####################################################################################################################
    # hypothesis 33: avg expanded nodes higher for Individual than Prioritised and low demand
    ####################################################################################################################
    A_exp_ind_prio_lo = VD_A(list(res_ind_lo['exp_nodes']), list(res_prio_lo['exp_nodes']))
    print('---------------')
    print('Vargha and Delaney A index for comparing expanded nodes: Individual vs Prioritised and low demand: '
          + str(A_exp_ind_prio_lo))

    ####################################################################################################################
    # hypothesis 34: avg deadlocks higher for CBS than Individual for high demand
    ####################################################################################################################
    A_dead_cbs_ind_hi = VD_A(list(res_cbs_hi['deadlocks']), list(res_ind_hi['deadlocks']))
    print('---------------')
    print('Vargha and Delaney A index for comparing deadlocks: CBS vs Individual and high demand: '
          + str(A_dead_cbs_ind_hi))

    ####################################################################################################################
    # hypothesis 35: avg deadlocks higher for CBS than Prioritised for high demand
    ####################################################################################################################
    A_dead_cbs_prio_hi = VD_A(list(res_cbs_hi['deadlocks']), list(res_prio_hi['deadlocks']))
    print('---------------')
    print('Vargha and Delaney A index for comparing deadlocks: CBS vs Prioritised and high demand: '
          + str(A_dead_cbs_prio_hi))

    ####################################################################################################################
    # hypothesis 36: avg deadlocks higher for Individual than Prioritised for high demand
    ####################################################################################################################
    A_dead_ind_prio_hi = VD_A(list(res_ind_hi['deadlocks']), list(res_prio_hi['deadlocks']))
    print('---------------')
    print('Vargha and Delaney A index for comparing deadlocks: Individual vs Prioritised and high demand: '
          + str(A_dead_ind_prio_hi))

    ####################################################################################################################
    # hypothesis 37: avg deadlocks higher for CBS than Individual for low demand
    ####################################################################################################################
    A_dead_cbs_ind_lo = VD_A(list(res_cbs_lo['deadlocks']), list(res_ind_lo['deadlocks']))
    print('---------------')
    print('Vargha and Delaney A index for comparing deadlocks: CBS vs Individual and low demand: '
          + str(A_dead_cbs_ind_lo))

    ####################################################################################################################
    # hypothesis 38: avg deadlocks higher for CBS than Prioritised for low demand
    ####################################################################################################################
    A_dead_cbs_prio_lo = VD_A(list(res_cbs_lo['deadlocks']), list(res_prio_lo['deadlocks']))
    print('---------------')
    print('Vargha and Delaney A index for comparing deadlocks: CBS vs Prioritised and low demand: '
          + str(A_dead_cbs_prio_lo))

    ####################################################################################################################
    # hypothesis 39: avg deadlocks higher for Individual than Prioritised for high demand
    ####################################################################################################################
    A_dead_ind_prio_lo = VD_A(list(res_ind_lo['deadlocks']), list(res_prio_lo['deadlocks']))
    print('---------------')
    print('Vargha and Delaney A index for comparing deadlocks: Individual vs Prioritised and low demand: '
          + str(A_dead_ind_prio_lo))

    ####################################################################################################################
    # hypothesis 40: avg travel time CBS high vs low demand
    ####################################################################################################################
    A_travel_t_cbs = VD_A(list(res_cbs_hi['travel_t_avg']), list(res_cbs_lo['travel_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time: CBS high and low demand: '
          + str(A_travel_t_cbs))

    ####################################################################################################################
    # hypothesis 41: avg travel distance CBS high vs low demand
    ####################################################################################################################
    A_travel_d_cbs = VD_A(list(res_cbs_hi['travel_d_avg']), list(res_cbs_lo['travel_d_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel distance: CBS high and low demand: '
          + str(A_travel_d_cbs))

    ####################################################################################################################
    # hypothesis 42: avg travel distance CBS high vs low demand
    ####################################################################################################################
    A_travel_td_cbs = VD_A(list(res_cbs_hi['travel_td_avg']), list(res_cbs_lo['travel_td_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time/distance: CBS high and low demand: '
          + str(A_travel_td_cbs))

    ####################################################################################################################
    # hypothesis 43: avg computation time CBS high vs low demand
    ####################################################################################################################
    A_comp_cbs = VD_A(list(res_cbs_hi['comput_t_avg']), list(res_cbs_lo['comput_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing computation time: CBS high and low demand: '
          + str(A_comp_cbs))

    ####################################################################################################################
    # hypothesis 44: avg throughput CBS high vs low demand
    ####################################################################################################################
    A_tru_cbs = VD_A(list(res_cbs_hi['throughput_avg']), list(res_cbs_lo['throughput_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing throughput: CBS high and low demand: '
          + str(A_tru_cbs))

    ####################################################################################################################
    # hypothesis 45: avg expanded nodes CBS high vs low demand
    ####################################################################################################################
    A_exp_cbs = VD_A(list(res_cbs_hi['exp_nodes']), list(res_cbs_lo['exp_nodes']))
    print('---------------')
    print('Vargha and Delaney A index for comparing expanded nodes: CBS high and low demand: '
          + str(A_exp_cbs))

    ####################################################################################################################
    # hypothesis 46: avg deadlocks CBS high vs low demand
    ####################################################################################################################
    A_dead_cbs = VD_A(list(res_cbs_hi['deadlocks']), list(res_cbs_lo['deadlocks']))
    print('---------------')
    print('Vargha and Delaney A index for comparing deadlocks: CBS high and low demand: '
          + str(A_dead_cbs))

    ####################################################################################################################
    # hypothesis 47: avg travel time Prioritised high vs low demand
    ####################################################################################################################
    A_travel_t_prio = VD_A(list(res_prio_hi['travel_t_avg']), list(res_prio_lo['travel_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time: Prioritised high and low demand: '
          + str(A_travel_t_prio))

    ####################################################################################################################
    # hypothesis 48: avg travel distance Prioritised high vs low demand
    ####################################################################################################################
    A_travel_d_prio = VD_A(list(res_prio_hi['travel_d_avg']), list(res_prio_lo['travel_d_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel distance: Prioritised high and low demand: '
          + str(A_travel_d_prio))

    ####################################################################################################################
    # hypothesis 49: avg travel time/distance Prioritised high vs low demand
    ####################################################################################################################
    A_travel_td_prio = VD_A(list(res_prio_hi['travel_td_avg']), list(res_prio_lo['travel_td_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time/distance: Prioritised high and low demand: '
          + str(A_travel_td_prio))

    ####################################################################################################################
    # hypothesis 50: avg computation time Prioritised high vs low demand
    ####################################################################################################################
    A_comp_prio = VD_A(list(res_prio_hi['comput_t_avg']), list(res_prio_lo['comput_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing computation time: Prioritised high and low demand: '
          + str(A_comp_prio))

    ####################################################################################################################
    # hypothesis 51: avg throughput Prioritised high vs low demand
    ####################################################################################################################
    A_tru_prio = VD_A(list(res_prio_hi['throughput_avg']), list(res_prio_lo['throughput_avg']))
    print('---------------')
    print('Vargha and Delaney A index for throughput: Prioritised high and low demand: '
          + str(A_tru_prio))

    ####################################################################################################################
    # hypothesis 52: avg exp nodes Prioritised high vs low demand
    ####################################################################################################################
    A_exp_prio = VD_A(list(res_prio_hi['exp_nodes']), list(res_prio_lo['exp_nodes']))
    print('---------------')
    print('Vargha and Delaney A index for comparing expanded nodes: Prioritised high and low demand: '
          + str(A_exp_prio))

    ####################################################################################################################
    # hypothesis 53: avg deadlocks Prioritised high vs low demand
    ####################################################################################################################
    A_dead_prio = VD_A(list(res_prio_hi['deadlocks']), list(res_prio_lo['deadlocks']))
    print('---------------')
    print('Vargha and Delaney A index for comparing deadlocks: Prioritised high and low demand: '
          + str(A_dead_prio))

    ####################################################################################################################
    # hypothesis 54: avg travel time Individual high vs low demand
    ####################################################################################################################
    A_travel_t_ind = VD_A(list(res_ind_hi['travel_t_avg']), list(res_ind_lo['travel_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time: Individual high and low demand: '
          + str(A_travel_t_ind))

    ####################################################################################################################
    # hypothesis 55: avg travel distance Individual high vs low demand
    ####################################################################################################################
    A_travel_d_ind = VD_A(list(res_ind_hi['travel_d_avg']), list(res_ind_lo['travel_d_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel distance: Individual high and low demand: '
          + str(A_travel_d_ind))

    ####################################################################################################################
    # hypothesis 56: avg travel time/distance Individual high vs low demand
    ####################################################################################################################
    A_travel_td_ind = VD_A(list(res_ind_hi['travel_td_avg']), list(res_ind_lo['travel_td_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time/distance: Individual high and low demand: '
          + str(A_travel_td_ind))

    ####################################################################################################################
    # hypothesis 57: avg computation time Individual high vs low demand
    ####################################################################################################################
    A_comp_ind = VD_A(list(res_ind_hi['comput_t_avg']), list(res_ind_lo['comput_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing computation time: Individual high and low demand: '
          + str(A_comp_ind))

    ####################################################################################################################
    # hypothesis 58: avg throughput Individual high vs low demand
    ####################################################################################################################
    A_tru_ind = VD_A(list(res_ind_hi['throughput_avg']), list(res_ind_lo['throughput_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing throughput: Individual high and low demand: '
          + str(A_tru_ind))

    ####################################################################################################################
    # hypothesis 59: avg exp nodes Individual high vs low demand
    ####################################################################################################################
    A_exp_ind = VD_A(list(res_ind_hi['exp_nodes']), list(res_ind_lo['exp_nodes']))
    print('---------------')
    print('Vargha and Delaney A index for comparing expanded nodes: Individual high and low demand: '
          + str(A_exp_ind))

    ####################################################################################################################
    # hypothesis 60: avg deadlocks Individual high vs low demand
    ####################################################################################################################
    A_dead_ind = VD_A(list(res_ind_hi['deadlocks']), list(res_ind_lo['deadlocks']))
    print('---------------')
    print('Vargha and Delaney A index for comparing deadlocks: Individual high and low demand: '
          + str(A_dead_ind))

    print('-----------------SENSITIVITY ANALYSIS----------------------------')
    print('-----------INDIVIDUAL PLANNING OBSERVATION SIZE--------------------')

    ####################################################################################################################
    # hypothesis 61: avg travel time higher for observation size 2 vs 3 with high demand
    ####################################################################################################################
    A_travel_t_23_hi = VD_A(list(res_obs_2_hi['travel_t_avg']), list(res_obs_3_hi['travel_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time: obs 2 vs 3 and high demand: '
          + str(A_travel_t_23_hi))

    ####################################################################################################################
    # hypothesis 62: avg travel time higher for observation size 2 vs 4 with high demand
    ####################################################################################################################
    A_travel_t_24_hi = VD_A(list(res_obs_2_hi['travel_t_avg']), list(res_obs_4_hi['travel_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time: obs 2 vs 4 and high demand: '
          + str(A_travel_t_24_hi))

    ####################################################################################################################
    # hypothesis 63: avg travel time higher for observation size 3 vs 4 with high demand
    ####################################################################################################################
    A_travel_t_34_hi = VD_A(list(res_obs_3_hi['travel_t_avg']), list(res_obs_4_hi['travel_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time: obs 3 vs 4 and high demand: '
          + str(A_travel_t_34_hi))

    ####################################################################################################################
    # hypothesis 64: avg travel distance higher for observation size 2 vs 3 with high demand
    ####################################################################################################################
    A_travel_d_23_hi = VD_A(list(res_obs_2_hi['travel_d_avg']), list(res_obs_3_hi['travel_d_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel distance: obs 2 vs 3 and high demand: '
          + str(A_travel_d_23_hi))

    ####################################################################################################################
    # hypothesis 65: avg travel distance higher for observation size 2 vs 4 with high demand
    ####################################################################################################################
    A_travel_d_24_hi = VD_A(list(res_obs_2_hi['travel_d_avg']), list(res_obs_4_hi['travel_d_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel distance: obs 2 vs 4 and high demand: '
          + str(A_travel_d_24_hi))

    ####################################################################################################################
    # hypothesis 66: avg travel distance higher for observation size 3 vs 4 with high demand
    ####################################################################################################################
    A_travel_d_34_hi = VD_A(list(res_obs_3_hi['travel_d_avg']), list(res_obs_4_hi['travel_d_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel distance: obs 3 vs 4 and high demand: '
          + str(A_travel_d_34_hi))

    ####################################################################################################################
    # hypothesis 67: avg travel time/distance higher for observation size 2 vs 3 with high demand
    ####################################################################################################################
    A_travel_td_23_hi = VD_A(list(res_obs_2_hi['travel_td_avg']), list(res_obs_3_hi['travel_td_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time/distance: obs 2 vs 3 and high demand: '
          + str(A_travel_td_23_hi))

    ####################################################################################################################
    # hypothesis 68: avg travel time/distance higher for observation size 2 vs 4 with high demand
    ####################################################################################################################
    A_travel_td_24_hi = VD_A(list(res_obs_2_hi['travel_td_avg']), list(res_obs_4_hi['travel_td_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time/distance: obs 2 vs 4 and high demand: '
          + str(A_travel_td_24_hi))

    ####################################################################################################################
    # hypothesis 69: avg travel time/distance higher for observation size 3 vs 4 with high demand
    ####################################################################################################################
    A_travel_td_34_hi = VD_A(list(res_obs_3_hi['travel_td_avg']), list(res_obs_4_hi['travel_td_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time/distance: obs 3 vs 4 and high demand: '
          + str(A_travel_td_34_hi))

    ####################################################################################################################
    # hypothesis 70: avg computation time higher for observation size 2 vs 3 with high demand
    ####################################################################################################################
    A_comp_23_hi = VD_A(list(res_obs_2_hi['comput_t_avg']), list(res_obs_3_hi['comput_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing computation time: obs 2 vs 3 and high demand: '
          + str(A_comp_23_hi))

    ####################################################################################################################
    # hypothesis 71: avg computation time higher for observation size 2 vs 4 with high demand
    ####################################################################################################################
    A_comp_24_hi = VD_A(list(res_obs_2_hi['comput_t_avg']), list(res_obs_4_hi['comput_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing computation time: obs 2 vs 4 and high demand: '
          + str(A_comp_24_hi))

    ####################################################################################################################
    # hypothesis 72: avg computation time higher for observation size 3 vs 4 with high demand
    ####################################################################################################################
    A_comp_34_hi = VD_A(list(res_obs_3_hi['comput_t_avg']), list(res_obs_4_hi['comput_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing computation time: obs 3 vs 4 and high demand: '
          + str(A_comp_34_hi))

    ####################################################################################################################
    # hypothesis 73: avg throughput higher for observation size 2 vs 3 with high demand
    ####################################################################################################################
    A_tru_23_hi = VD_A(list(res_obs_2_hi['throughput_avg']), list(res_obs_3_hi['throughput_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing throughput: obs 2 vs 3 and high demand: '
          + str(A_tru_23_hi))

    ####################################################################################################################
    # hypothesis 74: avg throughput higher for observation size 2 vs 4 with high demand
    ####################################################################################################################
    A_tru_24_hi = VD_A(list(res_obs_2_hi['throughput_avg']), list(res_obs_4_hi['throughput_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing throughput: obs 2 vs 4 and high demand: '
          + str(A_tru_24_hi))

    ####################################################################################################################
    # hypothesis 75: avg throughput higher for observation size 3 vs 4 with high demand
    ####################################################################################################################
    A_tru_34_hi = VD_A(list(res_obs_3_hi['throughput_avg']), list(res_obs_4_hi['throughput_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing throughput: obs 3 vs 4 and high demand: '
          + str(A_tru_34_hi))

    ####################################################################################################################
    # hypothesis 76: avg exp nodes higher for observation size 2 vs 3 with high demand
    ####################################################################################################################
    A_exp_23_hi = VD_A(list(res_obs_2_hi['exp_nodes']), list(res_obs_3_hi['exp_nodes']))
    print('---------------')
    print('Vargha and Delaney A index for comparing expanded nodes: obs 2 vs 3 and high demand: '
          + str(A_exp_23_hi))

    ####################################################################################################################
    # hypothesis 77: avg exp nodes higher for observation size 2 vs 4 with high demand
    ####################################################################################################################
    A_exp_24_hi = VD_A(list(res_obs_2_hi['exp_nodes']), list(res_obs_4_hi['exp_nodes']))
    print('---------------')
    print('Vargha and Delaney A index for comparing expanded nodes: obs 2 vs 4 and high demand: '
          + str(A_exp_24_hi))

    ####################################################################################################################
    # hypothesis 78: avg exp nodes higher for observation size 3 vs 4 with high demand
    ####################################################################################################################
    A_exp_34_hi = VD_A(list(res_obs_3_hi['exp_nodes']), list(res_obs_4_hi['exp_nodes']))
    print('---------------')
    print('Vargha and Delaney A index for comparing expanded nodes: obs 3 vs 4 and high demand: '
          + str(A_exp_34_hi))

    ####################################################################################################################
    # hypothesis 79: avg deadlocks higher for observation size 2 vs 3 with high demand
    ####################################################################################################################
    A_dead_23_hi = VD_A(list(res_obs_2_hi['deadlocks']), list(res_obs_3_hi['deadlocks']))
    print('---------------')
    print('Vargha and Delaney A index for comparing deadlocks: obs 2 vs 3 and high demand: '
          + str(A_dead_23_hi))

    ####################################################################################################################
    # hypothesis 80: avg deadlocks higher for observation size 2 vs 4 with high demand
    ####################################################################################################################
    A_dead_24_hi = VD_A(list(res_obs_2_hi['deadlocks']), list(res_obs_4_hi['deadlocks']))
    print('---------------')
    print('Vargha and Delaney A index for comparing deadlocks: obs 2 vs 4 and high demand: '
          + str(A_dead_24_hi))

    ####################################################################################################################
    # hypothesis 81: avg deadlocks higher for observation size 3 vs 4 with high demand
    ####################################################################################################################
    A_dead_34_hi = VD_A(list(res_obs_3_hi['deadlocks']), list(res_obs_4_hi['deadlocks']))
    print('---------------')
    print('Vargha and Delaney A index for comparing deadlocks: obs 3 vs 4 and high demand: '
          + str(A_dead_34_hi))

    print('---------------BIDDING DECAY FACTOR----------------------------')
    ####################################################################################################################
    # hypothesis 82: avg travel time for bidding factor 1.1 vs 1.5 with high demand
    ####################################################################################################################
    A_bid_11_15 = VD_A(list(res_bid_11_hi['travel_t_avg']), list(res_bid_15_hi['travel_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time: bid 1.1 vs 1.5 and high demand: '
          + str(A_bid_11_15))

    ####################################################################################################################
    # hypothesis 83: avg travel time for bidding factor 1.1 vs 2.0 with high demand
    ####################################################################################################################
    A_bid_11_20 = VD_A(list(res_bid_11_hi['travel_t_avg']), list(res_bid_20_hi['travel_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time: bid 1.1 vs 12.0 and high demand: '
          + str(A_bid_11_20))

    ####################################################################################################################
    # hypothesis 84: avg travel time for bidding factor 1.1 vs full with high demand
    ####################################################################################################################
    A_bid_11_full = VD_A(list(res_bid_11_hi['travel_t_avg']), list(res_bid_full_hi['travel_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time: bid 1.1 vs full and high demand: '
          + str(A_bid_11_full))

    ####################################################################################################################
    # hypothesis 85: avg travel time for bidding factor 1.5 vs 2.0 with high demand
    ####################################################################################################################
    A_bid_15_20 = VD_A(list(res_bid_15_hi['travel_t_avg']), list(res_bid_20_hi['travel_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time: bid 1.5 vs 2.0 and high demand: '
          + str(A_bid_15_20))

    ####################################################################################################################
    # hypothesis 86: avg travel time for bidding factor 1.5 vs full with high demand
    ####################################################################################################################
    A_bid_15_full = VD_A(list(res_bid_15_hi['travel_t_avg']), list(res_bid_full_hi['travel_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time: bid 1.5 vs full and high demand: '
          + str(A_bid_15_full))

    ####################################################################################################################
    # hypothesis 87: avg travel time for bidding factor 2.0 vs full with high demand
    ####################################################################################################################
    A_bid_20_full = VD_A(list(res_bid_20_hi['travel_t_avg']), list(res_bid_full_hi['travel_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time: bid 2.0 vs full and high demand: '
          + str(A_bid_20_full))

    ####################################################################################################################
    # hypothesis 88: avg travel distance for bidding factor 1.1 vs 1.5 with high demand
    ####################################################################################################################
    A_bid_d_11_15 = VD_A(list(res_bid_11_hi['travel_d_avg']), list(res_bid_15_hi['travel_d_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel distance: bid 1.1 vs 1.5 and high demand: '
          + str(A_bid_d_11_15))

    ####################################################################################################################
    # hypothesis 89: avg travel distance for bidding factor 1.1 vs 2.0 with high demand
    ####################################################################################################################
    A_bid_d_11_20 = VD_A(list(res_bid_11_hi['travel_d_avg']), list(res_bid_20_hi['travel_d_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel distance: bid 1.1 vs 2.0 and high demand: '
          + str(A_bid_d_11_20))

    ####################################################################################################################
    # hypothesis 90: avg travel distance for bidding factor 1.1 vs full with high demand
    ####################################################################################################################
    A_bid_d_11_full = VD_A(list(res_bid_11_hi['travel_d_avg']), list(res_bid_full_hi['travel_d_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel distance: bid 1.1 vs full and high demand: '
          + str(A_bid_d_11_full))

    ####################################################################################################################
    # hypothesis 91: avg travel distance for bidding factor 1.5 vs 2.0 with high demand
    ####################################################################################################################
    A_bid_d_15_20 = VD_A(list(res_bid_15_hi['travel_d_avg']), list(res_bid_20_hi['travel_d_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel distance: bid 1.5 vs 2.0 and high demand: '
          + str(A_bid_d_15_20))

    ####################################################################################################################
    # hypothesis 92: avg travel distance for bidding factor 1.5 vs full with high demand
    ####################################################################################################################
    A_bid_d_15_full = VD_A(list(res_bid_15_hi['travel_d_avg']), list(res_bid_full_hi['travel_d_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel distance: bid 1.5 vs full and high demand: '
          + str(A_bid_d_15_full))

    ####################################################################################################################
    # hypothesis 93: avg travel distance for bidding factor 2.0 vs full with high demand
    ####################################################################################################################
    A_bid_d_20_full = VD_A(list(res_bid_20_hi['travel_d_avg']), list(res_bid_full_hi['travel_d_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel distance: bid 2.0 vs full and high demand: '
          + str(A_bid_d_20_full))

    ####################################################################################################################
    # hypothesis 94: avg travel time/distance for bidding factor 1.1 vs 1.5 with high demand
    ####################################################################################################################
    A_bid_td_11_15 = VD_A(list(res_bid_11_hi['travel_td_avg']), list(res_bid_15_hi['travel_td_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time/distance: bid 1.1 vs 1.5 and high demand: '
          + str(A_bid_td_11_15))

    ####################################################################################################################
    # hypothesis 95: avg travel time/distance for bidding factor 1.1 vs 2.0 with high demand
    ####################################################################################################################
    A_bid_td_11_20 = VD_A(list(res_bid_11_hi['travel_td_avg']), list(res_bid_20_hi['travel_td_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time/distance: bid 1.1 vs 2.0 and high demand: '
          + str(A_bid_td_11_20))

    ####################################################################################################################
    # hypothesis 96: avg travel time/distance for bidding factor 1.1 vs full with high demand
    ####################################################################################################################
    A_bid_td_11_full = VD_A(list(res_bid_11_hi['travel_td_avg']), list(res_bid_full_hi['travel_td_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time/distance: bid 1.1 vs full and high demand: '
          + str(A_bid_td_11_full))

    ####################################################################################################################
    # hypothesis 97: avg travel time/distance for bidding factor 1.5 vs 2.0 with high demand
    ####################################################################################################################
    A_bid_td_15_20 = VD_A(list(res_bid_15_hi['travel_td_avg']), list(res_bid_20_hi['travel_td_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time/distance: bid 1.5 vs 2.0 and high demand: '
          + str(A_bid_td_15_20))

    ####################################################################################################################
    # hypothesis 98: avg travel time/distance for bidding factor 1.5 vs full with high demand
    ####################################################################################################################
    A_bid_td_15_full = VD_A(list(res_bid_15_hi['travel_td_avg']), list(res_bid_full_hi['travel_td_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time/distance: bid 1.5 vs full and high demand: '
          + str(A_bid_td_15_full))

    ####################################################################################################################
    # hypothesis 99: avg travel time/distance for bidding factor 2.0 vs full with high demand
    ####################################################################################################################
    A_bid_td_20_full = VD_A(list(res_bid_20_hi['travel_td_avg']), list(res_bid_full_hi['travel_td_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing travel time/distance: bid 2.0 vs full and high demand: '
          + str(A_bid_td_20_full))

    ####################################################################################################################
    # hypothesis 100: avg computation time for bidding factor 1.1 vs 1.5 with high demand
    ####################################################################################################################
    A_bid_comp_11_15 = VD_A(list(res_bid_11_hi['comput_t_avg']), list(res_bid_15_hi['comput_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing computation time: bid 1.1 vs 1.5 and high demand: '
          + str(A_bid_comp_11_15))

    ####################################################################################################################
    # hypothesis 101: avg computation time for bidding factor 1.1 vs 2.0 with high demand
    ####################################################################################################################
    A_bid_comp_11_20 = VD_A(list(res_bid_11_hi['comput_t_avg']), list(res_bid_20_hi['comput_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing computation time: bid 1.1 vs 2.0 and high demand: '
          + str(A_bid_comp_11_20))

    ####################################################################################################################
    # hypothesis 102: avg computation time for bidding factor 1.1 vs full with high demand
    ####################################################################################################################
    A_bid_comp_11_full = VD_A(list(res_bid_11_hi['comput_t_avg']), list(res_bid_full_hi['comput_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing computation time: bid 1.1 vs full and high demand: '
          + str(A_bid_comp_11_full))

    ####################################################################################################################
    # hypothesis 103: avg computation time for bidding factor 1.5 vs 2.0 with high demand
    ####################################################################################################################
    A_bid_comp_15_20 = VD_A(list(res_bid_15_hi['comput_t_avg']), list(res_bid_20_hi['comput_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing computation time: bid 1.5 vs 2.0 and high demand: '
          + str(A_bid_comp_15_20))

    ####################################################################################################################
    # hypothesis 104: avg computation time for bidding factor 1.5 vs full with high demand
    ####################################################################################################################
    A_bid_comp_15_full = VD_A(list(res_bid_15_hi['comput_t_avg']), list(res_bid_full_hi['comput_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing computation time: bid 1.5 vs full and high demand: '
          + str(A_bid_comp_15_full))

    ####################################################################################################################
    # hypothesis 105: avg computation time for bidding factor 2.0 vs full with high demand
    ####################################################################################################################
    A_bid_comp_20_full = VD_A(list(res_bid_20_hi['comput_t_avg']), list(res_bid_full_hi['comput_t_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing computation time: bid 2.0 vs full and high demand: '
          + str(A_bid_comp_20_full))

    ####################################################################################################################
    # hypothesis 106 avg throughput for bidding factor 1.1 vs 1.5 with high demand
    ####################################################################################################################
    A_bid_tru_11_15 = VD_A(list(res_bid_11_hi['throughput_avg']), list(res_bid_15_hi['throughput_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing throughput: bid 1.1 vs 1.5 and high demand: '
          + str(A_bid_tru_11_15))

    ####################################################################################################################
    # hypothesis 107 avg throughput for bidding factor 1.1 vs 2.0 with high demand
    ####################################################################################################################
    A_bid_tru_11_20 = VD_A(list(res_bid_11_hi['throughput_avg']), list(res_bid_20_hi['throughput_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing throughput: bid 1.1 vs 2.0 and high demand: '
          + str(A_bid_tru_11_20))

    ####################################################################################################################
    # hypothesis 108 avg throughput for bidding factor 1.1 vs full with high demand
    ####################################################################################################################
    A_bid_tru_11_full = VD_A(list(res_bid_11_hi['throughput_avg']), list(res_bid_full_hi['throughput_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing throughput: bid 1.1 vs full and high demand: '
          + str(A_bid_tru_11_full))

    ####################################################################################################################
    # hypothesis 109 avg throughput for bidding factor 1.5 vs 2.0 with high demand
    ####################################################################################################################
    A_bid_tru_15_20 = VD_A(list(res_bid_15_hi['throughput_avg']), list(res_bid_20_hi['throughput_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing throughput: bid 1.5 vs 2.0 and high demand: '
          + str(A_bid_tru_15_20))

    ####################################################################################################################
    # hypothesis 110 avg throughput for bidding factor 1.5 vs full with high demand
    ####################################################################################################################
    A_bid_tru_15_full = VD_A(list(res_bid_15_hi['throughput_avg']), list(res_bid_full_hi['throughput_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing throughput: bid 1.5 vs full and high demand: '
          + str(A_bid_tru_15_full))

    ####################################################################################################################
    # hypothesis 111 avg throughput for bidding factor 2.0 vs full with high demand
    ####################################################################################################################
    A_bid_tru_20_full = VD_A(list(res_bid_20_hi['throughput_avg']), list(res_bid_full_hi['throughput_avg']))
    print('---------------')
    print('Vargha and Delaney A index for comparing throughput: bid 2.0 vs full and high demand: '
          + str(A_bid_tru_20_full))

    ####################################################################################################################
    # hypothesis 112 avg exp nodes for bidding factor 1.1 vs 1.5 with high demand
    ####################################################################################################################
    A_bid_exp_11_15 = VD_A(list(res_bid_11_hi['exp_nodes']), list(res_bid_15_hi['exp_nodes']))
    print('---------------')
    print('Vargha and Delaney A index for comparing expanded nodes: bid 1.1 vs 1.5 and high demand: '
          + str(A_bid_exp_11_15))

    ####################################################################################################################
    # hypothesis 113 avg exp nodes for bidding factor 1.1 vs 2.0 with high demand
    ####################################################################################################################
    A_bid_exp_11_20 = VD_A(list(res_bid_11_hi['exp_nodes']), list(res_bid_20_hi['exp_nodes']))
    print('---------------')
    print('Vargha and Delaney A index for comparing expanded nodes: bid 1.1 vs 2.0 and high demand: '
          + str(A_bid_exp_11_20))

    ####################################################################################################################
    # hypothesis 114 avg exp nodes for bidding factor 1.1 vs full with high demand
    ####################################################################################################################
    A_bid_exp_11_full = VD_A(list(res_bid_11_hi['exp_nodes']), list(res_bid_full_hi['exp_nodes']))
    print('---------------')
    print('Vargha and Delaney A index for comparing expanded nodes: bid 1.1 vs full and high demand: '
          + str(A_bid_exp_11_full))

    ####################################################################################################################
    # hypothesis 115 avg exp nodes for bidding factor 1.5 vs 2.0 with high demand
    ####################################################################################################################
    A_bid_exp_15_20 = VD_A(list(res_bid_15_hi['exp_nodes']), list(res_bid_20_hi['exp_nodes']))
    print('---------------')
    print('Vargha and Delaney A index for comparing expanded nodes: bid 1.5 vs 2.0 and high demand: '
          + str(A_bid_exp_15_20))

    ####################################################################################################################
    # hypothesis 116 avg exp nodes for bidding factor 1.5 vs full with high demand
    ####################################################################################################################
    A_bid_exp_15_full = VD_A(list(res_bid_15_hi['exp_nodes']), list(res_bid_full_hi['exp_nodes']))
    print('---------------')
    print('Vargha and Delaney A index for comparing expanded nodes: bid 1.5 vs full and high demand: '
          + str(A_bid_exp_15_full))

    ####################################################################################################################
    # hypothesis 117 avg exp nodes for bidding factor 2.0 vs full with high demand
    ####################################################################################################################
    A_bid_exp_20_full = VD_A(list(res_bid_20_hi['exp_nodes']), list(res_bid_full_hi['exp_nodes']))
    print('---------------')
    print('Vargha and Delaney A index for comparing expanded nodes: bid 2.0 vs full and high demand: '
          + str(A_bid_exp_20_full))

    ####################################################################################################################
    # hypothesis 118 avg deadlocks for bidding factor 1.1 vs 1.5 with high demand
    ####################################################################################################################
    A_bid_dead_11_15 = VD_A(list(res_bid_11_hi['deadlocks']), list(res_bid_15_hi['deadlocks']))
    print('---------------')
    print('Vargha and Delaney A index for comparing deadlocks: bid 1.1 vs 1.5 and high demand: '
          + str(A_bid_dead_11_15))

    ####################################################################################################################
    # hypothesis 119 avg deadlocks for bidding factor 1.1 vs 2.0 with high demand
    ####################################################################################################################
    A_bid_dead_11_20 = VD_A(list(res_bid_11_hi['deadlocks']), list(res_bid_20_hi['deadlocks']))
    print('---------------')
    print('Vargha and Delaney A index for comparing deadlocks: bid 1.1 vs 2.0 and high demand: '
          + str(A_bid_dead_11_20))

    ####################################################################################################################
    # hypothesis 120 avg deadlocks for bidding factor 1.1 vs full with high demand
    ####################################################################################################################
    A_bid_dead_11_full = VD_A(list(res_bid_11_hi['deadlocks']), list(res_bid_full_hi['deadlocks']))
    print('---------------')
    print('Vargha and Delaney A index for comparing deadlocks: bid 1.1 vs full and high demand: '
          + str(A_bid_dead_11_full))

    ####################################################################################################################
    # hypothesis 121 avg deadlocks for bidding factor 1.5 vs 2.0 with high demand
    ####################################################################################################################
    A_bid_dead_15_20 = VD_A(list(res_bid_15_hi['deadlocks']), list(res_bid_20_hi['deadlocks']))
    print('---------------')
    print('Vargha and Delaney A index for comparing deadlocks: bid 1.5 vs 2.0 and high demand: '
          + str(A_bid_dead_15_20))

    ####################################################################################################################
    # hypothesis 122 avg deadlocks for bidding factor 1.5 vs full with high demand
    ####################################################################################################################
    A_bid_dead_15_full = VD_A(list(res_bid_15_hi['deadlocks']), list(res_bid_full_hi['deadlocks']))
    print('---------------')
    print('Vargha and Delaney A index for comparing deadlocks: bid 1.5 vs full and high demand: '
          + str(A_bid_dead_15_full))

    ####################################################################################################################
    # hypothesis 123 avg deadlocks for bidding factor 2.0 vs full with high demand
    ####################################################################################################################
    A_bid_dead_20_full = VD_A(list(res_bid_20_hi['deadlocks']), list(res_bid_full_hi['deadlocks']))
    print('---------------')
    print('Vargha and Delaney A index for comparing deadlocks: bid 2.0 vs full and high demand: '
          + str(A_bid_dead_20_full))