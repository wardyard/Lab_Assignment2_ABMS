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
    # Examples
    '''
    # negligible
    F = [0.8236111111111111, 0.7966666666666666, 0.923611111111111, 0.8197222222222222, 0.7108333333333333]
    G = [0.8052777777777779, 0.8172222222222221, 0.8322222222222223, 0.783611111111111, 0.8141666666666666]
    print(VD_A(G, F))

    # small
    A = [0.478515625, 0.4638671875, 0.4638671875, 0.4697265625, 0.4638671875, 0.474609375, 0.4814453125, 0.4814453125,
         0.4697265625, 0.4814453125, 0.474609375, 0.4833984375, 0.484375, 0.44921875, 0.474609375, 0.484375,
         0.4814453125, 0.4638671875, 0.484375, 0.478515625, 0.478515625, 0.45703125, 0.484375, 0.419921875,
         0.4833984375, 0.478515625, 0.4697265625, 0.484375, 0.478515625, 0.4638671875]
    B = [0.4814453125, 0.478515625, 0.44921875, 0.4814453125, 0.4638671875, 0.478515625, 0.474609375, 0.4638671875,
         0.474609375, 0.44921875, 0.474609375, 0.478515625, 0.478515625, 0.474609375, 0.4697265625, 0.474609375,
         0.45703125, 0.4697265625, 0.478515625, 0.4697265625, 0.4697265625, 0.484375, 0.45703125, 0.474609375,
         0.474609375, 0.4638671875, 0.45703125, 0.474609375, 0.4638671875, 0.4306640625]

    print(VD_A(A, B))

    # medium
    C = [0.9108333333333334, 0.8755555555555556, 0.900277777777778, 0.9274999999999999, 0.8777777777777779]
    E = [0.8663888888888888, 0.8802777777777777, 0.7816666666666667, 0.8377777777777776, 0.9305555555555556]
    print(VD_A(C, E))

    # Large
    D = [0.7202777777777778, 0.77, 0.8544444444444445, 0.7947222222222222, 0.7577777777777778]
    print(VD_A(C, D))
    '''
    # load data
    res_cbs_hi = pd.read_csv('results_cbs_hi.csv')
    res_cbs_lo = pd.read_csv('results_cbs_lo.csv')
    res_ind_hi = pd.read_csv('results_ind_hi.csv')
    res_ind_lo = pd.read_csv('results_ind_lo.csv')
    res_prio_hi = pd.read_csv('results_prio_hi.csv')
    res_prio_lo = pd.read_csv('results_prio_lo.csv')

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
    # hypothesis 5: average travel time for CBS is higher than for Prioritised planning with a high arrival rate
    ####################################################################################################################
    A_travel_t_cbs_prio_lo = VD_A(list(res_cbs_lo['travel_t_avg']), list(res_prio_lo['travel_t_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average travel time: CBS vs Prioritised and high demand: '
          + str(A_travel_t_cbs_prio_lo))

    ####################################################################################################################
    # hypothesis 6: average travel time for Individual is higher than for Prioritised planning with a low arrival rate
    ####################################################################################################################
    A_travel_t_ind_prio_lo = VD_A(list(res_ind_lo['travel_t_avg']), list(res_prio_lo['travel_t_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average travel time: Individual vs Prioritised and high demand: '
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
    print('Vargha and Delaney A index for comparing average travel distance: CBS vs Individual and high demand: ' +
          str(A_comp_cbs_ind_hi))

    ####################################################################################################################
    # hypothesis 20: avg computation time for CBS higher than Prioritised for high arrival rate
    ####################################################################################################################
    A_comp_cbs_prio_hi = VD_A(list(res_cbs_hi['comput_t_avg']), list(res_prio_hi['comput_t_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average travel distance: CBS vs Prioritised and high demand: ' +
          str(A_comp_cbs_prio_hi))

    ####################################################################################################################
    # hypothesis 21: avg computation time for Individual higher than Prioritised for high arrival rate
    ####################################################################################################################
    A_comp_ind_prio_hi = VD_A(list(res_ind_hi['comput_t_avg']), list(res_prio_hi['comput_t_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average travel distance: Individual vs Prioritised and high demand: '
          + str(A_comp_ind_prio_hi))

    ####################################################################################################################
    # hypothesis 19: avg computation time for CBS higher than Individual for low arrival rate
    ####################################################################################################################
    A_comp_cbs_ind_lo = VD_A(list(res_cbs_lo['comput_t_avg']), list(res_ind_lo['comput_t_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average travel distance: CBS vs Individual and low demand: ' +
          str(A_comp_cbs_ind_lo))

    ####################################################################################################################
    # hypothesis 20: avg computation time for CBS higher than Prioritised for low arrival rate
    ####################################################################################################################
    A_comp_cbs_prio_lo = VD_A(list(res_cbs_lo['comput_t_avg']), list(res_prio_lo['comput_t_avg']))
    print('----------------')
    print('Vargha and Delaney A index for comparing average travel distance: CBS vs Prioritised and low demand: ' +
          str(A_comp_cbs_prio_lo))

    ####################################################################################################################
    # hypothesis 21: avg computation time for Individual higher than Prioritised for high arrival rate
    ####################################################################################################################
    A_comp_ind_prio_lo = VD_A(list(res_ind_lo['comput_t_avg']), list(res_prio_lo['comput_t_avg']))
    print('----------------')
    print(
        'Vargha and Delaney A index for comparing average travel distance: Individual vs Prioritised and low demand: '
        + str(A_comp_ind_prio_lo))