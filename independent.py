"""
This is an example planner, that calls all agents to plan their route independently.
"""
import time

def run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t):
    expanded_nodes = 0    # expanded nodes KPI
    start = time.time()     # counter for computation speed
    for ac in aircraft_lst:
        if ac.spawntime == t:
            ac.status = "taxiing" 
            ac.position = nodes_dict[ac.start]["xy_pos"]
            exp_nodes = ac.plan_independent(nodes_dict, edges_dict, heuristics, t)
            expanded_nodes += exp_nodes    # expanded nodes KPI
    stop = time.time()      # counter for computation speed
    time_delta = stop - start

    return time_delta, expanded_nodes