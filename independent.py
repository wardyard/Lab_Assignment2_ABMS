"""
This is an example planner, that calls all agents to plan their route independently.
"""
import time

def run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t):
    start = time.clock()     # counter for computation speed
    for ac in aircraft_lst:
        if ac.spawntime == t:
            ac.status = "taxiing" 
            ac.position = nodes_dict[ac.start]["xy_pos"]
            ac.plan_independent(nodes_dict, edges_dict, heuristics, t)
    stop = time.clock()      # counter for computation speed
    time_delta = stop - start
    return time_delta