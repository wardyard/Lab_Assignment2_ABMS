"""
Implement prioritized planner here
"""
import time



def run_prioritized_planner(aircraft_list, nodes_dict, edges_dict, heuristics, constraints, dt, t):
    """
    function gets called when a new AC has spawned, it then computes a path for this AC
    with respect to the constraints already in place
    Args:
        dt: time step difference
        constraints: list of all constraints for all AC
        aircraft_list: all aircraft currently on the field
        nodes_dict:
        edges_dict:
        heuristics:
        t: current time step

    Returns:
        computing time
        expanded nodes
    """
    deadlocks = 0
    expanded_nodes = 0  # KPI
    start = time.perf_counter_ns()     # KPI

    for ac in aircraft_list:
        if ac.spawntime == t:   # so first come first serve priority
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]
            exp_nodes, constraints, deadlcks = ac.plan_prioritized(nodes_dict, edges_dict, heuristics,
                                                                   constraints, dt, t)
            deadlocks += deadlcks
            expanded_nodes += exp_nodes
    stop = time.perf_counter_ns()
    time_delta = stop - start   # in nanoseconds

    return time_delta, expanded_nodes, deadlocks