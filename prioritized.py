"""
Implement prioritized planner here
"""
import time


# TODO: remove constraints for AC which have arrived at their destination

def run_prioritized_planner(aircraft_list, nodes_dict, edges_dict, heuristics, constraints, t ):
    """
    function gets called when a new AC has spawned, it then computes a path for this AC
    with respect to the constraints already in place
    Args:
        constraints:
        aircraft_list:
        nodes_dict:
        edges_dict:
        heuristics:
        t:

    Returns:
        computing time
        expanded nodes
    """
    expanded_nodes = 0  # KPI
    start = time.perf_counter_ns()     # KPI

    for ac in aircraft_list:
        if ac.spawntime == t:   # so first come first serve priority
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]
            exp_nodes, constraints = ac.plan_prioritized(nodes_dict, edges_dict, heuristics, constraints, t)
            expanded_nodes += exp_nodes
    stop = time.perf_counter_ns()
    time_delta = stop - start   # in nanoseconds

    return time_delta, expanded_nodes