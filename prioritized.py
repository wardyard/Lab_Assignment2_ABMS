"""
Implement prioritized planner here
"""
import time

# constraints is an instance variable, we start with an empty list for the first spawned AC
constraints = []
# TODO: remove constraints for AC which have arrived at their destination

def run_prioritized_planner(aircraft_list, nodes_dict, edges_dict, heuristics, t ):
    """
    function gets called when a new AC has spawned, it then computes a path for this AC
    with respect to the constraints already in place
    Args:
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
    start = time.time()     # KPI

    for ac in aircraft_list:
        if ac.spawntime == t:   # so first come first serve priority
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]
            exp_nodes, constrnts = ac.plan_prioritized(nodes_dict, edges_dict, heuristics, constraints, t)
            constraints = constrnts     # TODO: check inheritance
            expanded_nodes += exp_nodes

    raise Exception("Prioritized planner not defined yet.")
    return