'''
individual planner where AC agents plan their routes themselves
'''
import time


def run_individual_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, dt, observation_size):
    deadlocks = 0
    expanded_nodes = 0  # KPI
    start = time.perf_counter_ns()
    # extract dictionary with nodeID keys and corresponding AC on this node
    radar_dict = radar(aircraft_lst)
    for ac in aircraft_lst:
        if ac.spawntime == t:
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]
        create_observation_space(ac, radar_dict, nodes_dict, observation_size)
        observed_ac = ac.scan()
        exp_nodes, deadlcks = ac.perform_ind_planning(observed_ac, t, dt, heuristics)
        deadlocks += deadlcks
        expanded_nodes += exp_nodes

    stop = time.perf_counter_ns()
    # computing time performance indicator
    time_delta = stop - start   # in nanoseconds
    return time_delta, expanded_nodes, deadlocks


def radar(aircraft_list):
    """
    creates a dictionary with node IDs as keys and a list with the AC which are currently at this node.
    Nodes which don't have an AC on them will not be added to the list for memory management
    Args:
        aircraft_list:

    Returns:
        dictionary in the form of {node_ID: AC, node_ID2: AC2}
    """
    radar_dict = dict()
    for ac in aircraft_list:
        if ac.status == "taxiing":
            # if AC has already moved, ac.from_to[0] will have al value different to 0
            ac_node_id = ac.from_to[0] if ac.from_to[0] != 0 else ac.start
            if ac_node_id not in radar_dict:
                radar_dict[ac_node_id] = ac
            else:
                raise BaseException('More than 1 AC at same node in radar dict')
    return radar_dict


def create_observation_space(ac, radar_dict, nodes_dict, size):
    """
    creates the observation space for the current aicraft. The size determines how far the AC can look.
    Size = 1: Ac can only look at neighboring nodes
    Size = 2: Ac can look to nieghboring nodes an their neighbors
    This function modifies/initializes the observation_space instance variable of the Aircraft class, which was created
    for this function. Observation_space is a dictionary with node IDs as keys and aircraft on this node as values.
    If there's no Ac on this ID, None will be added as value
    Args:
        ac: AC for which to construct the observation space
        radar_dict: current AC in the field and their nodes
        nodes_dict:
        size: how far the AC can look ahead

    Returns:
        None
    """
    curr_ac_node = ac.from_to[0] if ac.from_to[0] != 0 else ac.start

    # loop over nieghbors of current position node
    for neighbor in nodes_dict[curr_ac_node]["neighbors"]:
        if neighbor in radar_dict:
            ac.observation_space[neighbor] = radar_dict[neighbor]
        else:
            ac.observation_space[neighbor] = None
        # If AC can look more than 1 node ahead
        if size > 1:
            for neighborr in nodes_dict[neighbor]["neighbors"]:
                if neighborr in radar_dict:
                    ac.observation_space[neighborr] = radar_dict[neighborr]
                else:
                    # current node will be looped over a few times, but that's no prob I think
                    ac.observation_space[neighborr] = None
            if size > 2:
                for neighborrr in nodes_dict[neighborr]["neighbors"]:
                    if neighborrr in radar_dict:
                        ac.observation_space[neighborrr] = radar_dict[neighborrr]
                    else:
                        # current node will be looped over a few times, but that's no prob I think
                        ac.observation_space[neighborrr] = None
    return None