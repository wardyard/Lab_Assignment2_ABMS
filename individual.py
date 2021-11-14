'''
individual planner where AC agents plan their routes themselves
'''
import time


def run_individual_planner(aircraft_lst, nodes_dict, heuristics, t, dt, observation_size):
    deadlocks = 0
    expanded_nodes = 0

    # indicates whether there were still collisions detected when planning for all AC
    collisions_detected = 0
    start = time.perf_counter_ns()

    # contains all the ACIDs of the AC which are deadlocked at this timestep
    deadlock_acids = []

    # run a first planning loop, this should already solve most of the collisions
    for ac in aircraft_lst:
        if ac.status == "taxiing":
            # extract dictionary with nodeID keys and corresponding AC on this node
            radar_dict = radar(aircraft_lst)
            create_observation_space(ac, radar_dict, nodes_dict, observation_size)
            observed_ac = ac.scan()
            exp_nodes, deadlcks, deadlock_ac, detected_col = ac.perform_ind_planning(observed_ac, t, dt, heuristics,
                                                                                      deadlock_acids, observation_size)
            collisions_detected += detected_col
            # if deadlock situations occurred, update its state to deadlocked so it won't be planned or moved
            if len(deadlock_ac) > 0:
                deadlocks += deadlcks
                for locked_ac in deadlock_ac:
                    locked_ac.status = "deadlocked"
            expanded_nodes += exp_nodes

    # re run the planning until there are no more collisions between aircraft
    while collisions_detected > 0:
        collisions_detected = 0
        for ac in aircraft_lst:
            if ac.status == "taxiing":
                # extract dictionary with nodeID keys and corresponding AC on this node
                radar_dict = radar(aircraft_lst)
                create_observation_space(ac, radar_dict, nodes_dict, observation_size)
                observed_ac = ac.scan()
                exp_nodes, deadlcks, deadlock_ac, detected_col = ac.perform_ind_planning(observed_ac, t, dt,
                                                                                          heuristics, deadlock_acids,
                                                                                          observation_size)
                collisions_detected += detected_col
                # if deadlock situations occurred, update its state to deadlocked so it won't be planned or moved
                if len(deadlock_ac) > 0:
                    deadlocks += deadlcks
                    for locked_ac in deadlock_ac:
                        locked_ac.status = "deadlocked"
                expanded_nodes += exp_nodes


    stop = time.perf_counter_ns()
    time_delta = stop - start
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
    # start with empty observation space
    ac.observation_space = dict()
    curr_ac_node = ac.from_to[0] if ac.from_to[0] != 0 else ac.start

    # loop over nieghbors of current position node
    for neighbor in nodes_dict[curr_ac_node]["neighbors"]:
        if neighbor in radar_dict:
            ac.observation_space[neighbor] = radar_dict[neighbor]
        else:
            ac.observation_space[neighbor] = None
        # If AC can look more than 1 node ahead
        if size > 1:
            for neighbor2 in nodes_dict[neighbor]["neighbors"]:
                if neighbor2 in radar_dict:
                    ac.observation_space[neighbor2] = radar_dict[neighbor2]
                else:
                    # current node will be looped over a few times, but that's no prob I think
                    ac.observation_space[neighbor2] = None
                if size > 2:
                    for neighbor3 in nodes_dict[neighbor2]["neighbors"]:
                        if neighbor3 in radar_dict:
                            ac.observation_space[neighbor3] = radar_dict[neighbor3]
                        else:
                            # current node will be looped over a few times, but that's no prob I think
                            ac.observation_space[neighbor3] = None
                        if size > 3:
                            for neighbor4 in nodes_dict[neighbor3]["neighbors"]:
                                if neighbor4 in radar_dict:
                                    ac.observation_space[neighbor4] = radar_dict[neighbor4]
                                else:
                                    ac.observation_space[neighbor4] = None

    return None