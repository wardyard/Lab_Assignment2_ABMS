'''
individual planner where AC agents plan their routes themselves
'''
import time


def run_individual_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t):
    # extract dictionary with nodeID keys and corresponding AC on this node
    radar_dict = radar(aircraft_lst)
    return None


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
