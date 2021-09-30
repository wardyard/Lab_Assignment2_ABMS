"""
This file contains single agent planner functions  that can be used by other planners.
Consider functions in this file as supporting functions.
"""

import heapq
import networkx as nx

def calc_heuristics(graph, nodes_dict):
    """
    Calculates the exact heuristic dict (shortest distance between two nodes) to be used in A* search.
    INPUT:
        - graph = networkX graph object
        - nodes_dict = dictionary with nodes and node properties
    RETURNS:
        - heuristics = dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
    """
    
    heuristics = {}
    for i in nodes_dict:
        heuristics[nodes_dict[i]["id"]] = {}
        for j in nodes_dict:
            path, path_length = heuristicFinder(graph, nodes_dict[i]["id"], nodes_dict[j]["id"])
            if path == False:
                pass
            else:
                heuristics[nodes_dict[i]["id"]][nodes_dict[j]["id"]] = path_length
    return heuristics

def heuristicFinder(graph, start_node, goal_node):
    """
    Finds exact distance between start_node and goal_node using the NetworkX graph.
    INPUT:
        - graph = networkX graph object
        - start_node, goal_node [int] = node_ids of start and goal node
    RETURNS:
        - path = list with node_ids that are part of the shortest path
        - path_length = length of the shortest path
    """
    try:
        path = nx.dijkstra_path(graph, start_node, goal_node, weight="weight")
        path_length = nx.dijkstra_path_length(graph, start_node, goal_node)
    except:
        path = False
        path_length = False
        raise Exception('Heuristic cannot be calculated: No connection between', start_node, "and", goal_node)
    return path, path_length


def simple_single_agent_astar(nodes_dict, from_node, goal_node, heuristics, time_start):
    # def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    # def a_start(nodes_dict, from_node, goal_node, heuristics, time_start, constraints)
    """
    Single agent A* search. Time start can only be the time that an agent is at a node.
    INPUT:
        - nodes_dict = [dict] dictionary with nodes and node properties
        - from_node = [int] node_id of node from which planning is done
        - goal_node = [int] node_id of node to which planning is done
        - heuristics = [dict] dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
        - time_start = [float] planning start time. 
        - Hint: do you need more inputs? -> yes, constraints
    RETURNS:
        - success = True/False. True if path is found and False is no path is found
        - path = list of tuples with (loc, timestep) pairs -> example [(37, 1), (101, 2)]. Empty list if success == False.
    """
    # expanded nodes performance indicator
    expanded_nodes = 0

    from_node_id = from_node
    goal_node_id = goal_node
    time_start = time_start
    
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = time_start
    h_value = heuristics[from_node_id][goal_node_id]
    root = {'loc': from_node_id, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': time_start}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        # one extra expanded node
        expanded_nodes += 1

        if curr['loc'] == goal_node_id and curr['timestep'] >= earliest_goal_timestep:
            return True, get_path(curr), expanded_nodes
        
        for neighbor in nodes_dict[curr['loc']]["neighbors"]:
            child = {'loc': neighbor,
                    'g_val': curr['g_val'] + 0.5,
                    'h_val': heuristics[neighbor][goal_node_id],
                    'parent': curr,
                    'timestep': curr['timestep'] + 0.5}
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)
    print("No path found, "+str(len(closed_list))+" nodes visited")
    return False, [], expanded_nodes    # Failed to find solutions


def build_constraint_table(constraints, spawntime):
    """
    builds a dictionary where the keys are timesteps. For these keys, a list of constraints corresponding to
    this timestep is constructed. Whether or not a constraint is applicable for a certain aircraft is decided
    based on the spawning time of the aircraft: if the constraint was constructed before or at this time, it applies
    to the aircraft
    Args:
        constraints:
        spawntime of the aircraft

    Returns:
        constraint_table
    """
    # constraints will be indexed by time step:
    constraint_table = dict()
    for constraint in constraints:
        if constraint['spawntime'] <= spawntime:
            # check if key if timestep already has constraints
            if constraint['timestep'] not in constraint_table:
                constraint_table[constraint['timestep']] = []
                constraint_table[constraint['timestep']].append(constraint)
            else:
                constraint_table[constraint['timestep']].append(constraint)
    print('constraint table for spawntime ' + str(spawntime) +': ' + str(constraint_table))
    return constraint_table


def is_constrained(curr_node, next_node, next_time, constraint_table):
    """
    checks whether the current move is constrained or not for this aircraft.
    Args:
        curr_node: node at which AC is currently
        next_node: node to which AC wants to move
        next_time: newt timestep
        constraint_table: constraint table for this AC

    Returns:
        true if AC can't make this move, false elsewhere
    """
    if constraint_table.get(next_time) is not None:
        constraints = constraint_table.get(next_time)
        for constraint in constraints:
            constr_loc = constraint['loc']

            # for edge constraints
            if len(constr_loc) > 1 and constr_loc[0] == curr_node and constr_loc[1] == next_node:
                return True

            # for vertex constraint
            elif len(constr_loc) == 1 and constr_loc[0] == next_node:
                return True

    return False


# added to keep the simple agent astar working until we finish this legit one
def astar(nodes_dict, from_node, goal_node, heuristics, constraints, spawntime):
    """
    Single agent A* search. Time start can only be the time that an agent is at a node.
    INPUT:
        - nodes_dict = [dict] dictionary with nodes and node properties
        - from_node = [int] node_id of node from which planning is done
        - goal_node = [int] node_id of node to which planning is done
        - heuristics = [dict] dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
        - time_start = [float] planning start time.
        - constraints = the set of global constraints
    RETURNS:
        - success = True/False. True if path is found and False is no path is found
        - path = list of tuples with (loc, timestep) pairs -> example [(37, 1), (101, 2)]. Empty list if success == False.
        - expanded_nodes = the amount of nodes expanded for reaching the solution
    """
    # expanded nodes performance indicator
    expanded_nodes = 0

    from_node_id = from_node
    goal_node_id = goal_node
    time_start = spawntime

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = time_start
    h_value = heuristics[from_node_id][goal_node_id]
    # construct constraint table for this AC
    constraint_table = build_constraint_table(constraints, time_start)
    root = {'loc': from_node_id, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': time_start}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        # one extra expanded node
        expanded_nodes += 1

        if curr['loc'] == goal_node_id and curr['timestep'] >= earliest_goal_timestep:
            return True, get_path(curr), expanded_nodes

        for neighbor in nodes_dict[curr['loc']]["neighbors"]:
            # check if moving to this neighbor is constrained, if not, a new child node may be constructed
            constrained = is_constrained(curr['loc'], neighbor, curr['timestep'] + 0.5, constraint_table)
            if not constrained:
                child = {'loc': neighbor,
                         'g_val': curr['g_val'] + 0.5,
                         'h_val': heuristics[neighbor][goal_node_id],
                         'parent': curr,
                         'timestep': curr['timestep'] + 0.5}
                if (child['loc'], child['timestep']) in closed_list:
                    existing_node = closed_list[(child['loc'], child['timestep'])]
                    if compare_nodes(child, existing_node):
                        closed_list[(child['loc'], child['timestep'])] = child
                        push_node(open_list, child)
                else:
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)

        # add child node which stays at same position for 1 timestep
        constrained = is_constrained(curr['loc'], curr['loc'], curr['timestep'] + 1, constraint_table)
        if not constrained:
            child_loc = curr['loc']
            child = {'loc': child_loc,
                     'g_val': curr['g_val'] + 1,  # was curr['g_val'] + 1
                     'h_val': heuristics[child_loc][goal_node_id],
                     'parent': curr,
                     'timestep': curr['timestep'] + 1}

            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)
    print("No path found, " + str(len(closed_list)) + " nodes visited")
    return False, [], expanded_nodes  # Failed to find solutions

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))

def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr

def compare_nodes(n1, n2):
    """Return true if n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

def get_path(goal_node):
    """Construct path if goal node is reached"""
    path = []
    curr = goal_node
    while curr is not None:
        path.append((curr['loc'], curr['timestep']))
        curr = curr['parent']
    path.reverse()
    #print(path)
    return path