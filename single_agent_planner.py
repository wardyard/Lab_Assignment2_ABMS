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
    #print("No path found, "+str(len(closed_list))+" nodes visited")
    return False, [], expanded_nodes    # Failed to find solutions


def build_constraint_table(constraints, spawntime):
    """
    builds a dictionary where the keys are timesteps. For these keys, a list of constraints corresponding to
    this timestep is constructed. Whether or not a constraint is applicable for a certain aircraft is decided
    based on the spawning time of the aircraft: if the constraint was constructed before or at this time, it applies
    to the aircraft
    Args:
        spawntime: spawntime of the aircraft
        constraints: list of all constraints

    Returns:
        constraint_table
    """
    #if len(constraints) == 0:
        #print('no constraints for timestep ' + str(spawntime) )
    # constraints will be indexed by timestep:
    constraint_table = dict()
    for constraint in constraints:
        if constraint['spawntime'] <= spawntime:
            # check if key timestep already has constraints
            if constraint['timestep'] not in constraint_table:
                # constraint_table[constraint['timestep']] = []
                constraint_table[constraint['timestep']] = []
                # constraint_table[constraint['timestep']].append(constraint)
                constraint_table[constraint['timestep']].append(constraint)
            else:
                # constraint_table[constraint['timestep']].append(constraint)
                constraint_table[constraint['timestep']].append(constraint)
    #print('constraint table for spawntime ' + str(spawntime) +': ' + str(constraint_table))
    return constraint_table


def build_constraint_table_cbs(constraints, acid):
    """
        builds a dictionary where the keys are timesteps. For these keys, a list of constraints corresponding to
        this timestep is constructed. Whether or not a constraint is applicable for a certain aircraft is decided
        based on the aircraft ID time of the aircraft
        Args:
            acid: aircraft ID of the aircraft for which the table has to be constructed
            constraints: list of all constraints

        Returns:
            constraint_table
        """
    # edge constraint form: ['acid': 2, 'loc': [35, 39], 'timestep': 1.5]
    # vertex constraint form: ['acid': 2, 'loc': [35], 'timestep': 1.5]

    #if len(constraints) == 0:
        #print('No CBS constraints for acid  ' + str(acid))
    # constraints will be indexed by timestep:
    constraint_table = dict()
    for constraint in constraints:
        if constraint['acid'] == acid:
            # check if key timestep already has constraints
            if constraint['timestep'] not in constraint_table:
                # constraint_table[constraint['timestep']] = []
                constraint_table[constraint['timestep']] = []
                # constraint_table[constraint['timestep']].append(constraint)
                constraint_table[constraint['timestep']].append(constraint)
            else:
                # constraint_table[constraint['timestep']].append(constraint)
                constraint_table[constraint['timestep']].append(constraint)
    #print('CBS constraint table for acid ' + str(acid) + ': ' + str(constraint_table))
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
def astar(nodes_dict, from_node, goal_node, heuristics, constraints, start_time, dt, acid, cbs, ac):
    """
    Single agent A* search. Time start can only be the time that an agent is at a node.
    INPUT:
        - nodes_dict = [dict] dictionary with nodes and node properties
        - from_node = [int] node_id of node from which planning is done
        - goal_node = [int] node_id of node to which planning is done
        - heuristics = [dict] dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
        - time_start = [float] planning start time.
        - constraints = the set of global constraints
        - acid, aircraft ID, only relevant for CBS planning
        - cbs: True if the planning happens via CBS, False otherwise
        - ac: the aircraft object for which to plan the trajectory. This parameter is used to forbid 180° turns via retrieval of the heading
    RETURNS:
        - success = True/False. True if path is found and False is no path is found
        - path = list of tuples with (loc, timestep) pairs -> example [(37, 1), (101, 2)]. Empty list if success == False.
        - expanded_nodes = the amount of nodes expanded for reaching the solution
    """
    # max f_value limit for a node, if the f_value goes above this limit, we can safely say we're in a deadlock
    max_f_value = 100
    # expanded nodes performance indicator
    expanded_nodes = 0

    from_node_id = from_node
    goal_node_id = goal_node
    time_start = start_time

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = time_start
    h_value = heuristics[from_node_id][goal_node_id]
    # construct constraint table for this AC based on whether we're planning using CBS or not
    if not cbs:
        constraint_table = build_constraint_table(constraints, time_start)
    elif cbs:
        constraint_table = build_constraint_table_cbs(constraints, acid)

    root = {'loc': from_node_id, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': time_start}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root
    while len(open_list) > 0:
        # pop node with smallest f_value
        curr = pop_node(open_list)
        # check if the cost of the current node is too high. This denotes a deadlock situation
        if curr['g_val'] + curr['h_val'] > max_f_value:
            # didn't find a solution, deadlock
            break
        # one extra expanded node
        expanded_nodes += 1
        # determine path so far
        if len(ac.current_path) > 0:
            index_curr_timestep = ac.current_path.index(list(filter(lambda node_t_pair: node_t_pair[1] == start_time
                                                                    in node_t_pair, ac.current_path))[0])
            path = ac.current_path[:index_curr_timestep] + get_path(curr)
        else:
            path = get_path(curr)
        # current node is the goal node, we've found a path
        if curr['loc'] == goal_node_id and curr['timestep'] >= earliest_goal_timestep:
            return True, get_path(curr), expanded_nodes
        # no 180 turns are allowed
        for neighbor in nodes_dict[curr['loc']]["neighbors"]:
            # check if moving to this neighbor is constrained, if not, a new child node may be constructed
            constrained = is_constrained(curr['loc'], neighbor, curr['timestep'] + dt, constraint_table)
            # if the selected neighbor causes the AC to make a 180° move, forbid the move
            # note: is an AC is stationary at a node for a certain amount of timesteps, it cannot
            # return to the node it was at before this one either
            if len(path) > 1 and not constrained:
                last_node = path[-1][0]  # equal to 'current position' in the path construction
                index = -1
                different = False
                while not different and len(path) > index * (-1):
                    index += -1
                    different = last_node != path[index][0]
                if different:
                    diff_node = path[index][0]
                    # #print('diff_node: ' + str(diff_node))
                    if neighbor == diff_node:
                        constrained = True
                # else:
                  #  #print('no different node in previous positions')
            # 180° constraints for CBS
            # current heading of the AC
            ac_heading = ac.heading
            if len(path) <= 1 and not constrained and curr['loc'] != ac.start:
            # if not constrained and curr['loc'] != ac.start:
                # xy pos of the neighboring node
                neighbor_xy = nodes_dict[neighbor]["xy_pos"]
                # xy pos of the current node
                curr_xy = nodes_dict[curr['loc']]["xy_pos"]
                # chenge the heading of AC for a fictional move to neighbor
                ac.get_heading(curr_xy,neighbor_xy)
                # retreive the updated heading
                ac_heading_after_move = ac.heading
                # if the heading differs by 180°, the move should be constrained
                if abs(ac_heading - ac_heading_after_move) == 180:
                    constrained = True
                # restore the original AC heading
                ac.heading = ac_heading
            if not constrained:
                child = {'loc': neighbor,
                         'g_val': curr['g_val'] + dt,
                         'h_val': heuristics[neighbor][goal_node_id],
                         'parent': curr,
                         'timestep': curr['timestep'] + dt}
                if (child['loc'], child['timestep']) in closed_list:
                    existing_node = closed_list[(child['loc'], child['timestep'])]
                    if compare_nodes(child, existing_node):
                        closed_list[(child['loc'], child['timestep'])] = child
                        push_node(open_list, child)
                else:
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)

        # add child node which stays at same position for 1 timestep
        constrained = is_constrained(curr['loc'], curr['loc'], curr['timestep'] + dt, constraint_table)
        if not constrained:
            child_loc = curr['loc']
            # there was a mistake here where the timestep got incremented by 1. Fixed this
            # and the head-on collisions seemed to be fixed
            child = {'loc': child_loc,
                     'g_val': curr['g_val'] + 1,  # was curr['g_val'] + 1
                     'h_val': heuristics[child_loc][goal_node_id],
                     'parent': curr,
                     'timestep': curr['timestep'] + dt}

            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)

    return False, [], expanded_nodes  # Failed to find solutions


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node['timestep'], node))


def pop_node(open_list):
    _, _, _, _, curr = heapq.heappop(open_list)
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
   
    return path
