"""
Implement CBS here
"""
import time
import heapq
from single_agent_planner import astar


def run_CBS(aircraft_list, nodes_dict, edges_dict, heuristics, dt, t):
    """
    Performs CBS planning. Everytime a new AC has spawned, the CBS algorithm plans the paths for all AC in the map.
    AC which have arrived at their destination will not be counted in planning
    Args:
        aircraft_list: the global aircraft list, containing both arrived and taxiing aircraft
        nodes_dict: nodes of the grid
        edges_dict: edges
        heuristics:
        dt: timestep difference
        t: current timestep in simulation

    Returns:
        time_delta: computation time
        expanded_nodes: number of expanded nodes needed for a solution
        deadlocks: amount of deadlocks occured
    """
    deadlocks = 0
    expanded_nodes = 0  # KPI
    start = time.perf_counter_ns()  # KPI
    for ac in aircraft_list:
        if ac.spawntime == t:   # only is a new AC has spawned, CBS will be ran again
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]
            # ac.from_to = [0, 0]
            exp_nodes, deadlcks = cbs(aircraft_list, nodes_dict, edges_dict, heuristics, dt, t)
            deadlocks += deadlcks
            expanded_nodes += exp_nodes
    stop = time.perf_counter_ns()
    time_delta = stop - start  # in nanosecond

    return time_delta, expanded_nodes, deadlocks


def cbs(aircraft_list, nodes_dict, edges_dict, heuristics, dt, t):
    """
    Does the actual path plannning.
    Args:
        aircraft_list: the global aircraft list
        nodes_dict:
        edges_dict:
        heuristics:
        dt: time step difference
        t: current simulation time step

    Returns:
        number of expanded nodes needed to solve the system at timestep t
    """
    #print('------------------------------------------------------------')
    open_list = []
    num_of_generated = 0
    num_of_expanded = 0     # will be updated and returned upon success
    num_of_deadlocks = 0    # deadlocks performance indicator
    deadlock_ac = []        # list of AC which were in a deadlock

    # generate root node with no constraints. Paths of root node are planned independently.
    # Acids denotes the list of aircraft IDs for which a path should be planned. The paths in p['paths'] correspond
    # to the ACID of the same index in p['acids'].
    # The cost is the sum of path lengths.
    # Constraints is a list with the constraints applicable to this node only. Every child node will add 1 constraint to
    # this list.
    # Collisions is the list of collisions that still remain for this solution of paths
    root = {
        'cost': 0,
        'constraints': [],
        'paths': [],
        'collisions': [],
        'acids': []
    }
    # first plan the independent paths for every AC
    for ac in aircraft_list:
        # prevent arrived aircraft to still influence the planning, only AC on the map are accounted for
        if ac.status == "taxiing":
            # plan path from current AC location and timestep, since the AC might have already moved on the grid
            # the start location shouldn't be used per se. Note the difference between rpioritized here: prioritized
            # plans for every AC from spawntime, CBS replans for every AC from the current time onwards

            # current AC position is start position if it hasn't moved, and from_to[0] if it has
            curr_pos = ac.from_to[0] if len(ac.path_to_goal) > 0 else ac.start
            # no constraints are given to astar for the root node. ac.id is a parameter because it is used
            # to construct the constraint table since this has a different format for CBS (see single_agent_planner)
            success, path, expanded_nodes = astar(nodes_dict, curr_pos, ac.goal, heuristics, [], t, dt, ac.id, True, ac)
            # update KPI
            num_of_expanded += expanded_nodes
            if success:
                # update the path instance variables of ac
                ac.update_path_variables(ac, path, t)
                # append the constructed path to the root node
                root['paths'].append(path)
                # append ACID of this AC to the acids list
                root['acids'].append(ac.id)
            else:
                ac.status = "deadlocked"
                num_of_deadlocks += 1
                raise BaseException('Deadlock CBS independent paths')

    # determine cost of root node paths
    root['cost'] = get_sum_of_cost(root['paths'])
    # detect all the collisions that this solution of paths bring with it
    root['collisions'] = detect_collisions(root['paths'], root['acids'], dt)
    # push this node to the open list since it's not fully expanded. this function return the number of generated nodes
    # which is semi handy but not really used
    num_of_generated = push_node(open_list, root, num_of_generated)

    # if there are still nodes unexplored:
    while len(open_list) > 0:
        # pop node with smallest cost. Function returns the number of expanded nodes so far
        p, num_of_expanded = pop_node(open_list, num_of_expanded)
        # if there are no collisions, the solution is optimal
        if not p['collisions']:
            # loop over all the AC currently in the aircraft list to find aircraft which are used by p node
            for ac1 in aircraft_list:
                acid = ac1.id
                # if the aircraft is currently in the map:
                if acid in p['acids']:
                    # determine index of this ACID in the p['acids'] list
                    index_p_acids = p['acids'].index(acid)
                    # retrieve the path corresponding to this ACID
                    acid_path = p['paths'][index_p_acids]
                    # update the Aircraft instance variables
                    ac1.update_path_variables(ac1, acid_path, t)
                    if acid_path[0][1] != t:
                        raise Exception("Something is wrong with the timing of the path planning")
            return num_of_expanded, num_of_deadlocks

        # there are collisions, extract 1 of them
        collision = p['collisions'][0]
        # create 2 constraints out of the collision
        # note that constraints have different format than for prioritized planning due to spawntime not being
        # important for constraint obedience anymore
        constraints = standard_splitting(collision, dt)
        # loop over both constraints, generating a new node for each one
        for constraint in constraints:
            # current constraints from parent node
            p_constraints = p['constraints'] if p['constraints'] is not None else []
            # make a copy of these constraints and append the constraint used in this loop
            # this way, we'll create a child node with just 1 extra constraint
            q_constraints = p_constraints.copy()
            q_constraints.append(constraint)
            # create child node. The paths will be updated later on
            q = {
                'cost': 0,
                'constraints': q_constraints,
                'paths': p['paths'].copy(),
                'collisions': [],
                'acids': p['acids'].copy() # added 24 oct
            }

            # find out the aircraft ID of the aircraft in the constraint
            q_acid = constraint['acid']
            # extract the aircraft itself from the global aircraft list
            aircr = None
            for ac2 in aircraft_list:
                if ac2.id == q_acid:
                    aircr = ac2
            # calculate path for the aircraft with a new constraint
            if aircr is not None:
                # extract path corresponding to this aircraft in p
                index_p_acids = p['acids'].index(q_acid)
                p_path = p['paths'][index_p_acids].copy()
                # update current path of aircraft using this obtained path. The current path needs to be up to date with
                # the current node path to ensure proper indexing
                aircr.update_path_variables(aircr, p_path, t)
                # current position is different from start node if AC has already moved
                curr_pos = aircr.from_to[0] if len(aircr.path_to_goal) > 0 else aircr.start
                # determine path with extra constraints
                success, q_path, expanded_nodes = astar(nodes_dict, curr_pos, aircr.goal, heuristics, q_constraints, t,
                                                        dt, q_acid, True, aircr)
                # update KPI
                num_of_expanded += expanded_nodes
                # if a path was found, update the AC instance variables and update Q node
                if success:
                    # find paths index of this aircraft via the ACIDS list
                    index = q['acids'].index(q_acid)
                    # replace the path corresponding to this AC
                    q['paths'][index] = q_path
                    q['collisions'] = detect_collisions(q['paths'], q['acids'], dt)
                    q['cost'] = get_sum_of_cost(q['paths'])
                    num_of_generated = push_node(open_list, q, num_of_generated)
                else:
                    # unable to find path for this AC. It shouldn't be removed tho, but this node should get a really
                    # high cost
                    print('no path found CBS, assigning infinite cost')
                    q['cost'] = float('inf')

            else:
                raise Exception("CBS: no aircraft found in node list with ID: " + str(q_acid))

    return None


def detect_collision(path1, path2, dt):
    """
    Returns first collsion that appears between the 2 paths.
    - vertex collision has the form [[node], timestep]
    - edge collision has the form [[node1, node2], timestep]
    """
    # iterate over longest path to ensure goal collisions are noticed as well
    longest_path = path1 if len(path1) > len(path2) else path2
    for node_time_pair in longest_path:
        # extract timestep of node_time_pair in path
        timestep = node_time_pair[1]

        loc1 = get_location(path1, timestep)
        loc2 = get_location(path2, timestep)

        loc1_t_1 = get_location(path1, timestep + dt)
        loc2_t_1 = get_location(path2, timestep + dt)
        # check is current path locations are valid (so AC hasn't reached destination)
        curr_loc_valid = loc1 is not None and loc2 is not None
        # vertex collision
        # loc is None when t > max t in path, meaning the AC has arrived.
        if curr_loc_valid and loc1 == loc2:
            return [[loc1], timestep]
        # check whether 2 AC will be at departing runway at the same time
        # the collision format will be the same as an edge constraint, but standard_splitting is modified
        # to account for this special type of constraint
        elif curr_loc_valid and (loc1 == 1.0 or loc2 == 1.0) and (loc1 == 2.0 or loc2 == 2.0) and loc1 != loc2:
            ##print('RUNWAY CONSTRAINT DETECT_COLLISION')
            return [[loc1, loc2], timestep]
        # edge collision
        elif loc1 is not None and loc2 is not None and loc1_t_1 is not None and loc2_t_1 is not None and loc1 == loc2_t_1 and loc2 == loc1_t_1:
            return [[loc1, loc2], timestep]  # used to be timestep + dt
    return None


def get_location(path, t):
    """
    returns location of AC if it follows path at time t
    returns None if the AC has reached its goal by timestep t. It will be deleted from the
    map by then
    """
    # paths has following structure: [(node1, t1), (node2, t2), (node3, t3), ...],
    # we only want the nodes
    if t < 0:
        return path[0][0]
    elif t <= path[-1][1]:
        for node_time_pair in path:
            if node_time_pair[1] == t:  # if this node_time_pair is for the correct timestep
                return node_time_pair[0]  # return node
    else:   # AC has arrived, it won't have a position anymore
        return None


def detect_collisions(paths, acids, dt):
    """
    Returns a list of all first collisions between 2 paths of the set of paths
    Args:
        paths: all the current paths
        acids: the aircraft IDs for which the paths are calculated
        dt: time step difference

    Returns:
        list of collisions formatted in the same way as detect_collision

    """

    # paths have following structure [(node1, t1), (node2, t2), (node3, t3), ...]
    # we always need to compare two paths to eachother and iterate over all path combinations,
    # avoiding doubles

    # important! collisions have this format: {'acid1': 5, 'acid2': 3, 'loc': [32.93], 'timestep':2.5}
    # this different formatting causes the constraints to have a different format as well, see standard_splitting()

    combs = []
    collisions = []

    for i in range(len(paths)):  # loop over all paths
        for j in range(len(paths)):  # loop over all other paths
            # if this combination hasn't already been treated
            if not ((i, j) in combs or (j, i) in combs) and i != j:
                # paths are not compared yet, find collision
                collision = detect_collision(paths[i], paths[j], dt)
                if collision is not None:
                    collisions.append({'acid1': acids[i], 'acid2': acids[j], 'loc': collision[0], 'timestep': collision[1]})
                combs.append((i, j))

    return collisions


def standard_splitting(collision, dt):
    """
    returns a list of 2 constraints to resolve the collision
    Args:
        collision: collision to be resolved
        dt: time step difference

    Returns:
        - list with 2 constranits that can fix the collision, 1 for every AC involved
    """

    # vertex collision has following structure: {'acid1': 0, 'acid2': 1, 'loc': [35], 'timestep': 5}
    # edge collision has following structure: {'acid1': 0, 'acid2': 1, 'loc': [36, 63], 'timestep': 5}

    # important! due to the different collision format, the constraint format will be different as well.
    # this causes the function build_constraint_table to be implemented differently. The choice was made to
    # create a new function, build_constraint_table_cbs. In the a_star function, it will first be checked whether
    # it is optimizing for prioritized or cbs and then use the correct constraint table builder
    constraints = []

    loc = collision['loc']
    timestep = collision['timestep']

    # edge constraint
    if len(loc) > 1:
        loc1 = loc[0]
        loc2 = loc[1]
        # same departure time constraint
        if (loc1 == 2 or loc2 == 2) and (loc1 == 1 or loc2 == 1 ) and loc1 != loc2:
            constraint1 = {'acid': collision['acid1'], 'loc': [loc1], 'timestep': timestep}
            constraint2 = {'acid': collision['acid2'], 'loc': [loc2], 'timestep': timestep}
        else:
            constraint1 = {'acid': collision['acid1'], 'loc': [loc[0], loc[1]], 'timestep': timestep + dt}
            # reverse positions for second constraint
            constraint2 = {'acid': collision['acid2'], 'loc': [loc[1], loc[0]], 'timestep': timestep + dt}
        constraints.append(constraint1)
        constraints.append(constraint2)

    # vertex constraint
    elif len(collision['loc']) == 1:
        constraint1 = {'acid': collision['acid1'], 'loc': loc, 'timestep': timestep}
        constraint2 = {'acid': collision['acid2'], 'loc': loc, 'timestep': timestep}
        constraints.append(constraint1)
        constraints.append(constraint2)

    return constraints


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def push_node(open_list, node, num_of_generated):
    """
    pushes node to open_list while maintaining ordering of minimum cost
    Args:
        open_list:
        node: node to be pushed to open list
        num_of_generated:

    Returns:
        number of generated nodes so far, not really relevant right now
    """
    heapq.heappush(open_list, (node['cost'], len(node['collisions']), num_of_generated, node))

    num_of_generated += 1
    return num_of_generated


def pop_node(open_list, num_of_expanded):
    """
    extracts node with the smallest cost tha hasn't been expanded fully
    Args:
        open_list:
        num_of_expanded: current number of expanded nodes

    Returns:
        - number of expanded nodes after popping
    """
    _, _, _, node = heapq.heappop(open_list)
    num_of_expanded += 1
    return node, num_of_expanded
