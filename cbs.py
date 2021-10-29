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
    print('------------------------------------------------------------')
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
    print('1) Planning independent paths for root node ')
    # first plan the independent paths for every AC
    for ac in aircraft_list:
        # prevent arrived aircraft to still influence the planning, only AC on the map are accounted for
        if ac.status == "taxiing":
            # plan path from current AC location and timestep, since the AC might have already moved on the grid
            # the start location shouldn't be used per se. Note the difference between rpioritized here: prioritized
            # plans for every AC from spawntime, CBS replans for every AC from the current time onwards
            print('1.1) ACID: ' + str(ac.id))
            # current AC position is start position if it hasn't moved, and from_to[0] if it has
            print('1.1) ac.from_to[0]: ' + str(ac.from_to[0]))
            print('1.1) ac.start: ' + str(ac.start))
            print('1.1) ac_path_to_goal original: ' + str(ac.path_to_goal))
            # current position is start node if AC hasn't moved yet
            curr_pos = ac.from_to[0] if len(ac.path_to_goal) > 0 else ac.start
            print('1.1) curr_pos: ' + str(curr_pos))
            # no constraints are given to astar for the root node. ac.id is a parameter because it is used
            # to construct the constraint tabel since this has a different format for CBS (see single_agent_planner)
            success, path, expanded_nodes = astar(nodes_dict, curr_pos, ac.goal, heuristics, [], t, dt, ac.id, True, ac)
            # update KPI
            num_of_expanded += expanded_nodes
            if success:
                # update the remain path to goal for this AC
                ac.path_to_goal = path[1:]
                print('1.1) path: ' + str(path))
                print('1.1) ac.path_to_goal: ' + str(ac.path_to_goal))
                # next node in the determined path
                next_node = path[1][0]
                # update the AC from_to list. this list is used to determine the current position of the aircraft when
                # astar has to be performed
                ac.from_to = [path[0][0], next_node]
                # update current planned path of AC
                ac.current_path = path.copy()
                # append the constructed path to the root node
                root['paths'].append(path)
                # append ACID of this AC to the acids list
                root['acids'].append(ac.id)
            else:
                # TODO: remove path etc from root?
                ac.status = "deadlocked"
                num_of_deadlocks += 1
                raise BaseException('Deadlock CBS independent paths')

    print('2) Root node paths: ' + str(root['paths']))
    # determine cost of root node paths
    root['cost'] = get_sum_of_cost(root['paths'])
    # detect all the collisions that this solution of paths bring with it
    root['collisions'] = detect_collisions(root['paths'], root['acids'], dt)
    print('3) Root node collisions: ' + str(root['collisions']))
    print('4) Root node cost: ' + str(root['cost']))
    # push this node to the open list since it's not fully expanded. this function return the number of generated nodes
    # which is semi handy but not really used
    num_of_generated = push_node(open_list, root, num_of_generated)

    # if there are still nodes unexplored:
    while len(open_list) > 0:
        # pop node with smallest cost. Function returns the number of expanded nodes so far
        p, num_of_expanded = pop_node(open_list, num_of_expanded)
        print('5) Smallest cost parent node: ' + str(p))
        # if there are no collisions, the solution is optimal
        if not p['collisions']:
            print('no CBS collisions, optimal paths')
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
                    ac1.path_to_goal = acid_path[1:]
                    # update the AC from_to nodes
                    ac1.from_to = [acid_path[0][0], acid_path[1][0]]
                    # compute the KPIs
                    ac1.compute_time_distance()
                    if acid_path[0][1] != t:
                        raise Exception("Something is wrong with the timing of the path planning")
            return num_of_expanded, num_of_deadlocks

        # there are collisions, extract 1 of them
        collision = p['collisions'][0]
        print('6) There are collisions in parent node: ' + str(collision))
        # create 2 constraints out of the collision
        # note that constraints have different format than for prioritized planning due to spawntime not being
        # important for constraint obedience anymore
        constraints = standard_splitting(collision, dt)
        print('6.6) Constraints: ' + str(constraints))
        # loop over both constraints, generating a new node for each one
        for constraint in constraints:
            print('7) looping over constraints, constraint: ' + str(constraint))
            # current constraints from parent node
            p_constraints = p['constraints'] if p['constraints'] is not None else []
            # make a copy of these constraints and append the constraint used in this loop
            # this way, we'll create a child node with just 1 extra constraint
            q_constraints = p_constraints.copy()
            q_constraints.append(constraint)
            print('8) q_constraints: ' + str(q_constraints))
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
            print('9) ACID: ' + str(q_acid))
            # extract the aircraft itself from the global aircraft list
            aircr = None
            for ac2 in aircraft_list:
                if ac2.id == q_acid:
                    aircr = ac2
            print('9.9) aircr.id: ' + str(aircr.id))
            # calculate path for the aircraft with a new constraint
            if aircr is not None:
                print('9.10) aircr.fomr_to[0]: ' + str(aircr.from_to[0]))
                print('9.10) aircr.start: ' + str(aircr.start))
                print('9.11) current time: ' + str(t) + ' AC spawntime: ' + str(aircr.spawntime))
                # current position is different from start node if AC has already moved
                curr_pos = aircr.from_to[0] if len(aircr.path_to_goal) > 0 else aircr.start
                print('10) curr_pos: ' + str(curr_pos))
                print('11) aircr.goal: ' + str(aircr.goal))
                # calculate path with extra constraints
                success, q_path, expanded_nodes = astar(nodes_dict, curr_pos, aircr.goal, heuristics, q_constraints, t,
                                                        dt, q_acid, True, aircr)
                # update KPI
                num_of_expanded += expanded_nodes
                # if a path was found, update the AC instance variables and update Q node
                if success:
                    aircr.path_to_goal = q_path[1:].copy()
                    next_node_id = q_path[1][0]
                    aircr.from_to = [q_path[0][0], next_node_id]
                    # update the current path of the aircraft, used for performance indicators
                    curr_path = aircr.current_path
                    # find index in current_path that corresponds to this timestep. Us e filter with lambda
                    # expression to find the tuple corresponding to the current timestep. Then find the index of
                    # this tuple in the current_path
                    index_curr_timestep = curr_path.index(
                        list(filter(lambda node_t_pair: node_t_pair[1] == t in node_t_pair, curr_path))[0])
                    # now change the current_path of the AC from this step onwards into the newly calculated path
                    curr_path[index_curr_timestep:] = q_path
                    aircr.current_path = curr_path.copy()
                    print('12) aircr.from_to updated: ' + str(aircr.from_to))
                    print('12) Path was found for aircraft with extra constraint')
                    print('13) Found path for AC ' + str(q_acid) + ' : ' + str(q_path))
                    # find paths index of this aircraft via the ACIDS list
                    index = q['acids'].index(q_acid)
                    print('index: ' + str(index))
                    # replace the path corresponding to this AC
                    q['paths'][index] = q_path
                    print('14) q[paths] after update: ' + str(q['paths']))
                    # TODO: goes wrong here. Too much paths in q['paths']
                    q['collisions'] = detect_collisions(q['paths'], q['acids'], dt)
                    q['cost'] = get_sum_of_cost(q['paths'])
                    num_of_generated = push_node(open_list, q, num_of_generated)
                    aircr.compute_time_distance()
                else:
                    # unable to find path for this AC. It shouldn't be removed tho, but this node should get a really high cost
                    # added 24 oct
                    print('No path found for ACID: ' + str(q_acid))
                    # put a huge cost on this move
                    q['cost'] = 1000000
                    '''
                    index_q_acid = q['acids'].index(q_acid)
                    q['acids'].remove(q_acid)
                    # TODO: index out of range here
                    q['paths'].remove(q['paths'][index_q_acid])
                    aircraft_list.remove(aircr)
                    num_of_deadlocks += 1
                    '''
            else:
                raise Exception("CBS: no aircraft found in node list with ID: " + str(q_acid))

    return None


def detect_collision(path1, path2, dt):
    """
    Returns first collsion that appears between the 2 paths.
    - vertex collision has the form [[node], timestep]
    - edge collision has the form [[node1, node2], timestep]
    """
    # TODO: detect artificial collision here when 2 AC are at the departing runway at the same time
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
        # TODO: this collision format isn't good. It denotes an edge collision instead of 2 vertex collisions
        elif curr_loc_valid and (loc1 == 1.0 or loc2 == 1.0) and (loc1 == 2.0 or loc2 == 2.0) and loc1 != loc2:
            #print('RUNWAY CONSTRAINT DETECT_COLLISION')
            return [[loc1, loc2], timestep]
        # edge collision
        # TODO: find a pretty way to do this
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
    print('Detected collisions: ' + str(collisions))
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
    print('Standard splitting collision: ' + str(collision))
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
    # TODO: same departure runway constraint
    # vertex constraint
    elif len(collision['loc']) == 1:
        constraint1 = {'acid': collision['acid1'], 'loc': loc, 'timestep': timestep}
        constraint2 = {'acid': collision['acid2'], 'loc': loc, 'timestep': timestep}
        constraints.append(constraint1)
        constraints.append(constraint2)
    print('Standard splitting constraints: ' + str(constraints))
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
    print("Generate node {}".format(num_of_generated))
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
