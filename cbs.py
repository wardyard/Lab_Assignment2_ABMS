"""
Implement CBS here
"""
import time
import heapq
from single_agent_planner import astar

# TODO: remove constraints of AC which have arrived


def run_CBS(aircraft_list, nodes_dict, edges_dict, heuristics, dt, t):
    deadlocks = 0
    expanded_nodes = 0  # KPI
    start = time.perf_counter_ns()  # KPI
    for ac in aircraft_list:
        if ac.spawntime == t:   # only is a new AC has spawned, CBS will be ran again
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]
            # ac.from_to = [0, 0]
            # TODO: expanded nodes and deadlocks returns
            exp_nodes = cbs(aircraft_list, nodes_dict, edges_dict, heuristics, dt, t)
            #deadlocks += deadlocks
            #expanded_nodes += exp_nodes
    stop = time.perf_counter_ns()
    time_delta = stop - start  # in nanosecond

    return time_delta, expanded_nodes, deadlocks


def cbs(aircraft_list, nodes_dict, edges_dict, heuristics, dt, t):
    print('------------------------------------------------------------')
    open_list = []
    num_of_generated = 0
    num_of_expanded = 0
    # generate root node with no constraints. Paths of root node are planned independently
    '''
    root = {
        'cost': 0,
        'constraints': [],
        'paths': [],
        'collisions': [],
        'aircrafts': []
    }
    '''
    root = {
        'cost': 0,
        'constraints': [],
        'paths': [],
        'collisions': [],
        'acids': []
    }
    print('1) Planning independent paths for root node ')
    for ac in aircraft_list:
        # prevent arrived aircraft to still influence the planning
        if ac.status == "taxiing":
            # plan path from current AC location and timestep, since the AC might have already moved on the grid
            # the start location shouldn't be used. Note the difference between rpioritized here: prioritized plans
            # for every AC from spawntime, CBS replans for every AC from the current time
            print('1.1) ACID: ' + str(ac.id))
            # current AC position is start position if it hasn't moved, and from_to[0] if it has
            print('1.1) ac.from_to[0]: ' + str(ac.from_to[0]))
            print('1.1) ac.start: ' + str(ac.start))
            print('1.1) ac_path_to_goal original: ' + str(ac.path_to_goal))
            curr_pos = ac.from_to[0] if len(ac.path_to_goal) > 0 else ac.start
            print('1.1) curr_pos: ' + str(curr_pos))
            # TODO: I think the paths are created from the current timestep onwards => check this
            success, path, deadlocks = astar(nodes_dict, curr_pos, ac.goal, heuristics, [], t, dt, ac.id, True)
            if success:
                # added for infinite loop
                ac.path_to_goal = path[1:]
                # added for infinite loop
                print('1.1) path: ' + str(path))
                print('1.1) ac.path_to_goal: ' + str(ac.path_to_goal))
                next_node = path[1][0] # TODO: gives index out of range error here

                # added for infinite loop
                ac.from_to = [path[0][0], next_node]
                root['paths'].append(path)
                # print('path: ' + str(path))
                #root['aircrafts'].append(ac)
                # append ACID of this AC to the acids list
                root['acids'].append(ac.id)
            else:
                raise BaseException('No solution for CBS root node')
        '''
        elif ac.status == "arrived":
            # if an Ac has arrived, the corresponding path and ACID need to be removed
            # in addition, all the constraints need to be removed as well 
            acid = ac.id
            index = root['acids'].index(acid)
            root['acids'].pop(index)
            root['paths'].pop(index)
            for constraint in root['constraints']: 
                if constraint['acid'] == acid: 
                    root['constraints'].remove(constraint)
        '''
    print('2) Root node paths: ' + str(root['paths']))
    root['cost'] = get_sum_of_cost(root['paths'])
    root['collisions'] = detect_collisions(root['paths'], root['acids'], dt)
    print('3) Root node collisions: ' + str(root['collisions']))
    print('4) Root node cost: ' + str(root['cost']))
    num_of_generated = push_node(open_list, root, num_of_generated)

    while len(open_list) > 0:
        # pop node with smallest cost
        p, num_of_expanded = pop_node(open_list, num_of_expanded)
        print('5) Smallest cost parent node: ' + str(p))
        # if there are no collisions, the solution is optimal
        if not p['collisions']:
            print('no CBS collisions, optimal paths')
            # loop over all the AC currently in the aircraft list to find aircraft which are used by p node
            for ac1 in aircraft_list:
                acid = ac1.id
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
                    ac1.compute_time_distance(acid_path)
                    if acid_path[0][1] != t:
                        raise Exception("Something is wrong with the timing of the path planning")
                    
            # add paths to the Aircraft which are currently in the map
            # for i in range(len(p['aircrafts'])):  # used to be len(p['aircrafts'])
             #   ac = p['aircrafts'][i]
             #   path = p['paths'][i]
             #   ac.path_to_goal = path[1:]
             #   ac.compute_time_distance(path)
             #   next_node_id = ac.path_to_goal[0][0]
             #   ac.from_to = [path[0][0], next_node_id]
             #   print('optimal path AC ' + str(ac.id) + ': ' + str(ac.path_to_goal))
            
            return num_of_expanded

        # there are collisions, extract 1 of them
        collision = p['collisions'][0]
        print('6) There are collisions in parent node: ' + str(collision))
        # create 2 constraints out of the collision
        # not that constraints have different format than for prioritized planning due to spawntime not being
        # important for constraint obedience anymore
        constraints = standard_splitting(collision)
        print('6.6) Constraints: ' + str(constraints))
        # loop over both constraints, generating a new node for each one
        for constraint in constraints:
            # TODO: constraints get added over and over again to q. Something's wrong here
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
                'acids': p['acids']
            }
            
            #q = {
            #    'cost': 0,
            #    'constraints': q_constraints,
            #    'paths': p['paths'].copy(),
            #    'collisions': [],
            #    'aircrafts': p['aircrafts']     # TODO: should this be a copy? Don't think so since we want to change instance variables of these AC
            #}
            
            # find out the aircraft ID of the aircraft in the constraint
            q_acid = constraint['acid']
            print('9) ACID: ' + str(q_acid))
            # extract the aircraft itself from the global aircraft list
            aircr = None
            for ac2 in aircraft_list:
                if ac2.id == q_acid:
                    aircr = ac2  # TODO: check inheritance
            print('9.9) aircr.id: ' + str(aircr.id))
            # calculate path for the aircraft with a new constraint
            if aircr is not None:
                # current position is different from start node if AC has already moved
                print('9.10) aircr.fomr_to[0]: ' + str(aircr.from_to[0]))
                print('9.10) aircr.start: ' + str(aircr.start))
                print('9.11) current time: ' + str(t) + ' AC spawntime: ' + str(aircr.spawntime))
                curr_pos = aircr.from_to[0] if len(aircr.path_to_goal) > 0 else aircr.start  # TODO: this fucks up I think
                print('10) curr_pos: ' + str(curr_pos))
                print('11) aircr.goal: ' + str(aircr.goal))
                # TODO: the returned path seems incorrect w.r.t. the new constraint
                success, q_path, expanded_nodes = astar(nodes_dict, curr_pos, aircr.goal, heuristics, q_constraints, t,
                                                        dt, q_acid, True)
                # if a path was found, update the AC instance variables and update Q node
                if success:
                    aircr.path_to_goal = q_path[1:]
                    next_node_id = aircr.path_to_goal[0][0]
                    aircr.from_to = [path[0][0], next_node_id]
                    print('12) Path was found for aircraft with extra constraint')
                    print('13) Found path for AC ' + str(q_acid) + ' : ' + str(q_path))
                    # find paths index of this aircraft via the ACIDS list
                    index = q['acids'].index(q_acid)
                    print('index: ' + str(index))
                    # replace the path corresponding to this AC
                    #print('q[paths]: ' + str(q['paths']))
                    #q['paths'].pop(index)
                    q['paths'][index] = q_path # TODO: error here
                    print('14) q[paths] after update: ' + str(q['paths']))
                    q['collisions'] = detect_collisions(q['paths'], q['acids'], dt)
                    q['cost'] = get_sum_of_cost(q['paths'])
                    num_of_generated = push_node(open_list, q, num_of_generated)
                else:
                    print('13) no path found for AC with extra constraint')
            else:
                raise Exception("CBS: no aircraft found in node list with ID: " + str(q_acid))

    return None


def detect_collision(path1, path2, dt):
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    # vertex collision has the form [[node], timestep]
    # edge collision has the form [[node1, node2], timestep]

    # iterate over longest path to ensure goal collisions are noticed as well
    # length = len(path1) if (len(path1) > len(path2)) else len(path2)
    longest_path = path1 if len(path1) > len(path2) else path2
    for node_time_pair in longest_path:
        # extract timestep of node_time_pair in path
        timestep = node_time_pair[1]

        loc1 = get_location(path1, timestep)
        loc2 = get_location(path2, timestep)

        loc1_t_1 = get_location(path1, timestep + dt)
        loc2_t_1 = get_location(path2, timestep + dt)
        # vertex collision
        if loc1 == loc2:
            return [[loc1], timestep]
        # edge collision
        elif loc1 == loc2_t_1 and loc2 == loc1_t_1:
            return [[loc1, loc2], timestep]  # used to be timestep + dt
    return None


def get_location(path, t):
    # paths has following structure: [(node1, t1), (node2, t2), (node3, t3), ...],
    # we only want the nodes
    if t < 0:
        return path[0][0]
    elif t < path[-1][1]:
        for node_time_pair in path:
            if node_time_pair[1] == t:  # if this node_time_pair is for the correct timestep
                return node_time_pair[0]  # return node
    else:
        return path[-1][0]  # wait at the goal location


def detect_collisions(paths, acids, dt):
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    # paths have following structure [(node1, t1), (node2, t2), (node3, t3), ...]
    # we always need to compare to paths to eachother and iterate over all path combinations,
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


def standard_splitting(collision):
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

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
        constraint1 = {'acid': collision['acid1'], 'loc': [loc[0], loc[1]], 'timestep': timestep}
        # reverse positions for second constraint
        constraint2 = {'acid': collision['acid2'], 'loc': [loc[1], loc[0]], 'timestep': timestep}
        constraints.append(constraint1)
        constraints.append(constraint2)

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

    """
    heapq.heappush(open_list, (node['cost'], len(node['collisions']), num_of_generated, node))
    print("Generate node {}".format(num_of_generated))
    num_of_generated += 1
    #print('open list: ' + str(open_list))
    return num_of_generated

def pop_node(open_list, num_of_expanded):
    _, _, id, node = heapq.heappop(open_list)
    #print("Expand node {}".format(id))
    num_of_expanded += 1
    return node, num_of_expanded
