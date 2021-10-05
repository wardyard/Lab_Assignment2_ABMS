"""
Implement CBS here
"""
import time
import heapq
from single_agent_planner import astar

# TODO: remove constraints of AC which have arrived


def run_CBS(aircraft_list, nodes_dict, edges_dict, heuristics, constraints, dt, t):

    deadlocks = 0
    expanded_nodes = 0  # KPI
    start = time.perf_counter_ns()  # KPI

    for ac in aircraft_list:
        if ac.spawntime == t:   # only is a new AC has spawned, CBS will be ran again
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]
            # TODO: expanded nodes and deadlocks returns
            cbs(aircraft_list, nodes_dict, edges_dict, heuristics, constraints, dt, t)
            #deadlocks += deadlocks
            #expanded_nodes += exp_nodes
    stop = time.perf_counter_ns()
    time_delta = stop - start  # in nanoseconds

    return time_delta, expanded_nodes, deadlocks


def cbs(aircraft_list, nodes_dict, edges_dict, heuristics, constraints, dt, t):

    open_list = []

    # generate root node with no constraints. Paths of root node are planned independently
    root = {
        'cost': 0,
        'constraints': [],
        'paths': [],
        'collisions': [],
        'aircrafts': []
    }
    for ac in aircraft_list:
        if ac.status == "taxiing":
            # plan path from current AC location and timestep, since the AC might have already moved on the grid
            # the start location shouldn't be used. Note the difference between rpioritized here: prioritized plans
            # for every AC from spawntime, CBS replans for every AC from the current time

            # current AC position is start position if it hasn't moved, and from_to[0] if it has
            curr_pos = ac.from_to[0] if ac.from_to[0] != 0 else ac.start
            # TODO: I think the paths are created from the current timestep onwards => check this
            succes, path, deadlocks = astar(nodes_dict, curr_pos, ac.goal, heuristics, constraints, t, dt, ac.id, True)
            if path is None:
                raise BaseException('No solution for CBS root node')
            root['paths'].append(path)
            # print('path: ' + str(path))
            root['aircrafts'].append(ac)

    root['cost'] = get_sum_of_cost(root['paths'])
    root['collisions'] = detect_collisions(root['paths'], root['aircrafts'], dt)
    push_node(open_list, root)

    while len(open_list) > 0:
        # pop node with smallest cost
        p = pop_node(open_list)

        # if there are no collisions, the solution is optimal
        if not p['collisions']:
            print('no CBS collisions, optimal paths')
            # add paths to the Aircraft which are currently in the map
            for i in range(len(p['aircrafts'])):
                ac = p['aircrafts'][i]
                path = p['paths'][i]
                ac.path_to_goal = path[1:]
                next_node_id = ac.path_to_goal[0][0]
                ac.from_to = [path[0][0], next_node_id]
            return

        # there are collisions, extract 1 of them
        collision = p['collisions'][0]
        # create 2 constraints out of the collision
        # not that constraints have different format than for prioritized planning due to spawntime not being
        # important for constraint obedience anymore
        constraints = standard_splitting(collision)

        # loop over both constraints, generating a new node for each one
        for constraint in constraints:
            # current constraints from parent node
            p_constraints = p['constraints'] if p['constraints'] is not None else []
            # make a copy of these constraints and append the constrained used in this loop
            # this way, we'll create a child node with just 1 extra constraint
            q_constraints = p_constraints.copy()
            q_constraints.append(constraint)
            # create child node. The paths will be updated later on
            q = {
                'cost': 0,
                'constraints': q_constraints,
                'paths': p['paths'].copy,
                'collisions': [],
                'aircrafts': p['aircrafts']     # TODO: should this be a copy? Don't think so since we want to change instance variables of these AC
            }
            # find out the aircraft ID of the aircraft in the constraint
            q_acid = constraint['acid']
            # extract the aircraft itself from the aircrafts connected to this node
            aircr = None
            for ac in q['aircrafts']:
                if ac.id == q_acid:
                    aircr = ac  # TODO: check inheritance
            # calculate path for the aircraft with a new constraint
            if aircr is not None:
                curr_pos = aircr.from_to[0]
                success, q_path, expanded_nodes = astar(nodes_dict, curr_pos, aircr.goal, heuristics, q_constraints, t, dt, q_acid, True)
                # if a path was found
                if success:
                    # find paths index of this aircraft via the aircrafts list
                    index = q['aircrafts'].index(aircr)
                    print('index: ' + str(index))
                    # replace the path corresponding to this AC
                    q['paths'][index] = q_path
                    q['collisions'] = detect_collisions(q['paths'], q['aircrafts'], dt)
                    push_node(open_list, q)
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
            return [[loc1, loc2], timestep + dt]
    return None


def get_location(path, time):
    # paths has following structure: [(node1, t1), (node2, t2), (node3, t3), ...],
    # we only want the nodes
    if time < 0:
        return path[0][0]
    elif time < len(path):
        for node_time_pair in path:
            if node_time_pair[1] == time:  # if this node_time_pair is for the correct timestep
                return node_time_pair[0]  # return node
    else:
        return path[-1][0]  # wait at the goal location


def detect_collisions(paths, aircrafts, dt):
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
                    # TODO: incorporate ACID as agent IDs
                    ac1 = aircrafts[i]  # first aircraft
                    ac2 = aircrafts[j]
                    collisions.append({'acid1': ac1.id, 'acid2': ac2.id, 'loc': collision[0], 'timestep': collision[1]})
                combs.append((i, j))
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

    loc = collision['loc']
    timestep = collision['timestep']

    # edge constraint
    if len(loc) > 1:
        constraint1 = {'acid': collision['acid1'], 'loc': loc, 'timestep': timestep}
        # reverse positions for second constraint
        constraint2 = {'acid': collision['acid1'], 'loc': [loc[1], loc[0]], 'timestep': timestep}
        constraints.append(constraint1)
        constraints.append(constraint2)

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

def push_node(open_list, node):
    # TODO: number of generated nodes
    heapq.heappush(open_list, (node['cost'], len(node['collisions']), node))
    # print("Generate node {}".format(self.num_of_generated))
    # self.num_of_generated += 1

def pop_node(open_list):
    # TODO: expanded nodes
    _, _, node = heapq.heappop(open_list)
    # print("Expand node {}".format(id))
    # self.num_of_expanded += 1
    return node
