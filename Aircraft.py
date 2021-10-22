from single_agent_planner import simple_single_agent_astar, astar
# from cbs import detect_collision
from cbs import detect_collisions
from cbs import standard_splitting
import math
import random

class Aircraft(object):
    """Aircraft class, should be used in the creation of new aircraft."""

    def __init__(self, flight_id, a_d, start_node, goal_node, spawn_time, nodes_dict):
        """
        Initalisation of aircraft object.
        INPUT:
            - flight_id: [int] unique id for this aircraft
            - a_d: [str] "a" for arrival flight and "d" for departure flight
            - start_node: node_id of start node
            - goal_node: node_id of goal node
            - spawn_time: spawn_time of a/c 
            - nodes_dict: copy of the nodes_dict
        """

        # Fixed parameters
        self.speed = 1  # how much a/c moves per unit of t
        self.id = flight_id  # flight_id
        self.type = a_d  # arrival or departure (A/D)
        self.spawntime = spawn_time  # spawntime
        self.start = start_node  # start_node_id
        self.goal = goal_node  # goal_node_id
        self.nodes_dict = nodes_dict  # keep copy of nodes dict

        # Route related
        self.status = None
        self.path_to_goal = []  # planned path left from current location
        self.from_to = [0, 0]

        # State related
        self.heading = 0
        self.position = (0, 0)  # xy position on map

        # performance indicators
        self.current_path = []      # added to ensure compute_time_distance works for CBS and individual as well
        self.travel_time = len(self.path_to_goal)    # total travel time
        self.visited_nodes = []     # different nodes visited by A/C. To determine path_length
        self.path_length = len(self.visited_nodes)    # total spatial distance of calculated path
        self.time_length_ratio = 0  # ratio between travel time and path length. Indicates waiting time

        # individual planning
        self.observation_space = dict()
        # TODO: find suitable value
        self.budget = 100
        self.constraints = []   # constraints regarding this AC
        self.planned_t = False  # AC already has been planned for timestep t

    def get_heading(self, xy_start, xy_next):
        """
        Determines heading of an aircraft based on a start and end xy position.
        INPUT:
            - xy_start = tuple with (x,y) position of start node
            - xy_next = typle with (x,y) position of next node
        RETURNS:
            - heading = heading of aircraft in degrees
        """

        if xy_start[0] == xy_next[0]:  # moving up or down
            if xy_start[1] > xy_next[1]:  # moving down
                heading = 180
            elif xy_start[1] < xy_next[1]:  # moving up
                heading = 0
            else:
                heading = self.heading

        elif xy_start[1] == xy_next[1]:  # moving right or left
            if xy_start[0] > xy_next[0]:  # moving left
                heading = 90
            elif xy_start[0] < xy_next[0]:  # moving right
                heading = 270
            else:
                heading = self.heading
        else:
            raise Exception("Invalid movement")

        self.heading = heading

    def move(self, dt, t, constraints):
        """
        Moves an aircraft between from_node and to_node and checks if to_node or goal is reached.
        INPUT:
            - dt = time step delta
            - t = current time
        """

        # Determine nodes between which the ac is moving
        from_node = self.from_to[0]
        # division by 0 debugging
        #print('from_node: ' + str(from_node))
        to_node = self.from_to[1]
        # division by 0 debugging
        #print('to_node: ' + str(to_node))
        xy_from = self.nodes_dict[from_node]["xy_pos"]  # xy position of from node
        xy_to = self.nodes_dict[to_node]["xy_pos"]  # xy position of to node
        distance_to_move = self.speed * dt  # distance to move in this timestep

        # Update position with rounded values
        x = xy_to[0] - xy_from[0]
        y = xy_to[1] - xy_from[1]
        # division by 0 debugging
        #print('x: ' + str(x) + ', y: ' + str(y))
        x_normalized = 0 if (x == 0 and y == 0) else x / math.sqrt(x ** 2 + y ** 2)
        y_normalized = 0 if (x == 0 and y == 0) else y / math.sqrt(x ** 2 + y ** 2)
        posx = round(self.position[0] + x_normalized * distance_to_move, 2)  # round to prevent errors
        posy = round(self.position[1] + y_normalized * distance_to_move, 2)  # round to prevent errors
        self.position = (posx, posy)
        self.get_heading(xy_from, xy_to)

        # Check if goal is reached or if to_node is reached
        if self.position == xy_to and self.path_to_goal[0][1] == t + dt:  # If with this move its current to node is reached
            if self.position == self.nodes_dict[self.goal]["xy_pos"]:  # if the final goal is reached
                self.status = "arrived"
                for constraint in constraints:
                    # remove constraints which where constructed by this AC
                    if constraint['acid'] == self.id:
                        constraints.remove(constraint)

            else:  # current to_node is reached, update the remaining path
                remaining_path = self.path_to_goal
                self.path_to_goal = remaining_path[1:]

                new_from_id = self.from_to[1]  # new from node
                new_next_id = self.path_to_goal[0][0]  # new to node

                if new_from_id != self.from_to[0]:
                    self.last_node = self.from_to[0]

                self.from_to = [new_from_id, new_next_id]  # update new from and to node

    def plan_independent(self, nodes_dict, edges_dict, heuristics, t):
        """
        Plans a path for taxiing aircraft assuming that it knows the entire layout.
        Other traffic is not taken into account.
        INPUT:
            - nodes_dict: copy of the nodes_dict
            - edges_dict: edges_dict with current edge weights
        """

        if self.status == "taxiing":
            start_node = self.start  # node from which planning should be done
            goal_node = self.goal  # node to which planning should be done

            success, path, exp_nodes = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, t)
            if success:
                self.path_to_goal = path[1:]
                next_node_id = self.path_to_goal[0][0]  # next node is first node in path_to_goal
                self.from_to = [path[0][0], next_node_id]
                print("Path AC", self.id, ":", path)
                self.current_path = path
                # determine length and time of travelled path + their ratio
                self.compute_time_distance()
                print("travel time AC", self.id, ":", self.travel_time)
                print("travel distance AC", self.id, ":", self.path_length)
                print("travel time/distance ratio AC", self.id, ":", self.time_length_ratio)

            else:
                raise Exception("No solution found for", self.id)

            # Check the path
            if path[0][1] != t:
                raise Exception("Something is wrong with the timing of the path planning")
        return exp_nodes

    def plan_prioritized(self, nodes_dict, edges_dict, heuristics, constraints, dt, t):
        # TODO: remove AC in deadlock from map
        """
        Plans path for taxiing aircraft where constraints are constructed on the go in terms of priority
        Args:
            constraints:
            nodes_dict:
            edges_dict:
            heuristics:
            t:

        Returns:
            expanded nodes
            updated constraints
            number of deadlocks occurred
        """

        if self.status == "taxiing":
            start_node = self.start  # node from which planning should be done
            goal_node = self.goal  # node to which planning should be done

            # the parameter 0 denotes that the aircraft Id is not important here. It won't be used in the A star algorithm.
            # The False parameter indicated that the A star algorithm has to construct the constraint table using
            # the prioritized constraint format
            success, path, expanded_nodes = astar(nodes_dict, start_node, goal_node, heuristics, constraints, t, dt, 0,
                                                  False, self)

            if success:
                if path[0][1] != t:
                    raise Exception("Something is wrong with the timing of the path planning")
                self.path_to_goal = path[1:]
                next_node_id = self.path_to_goal[0][0]  # next node is first node in path_to_goal
                self.from_to = [path[0][0], next_node_id]
                self.current_path = path.copy()
                # determine length and time of travelled path + their ratio
                self.compute_time_distance()
                # print("travel time AC", self.id, ":", self.travel_time)
                # print("travel distance AC", self.id, ":", self.path_length)
                # print("travel time/distance ratio AC", self.id, ":", self.time_length_ratio)

                # now add constraints to other agents
                # constraints are not agent-specific, but spawn time specific
                # every agent that spawns after or at the time specified in the constraint, will have
                # to obey to this constraint
                for node_time_pair in path: # path is list of (node_id, timestep) tuples
                    node = node_time_pair[0]    # find node_id
                    timestep = node_time_pair[1]
                    # vertex constraint
                    # added acid (aircraft ID) as a field. This way, constraints constructed by a certain aircraft can be removed
                    # once this aircraft has reached its goal
                    constraints.append({'spawntime': self.spawntime, 'loc': [node], 'timestep': timestep, 'acid': self.id})
                    # edge constraint: only if aircraft has moved already
                    if not timestep <= self.spawntime + dt:
                        # find previous node in AC path. The 2*timestep is to convert half timesteps to indices
                        # print('timestep: ' + str(timestep))
                        previous_node = path[int((1/dt) * timestep - (1/dt) * self.spawntime - 1)][0]     # TODO: double check this, gave errors
                        # added acid (aircraft ID) as a field. This way, constraints constructed by a certain aircraft can be removed
                        # once this aircraft has reached its goal
                        constraints.append({'spawntime': self.spawntime, 'loc': [node, previous_node], 'timestep': timestep, 'acid': self.id})
                    # same departure time constraint
                    # TODO: check this
                    if node == 2 or node == 1:
                        constraints.append({'spawntime': self.spawntime, 'loc': [1], 'timestep': timestep, 'acid': self.id})
                        constraints.append({'spawntime': self.spawntime, 'loc': [2], 'timestep': timestep, 'acid': self.id})

                return expanded_nodes, constraints, 0, None

            else:
                print('No solution found for ' + str(self.id))
                return expanded_nodes, constraints, 1, self

    def compute_time_distance(self):
        """
        computes the performance indicators for travel time, travel distance and their ratio
        This function is called when the A/C already has a planned path. It then updates instance
        variables representing these performance indicators
        """
        # travel time performance indicator
        start_time = self.spawntime
        arrival_time = self.current_path[-1][1]
        self.travel_time = float(arrival_time - start_time)

        # travel distance performance indicator
        # check whether the aircraft waits at a certain node. If not, add this node to the list of different
        # visited nodes
        for position in self.current_path:
            node = position[0]
            if not node in self.visited_nodes:
                self.visited_nodes.append(node)
        # calculate travelled distance
        self.path_length = len(self.current_path)

        # travel time/travel distance performance indicator
        self.time_length_ratio = round(self.travel_time / float(self.path_length), 5)

    # added for individual planning
    def scan(self):
        """
        scans the observation space and extracts a list of AC objects which the current AC can see
        Returns: observed_ac, a list containing AC objects in the observation space. Will be empty if there are none

        """
        observed_ac = []
        for node_id in self.observation_space:
            ac_at_node = self.observation_space[node_id]
            if ac_at_node is not None:
                observed_ac.append(ac_at_node)
        return observed_ac

    # added for individual planning
    def perform_ind_planning(self, observed_ac, t, dt, heuristics):
        """
        The main individual planning function. The main loop runs over the AC in observed_ac list. For every AC (let's call it AC2)
        in this list, do the following:
        1) extract path of AC2 for the following 2 timesteps. 2 is chosen due to the amount of nodes an AC can look ahead
        2) detect collisions between currently planned path an AC2 path, a CBS function is used to recycle code. If a
        situation occurs where either AC or AC2 is at an intermediate node between 2 intersections and the other AC is
        at the intersection and heading right into a front-end collision, the AC at the intersection node needs to move
        and no negotiation is needed
        3) create constraints out of this collision
        4) let AC and AC2 plan path independently, given the extra constraint and determine the extra cost (length of
        path) for each newly created path. The extra cost will be expressed as a percentage of the original path length
        5) AC and AC2 want to bid the same percentage of their budget as the cost for rerouting their paths. So a 30%
        increase in path length indicates that the agent wants to bid 30% of its remaining budget. Calculate the bids
        of each of the 2 AC
        6) highest bid wins if the bids are equal, the AC with highest extra cost wins. The winning AC however only pays
        the bid of the other agent + 10%, with a max price of its own bid of course. The budget instance variable of
        winning AC gets updated
        7) the winning AC is thus able to keep its path, however, the losing AC plans it's path according to a new
        constraint. This constraint is given to the other aircraft in the observed_ac list to prevent them looking
        for solutions which would lead them into a collision with this AC
        Args:
            observed_ac:

        Returns:

        """
        # TODO: add corresponding constraint to losing aircraft
        # performance indicators
        expanded_nodes = 0
        deadlocks = 0

        # if there's an aircraft in a deadlock position, it will be added to this list
        deadlock_ac = []

        # if this AC was already treated and it lost to another AC
        if self.planned_t:
            return 0, 0, deadlock_ac
        # if the AC doesn't have a path yet, calculate it independently
        curr_pos_self = self.from_to[0] if self.from_to[0] != 0 else self.start
        if len(self.path_to_goal) == 0:
            success, path, exp_nodes = astar(self.nodes_dict, curr_pos_self, self.goal, heuristics, self.constraints,
                                                 t, dt, self.id, True, self)
            if success:
                self.path_to_goal = path[1:]
                self.from_to = [path[0][0], path[1][0]]
                # performance indicator
                expanded_nodes += exp_nodes
                # update current path, used for travelling time and distance performance indicator
                self.current_path = path.copy()
                self.compute_time_distance()
            else:
                # performance indictaor
                deadlocks += 1
                deadlock_ac.append(self)
                print('no base path found for ac1')

        for ac2 in observed_ac:
            if ac2.id != self.id and not self.planned_t:
                # denotes whether one of the 2 current AC is deadlocked
                deadlock_occurred = False
                curr_pos_ac2 = ac2.from_to[0] if ac2.from_to[0] != 0 else ac2.start
                # path for next 2 timesteps. Included current timestep and position as wel to detect edge constraints
                path2 = [(curr_pos_ac2, t)] + ac2.path_to_goal[:2]
                # if AC2 doesn't have a path yet, plan it
                if len(path2) == 1:
                    success, path, exp_nodes = astar(self.nodes_dict, curr_pos_ac2, ac2.goal, heuristics, ac2.constraints,
                                                     t, dt, ac2.id, True, ac2)
                    if success:
                        ac2.path_to_goal = path[1:]
                        ac2.from_to = [path[0][0], path[1][0]]
                        # update current path, used for travelling time and distance performance indicator
                        ac2.current_path = path.copy()
                        # performance indicator
                        expanded_nodes += exp_nodes
                        path2 = [(curr_pos_ac2, t)] + ac2.path_to_goal[:2]
                    else:
                        # performance indicator
                        deadlocks += 1
                        deadlock_ac.append(ac2)
                        deadlock_occurred = True
                        print('no base path found for ac2')
                # included current position and node as well to be able to detect edge collisions
                path = [(curr_pos_self, t)] + self.path_to_goal[:2]
                # detect collisions, use CBS function for this since we will use same constraint format as CBS
                collisions = detect_collisions([path, path2], [self.id, ac2.id], dt)
                if len(collisions) > 0 and not ac2.planned_t:
                    # extract first collision
                    collision = collisions[0]
                    constraints = []

                    # indicates situation with right of way
                    right_of_way = False
                    # first check whether one of the Ac is at an intersection, heading into the other AC which is at a
                    # node between intersections. This is a right of way situation, where the AC at the intersection
                    # HAS to move. No bidding is done in this case
                    if len(self.nodes_dict[curr_pos_self]["neighbors"]) == 2 and len(self.nodes_dict[curr_pos_ac2]["neighbors"]) > 2:
                        # self is located between intersections, AC2 is located at intersection and heading into it
                        # impose constraints on AC2 such that it HAS to move away from th intersection
                        constraints.append({'acid': ac2.id, 'loc': [curr_pos_ac2], 'timestep': t + dt})
                        constraints.append({'acid': ac2.id, 'loc': [curr_pos_self], 'timestep': t + dt})
                        # indicate that this is a right of way situation
                        right_of_way = True
                        # AC2 has to move, so we can consider it planned for this timestep
                        ac2.planned_t = True
                    elif len(self.nodes_dict[curr_pos_self]["neighbors"]) > 2 and len(self.nodes_dict[curr_pos_ac2]["neighbors"]) == 2:
                        # ac2 is located between intersections and self is located at intersection and heading into ac2
                        # impose constraints on self such that self HAS to move away from intersection
                        constraints.append({'acid': self.id, 'loc': [curr_pos_ac2], 'timestep': t + dt})
                        constraints.append({'acid': self.id, 'loc': [curr_pos_self], 'timestep': t + dt})
                        # indicate that this is a right of way situation
                        right_of_way = True
                        # self has to move, so we can consider it planned for this timestep
                        self.planned_t = True

                    # if no right of way situation occurred
                    else:
                        constraints = standard_splitting(collision, dt)
                        '''
                        # we may have a regular collision. Create constraints
                        loc = collision[0]
                        timestep = collision[1]
                        # edge collision
                        if len(loc) > 1:
                            constraints.append({'acid': self.id, 'loc': [loc[0], loc[1]], 'timestep': timestep + dt})
                            constraints.append({'acid': ac2.id, 'loc': [loc[1], loc[0]], 'timestep': timestep + dt})
                        # vertex collision
                        elif len(loc) == 1:
                            constraints.append({'acid': self.id, 'loc': loc, 'timestep': timestep})
                            constraints.append({'acid': ac2.id, 'loc': loc, 'timestep': timestep})
                        '''
                    # now, plan the paths independently for both AC with the newly added constraints
                    success, path_self, exp_nodes = astar(self.nodes_dict, curr_pos_self, self.goal, heuristics,
                                                          constraints + self.constraints, t, dt, self.id, True, self)

                    if not success:
                        # if self is in a deadlock, the function should immediately return. It cannot plan any further
                        deadlocks += 1      # performance indicator
                        expanded_nodes += exp_nodes     # performance indicator
                        deadlock_ac.append(self)
                        deadlock_occurred = True
                        print('!!! DEADLOCK FOR SELF !!!')
                        return expanded_nodes, deadlocks, deadlock_ac
                    else:
                        # performance indicator
                        expanded_nodes += exp_nodes

                    success, path_ac2, exp_nodes = astar(self.nodes_dict, curr_pos_ac2, ac2.goal, heuristics,
                                                         constraints + ac2.constraints, t, dt, ac2.id, True, ac2)
                    if not success:
                        # if ac2 is in a deadlock, the function should not immediately return because self can still
                        # plan with the other aircraft in its observation space. However, AC2 will have to be removed
                        # from the aircraft_lst, that's why it's added to deadlock_ac. If AC2 is in a deadlock, we
                        # just use path_self for self path instead of path. This may be changed later on.
                        # performance indicator
                        deadlocks += 1
                        expanded_nodes += exp_nodes  # performance indicator
                        # we state that self should be planned if AC2 is in a deadlock
                        self.planned_t = True
                        deadlock_occurred = True
                        deadlock_ac.append(ac2)
                        print('!!! DEADLOCK FOR AC2 !!!')
                    else:
                        # performance indicator
                        expanded_nodes += exp_nodes

                    # bidding only happens if we're not in a right of way situation
                    if not right_of_way and not deadlock_occurred:
                        # determine cost for each of the AC involved, calculated as fraction increase w.r.t. old path
                        # the -1 in numerator is because path_self and path_ac2 start from current position whereas
                        # path_to_goal starts from next node
                        cost_self = ((len(path_self) - 1 - len(self.path_to_goal))/len(self.path_to_goal))
                        cost_ac2 = ((len(path_ac2) - 1 - len(ac2.path_to_goal))/len(ac2.path_to_goal))

                        # calculate corresponding bids
                        bid_self = cost_self * self.budget
                        bid_ac2 = cost_ac2 * ac2.budget
                        # TODO: this part has a lot of duplicated code, fix this
                        # next position in the path of the losing aircraft
                        next_pos_losing = None
                        # if current AC wins negotiation or same bids and self has largest cost
                        if bid_self > bid_ac2 or (bid_self == bid_ac2 and cost_self > cost_ac2):
                            self.budget = self.budget - 1.1*bid_ac2 if 1.1*bid_ac2 < bid_self else self.budget - bid_self
                            # indicate that the planning of AC2 is done for this timestep
                            ac2.planned_t = True

                        # if AC2 wins negotiation or same bids and AC2 has largest cost
                        elif bid_self < bid_ac2 or (bid_self == bid_ac2 and cost_self < cost_ac2):
                            ac2.budget = ac2.budget - 1.1*bid_self if 1.1*bid_self < bid_ac2 else ac2.budget - bid_ac2
                            # indicate that the planning of self is done for this timestep
                            self.planned_t = True

                        # if everything is equal, determine arbitrary
                        elif bid_self == bid_ac2 and cost_self == cost_ac2:
                            if random.random() < 0.5:
                                self.budget = self.budget - 1.1 * bid_ac2 if 1.1 * bid_ac2 < bid_self \
                                    else self.budget - bid_self
                                # indicate that the planning of AC2 is done for this timestep
                                ac2.planned_t = True

                            else:
                                ac2.budget = ac2.budget - 1.1 * bid_self if 1.1 * bid_self < bid_ac2 \
                                    else ac2.budget - bid_ac2
                                # indicate that the planning of self is done for this timestep
                                self.planned_t = True
                    # check who won the bidding war, or who has to give priority
                    if self.planned_t:
                        # if ac2 wins biddding war, self has to adjust path
                        self.path_to_goal = path_self[1:]
                        next_pos_losing = path_self[1][0]
                        self.from_to = [path_self[0][0], next_pos_losing]
                        # update the current path of the AC. Used for determining performance indicators
                        curr_path = self.current_path
                        # if AC already has a path planned
                        if len(curr_path) > 0:
                            # find index in current_path that corresponds to this timestep. Us e filter with lambda
                            # expression to find the tuple corresponding to the current timestep. Then find the index of
                            # this tuple in the current_path
                            index_curr_timestep = curr_path.index(
                                list(filter(lambda node_t_pair: node_t_pair[1] == t in node_t_pair, curr_path))[0])
                            # now change the current_path of the AC from this step onwards into the newly calculated path
                            curr_path[index_curr_timestep:] = path_self
                            self.current_path = curr_path.copy()
                        else:
                            self.current_path = path_self.copy()
                        # compute travelling time and distance performance indiactors
                        self.compute_time_distance()
                        # TODO: if there are still collisions, this could be due to a inheritance problem here
                        # add the constraint to the ACs constraints list
                        constraints_self = self.constraints
                        for constrnt in constraints:
                            if constrnt['acid'] == self.id:
                                constraints_self.append(constrnt)
                        self.constraints = constraints_self.copy()

                    elif ac2.planned_t:
                        # if self wins biddding war, ac2 has to adjust path
                        ac2.path_to_goal = path_ac2[1:]
                        next_pos_losing = path_ac2[1][0]
                        ac2.from_to = [path_ac2[0][0], next_pos_losing]
                        # update the current path of the AC. Used for determining performance indicators
                        curr_path = ac2.current_path
                        # if AC already has a path planned
                        if len(curr_path) > 0:
                            # find index in current_path that corresponds to this timestep. Us e filter with lambda
                            # expression to find the tuple corresponding to the current timestep. Then find the index of
                            # this tuple in the current_path
                            index_curr_timestep = curr_path.index(
                                list(filter(lambda node_t_pair: node_t_pair[1] == t in node_t_pair, curr_path))[0])
                            # now change the current_path of the AC from this step onwards into the newly calculated path
                            curr_path[index_curr_timestep:] = path_ac2
                            ac2.current_path = curr_path.copy()
                        else:
                            ac2.current_path = path_ac2.copy()
                        # compute travelling time and distance performance indicators
                        ac2.compute_time_distance()
                        # TODO: if there are still collisions, this could be due to a inheritance problem here
                        # add the constraint to the ACs constraints list
                        constraints_ac2 = ac2.constraints
                        for constrnt in constraints:
                            if constrnt['acid'] == ac2.id:
                                constraints_ac2.append(constrnt)
                        ac2.constraints = constraints_ac2.copy()

                    else:
                        raise BaseException('no AC was planned')

                    # the other AC in the observation space cannot be at the losing aircraft position for the next
                    # timestep since its path is already planned
                    for ac in observed_ac:
                        if not ac.planned_t:
                            ac.constraints.append({'acid': ac.id, 'loc': [next_pos_losing], 'timestep': t + dt})

                elif len(collisions) > 0 and ac2.planned_t:
                    # if there is a collision between self and an already planned AC, self will need to add a constraint
                    # to itself for a path construction that avoids crashing into ac2. However, self should not be
                    # considered as "planned" because it will still be able to start bidding wars with other AC

                    # add constraint which prohibits self from crashing into ac2 at next timestep
                    collision = collisions[0]
                    constraints = standard_splitting(collision, dt)
                    # TODO: check inheritance
                    self.constraints.append(constraints[0].copy())

                    # find alternative path
                    success, updated_path, exp_nodes = astar(self.nodes_dict, curr_pos_self, self.goal, heuristics,
                                                          self.constraints, t, dt, self.id, True, self)
                    if not success:
                        # if self is in a deadlock, the function should immediately return. It cannot plan any further
                        deadlocks += 1  # performance indicator
                        expanded_nodes += exp_nodes  # performance indicator
                        deadlock_ac.append(self)
                        deadlock_occurred = True
                        print('!!! DEADLOCK FOR SELF !!!')
                        return expanded_nodes, deadlocks, deadlock_ac

                    self.path_to_goal = updated_path[1:]
                    self.from_to = [updated_path[0][0], updated_path[1][0]]
                    # performance indicator
                    expanded_nodes += exp_nodes

                    # we should now check whether the newly found path will coincide with the paths of the AC already
                    # looped over in observed_ac. To accomplish this, loop over all the ac which have been looped over
                    # so far in observed_ac and check whether there is a collision somewhere. If there is, the function
                    # perform_ind_planning should be repeated with the newly added constraint to self. This process
                    # should be repeated until there are no more colliding paths between self and the already looped
                    # over AC

                    index_ac2 = observed_ac.index(ac2)
                    for i in range(index_ac2):
                        # extract corresponding ac from observed_ac
                        ac22 = observed_ac[i]
                        if ac22.id != self.id:
                            # determine ac22 current position and path for next 2 timesteps
                            curr_pos_ac22 = ac22.from_to[0] if ac22.from_to[0] != 0 else ac22.start
                            path_ac22 = [(curr_pos_ac22, t)] + ac22.path_to_goal[:2]
                            # ac22 should have a path already; so no need to check the length of path22
                            updated_path_short = [(curr_pos_self, t)] + self.path_to_goal[:2]
                            # detect if there are collisions
                            new_collisions = detect_collisions([updated_path_short, path_ac22], [self.id, ac22.id], dt)
                            # if there are collisions, replan
                            if len(new_collisions) > 0:
                                self.perform_ind_planning(observed_ac[:index_ac2+1], t, dt, heuristics)

                    # the program gets to this line if there were no collisions with the other ac
                    # if the loop is finished without collisions the current path and necessary instance variables
                    # can be updated
                    curr_path = self.current_path
                    # if AC already has a path planned
                    if len(curr_path) > 0:
                        # find index in current_path that corresponds to this timestep. Us e filter with lambda
                        # expression to find the tuple corresponding to the current timestep. Then find the index of
                        # this tuple in the current_path
                        index_curr_timestep = curr_path.index(
                            list(filter(lambda node_t_pair: node_t_pair[1] == t in node_t_pair, curr_path))[0])
                        # now change the current_path of the AC from this step onwards into the newly calculated path
                        curr_path[index_curr_timestep:] = updated_path
                        self.current_path = curr_path.copy()
                    else:
                        self.current_path = updated_path.copy()
        return expanded_nodes, deadlocks, deadlock_ac

