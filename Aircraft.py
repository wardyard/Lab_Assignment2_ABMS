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
        #self.planned_t = False  # AC already has been planned for timestep t
        #self.right_of_way_t = False     # AC had to give right of way. this causes replanning for this AC even if it is
                                        # planned_t
        self.loss_list = []     # added for individual2. List will contain IDs of which the AC has lost in a bidding war
                                # note: only ACIDS get added if this AC was ac2 in the main individual loop

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
        ##print('from_node: ' + str(from_node))
        to_node = self.from_to[1]
        # division by 0 debugging
        ##print('to_node: ' + str(to_node))
        xy_from = self.nodes_dict[from_node]["xy_pos"]  # xy position of from node
        xy_to = self.nodes_dict[to_node]["xy_pos"]  # xy position of to node
        distance_to_move = self.speed * dt  # distance to move in this timestep

        # Update position with rounded values
        x = xy_to[0] - xy_from[0]
        y = xy_to[1] - xy_from[1]
        # division by 0 debugging
        ##print('x: ' + str(x) + ', y: ' + str(y))
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
                #print("Path AC", self.id, ":", path)
                self.current_path = path
                # determine length and time of travelled path + their ratio
                self.compute_time_distance()
                #print("travel time AC", self.id, ":", self.travel_time)
                #print("travel distance AC", self.id, ":", self.path_length)
                #print("travel time/distance ratio AC", self.id, ":", self.time_length_ratio)

            else:
                raise Exception("No solution found for", self.id)

            # Check the path
            if path[0][1] != t:
                raise Exception("Something is wrong with the timing of the path planning")
        return exp_nodes

    def plan_prioritized(self, nodes_dict, edges_dict, heuristics, constraints, dt, t):
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
                # #print("travel time AC", self.id, ":", self.travel_time)
                # #print("travel distance AC", self.id, ":", self.path_length)
                # #print("travel time/distance ratio AC", self.id, ":", self.time_length_ratio)

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
                        # #print('timestep: ' + str(timestep))
                        previous_node = path[int((1/dt) * timestep - (1/dt) * self.spawntime - 1)][0]
                        # added acid (aircraft ID) as a field. This way, constraints constructed by a certain aircraft can be removed
                        # once this aircraft has reached its goal
                        constraints.append({'spawntime': self.spawntime, 'loc': [node, previous_node], 'timestep': timestep, 'acid': self.id})
                    # same departure time constraint
                    if node == 2 or node == 1:
                        constraints.append({'spawntime': self.spawntime, 'loc': [1], 'timestep': timestep, 'acid': self.id})
                        constraints.append({'spawntime': self.spawntime, 'loc': [2], 'timestep': timestep, 'acid': self.id})

                return expanded_nodes, constraints, 0, None

            else:
                #print('No solution found for ' + str(self.id))
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
        # performance indicators
        expanded_nodes = 0
        deadlocks = 0

        # if there's an aircraft in a deadlock position, it will be added to this list
        deadlock_ac = []

        # if this AC was already treated and it lost to another AC
        if self.planned_t and not self.right_of_way_t:
            return 0, 0, deadlock_ac
        self.planned_t = False
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
                #print('no base path found for ac1')

        for ac2 in observed_ac:
            # this boolean is only used for debugging purposes
            # added to prevent entering the if len(collision)>0 and ac2.planned_t loop when ac2 is planned_t due to the
            # previous loo^p
            helper_boolean = False
            if ac2.id != self.id and not self.planned_t:
                # denotes whether one of the 2 current AC is deadlocked
                deadlock_occurred = False
                curr_pos_ac2 = ac2.from_to[0] if ac2.from_to[0] != 0 else ac2.start
                # path for next 2 timesteps. Included current timestep and position as wel to detect edge constraints
                path2 = [(curr_pos_ac2, t)] + ac2.path_to_goal[:2].copy()
                # if AC2 doesn't have a path yet, plan it
                if len(path2) == 1 and ac2.status == "taxiing":
                    success, path, exp_nodes = astar(self.nodes_dict, curr_pos_ac2, ac2.goal, heuristics, ac2.constraints,
                                                     t, dt, ac2.id, True, ac2)
                    if success:
                        ac2.path_to_goal = path[1:]
                        ac2.from_to = [path[0][0], path[1][0]]
                        # update current path, used for travelling time and distance performance indicator
                        ac2.current_path = path.copy()
                        # performance indicator
                        expanded_nodes += exp_nodes
                        path2 = [(curr_pos_ac2, t)] + ac2.path_to_goal[:2].copy()
                    else:
                        # performance indicator
                        deadlocks += 1
                        deadlock_ac.append(ac2)
                        deadlock_occurred = True
                        #print('no base path found for ac2')
                # included current position and node as well to be able to detect edge collisions
                path = [(curr_pos_self, t)] + self.path_to_goal[:2]
                # detect collisions, use CBS function for this since we will use same constraint format as CBS
                collisions = detect_collisions([path, path2], [self.id, ac2.id], dt)
                if len(collisions) > 0 and not ac2.planned_t:
                    helper_boolean = True
                    no_collisions = False
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
                        ac2.right_of_way_t = True
                    elif len(self.nodes_dict[curr_pos_self]["neighbors"]) > 2 and len(self.nodes_dict[curr_pos_ac2]["neighbors"]) == 2:
                        # ac2 is located between intersections and self is located at intersection and heading into ac2
                        # impose constraints on self such that self HAS to move away from intersection
                        constraints.append({'acid': self.id, 'loc': [curr_pos_ac2], 'timestep': t + dt})
                        constraints.append({'acid': self.id, 'loc': [curr_pos_self], 'timestep': t + dt})
                        # indicate that this is a right of way situation
                        right_of_way = True
                        # self has to move, so we can consider it planned for this timestep
                        self.planned_t = True
                        # we need to replan in this situation since the path from self might collide with paths of
                        # already looped AC which did'nt have any collisions

                    # if no right of way situation occurred
                    else:
                        constraints = standard_splitting(collision, dt)


                    # now, plan the paths independently for both AC with the newly added constraints
                    success, path_self, exp_nodes = astar(self.nodes_dict, curr_pos_self, self.goal, heuristics,
                                                          constraints + self.constraints, t, dt, self.id, True, self)

                    if not success:
                        # if self is in a deadlock, the function should immediately return. It cannot plan any further
                        deadlocks += 1      # performance indicator
                        expanded_nodes += exp_nodes     # performance indicator
                        deadlock_ac.append(self)
                        deadlock_occurred = True
                        #print('!!! DEADLOCK FOR SELF !!!')
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
                        #print('!!! DEADLOCK FOR AC2 !!!')
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
                            if random.random() < 1:
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
                        # add the constraint to the ACs constraints list
                        constraints_self = self.constraints
                        for constrnt in constraints:
                            if constrnt['acid'] == self.id:
                                constraints_self.append(constrnt)
                        self.constraints = constraints_self.copy()

                        # replan for the AC which have already passed in observed_ac
                        # this is done unconditionally
                        self.planned_t = False
                        # we need to check whether path_self doesn't collide with other ac paths
                        index_ac2 = observed_ac.index(ac2)
                        for i in range(index_ac2 + 1):
                            # extract corresponding ac from observed_ac
                            ac22 = observed_ac[i]
                            if ac22.id != self.id:
                                # determine ac22 current position and path for next 2 timesteps
                                curr_pos_ac22 = ac22.from_to[0] if ac22.from_to[0] != 0 else ac22.start
                                path_ac22 = [(curr_pos_ac22, t)] + ac22.path_to_goal[:2]
                                # ac22 should have a path already; so no need to check the length of path22
                                path_self_short = [(curr_pos_self, t)] + self.path_to_goal[:2]
                                # detect if there are collisions
                                new_collisions = detect_collisions([path_self_short, path_ac22],
                                                                   [self.id, ac22.id], dt)
                                # if there are collisions, replan
                                if len(new_collisions) > 0:
                                    exp_nodes, deadlcks, deadlock_ac = self.perform_ind_planning(
                                        observed_ac[:index_ac2 + 1], t, dt, heuristics)
                                    expanded_nodes += exp_nodes
                                    if len(deadlock_ac) > 0:
                                        deadlocks += deadlcks
                                        return expanded_nodes, deadlocks, deadlock_ac
                        # update the current path of the AC. Used for determining performance indicators
                        curr_path = self.current_path.copy()
                        # if AC already has a path planned
                        if len(curr_path) > 0:
                            # find index in current_path that corresponds to this timestep. Us e filter with lambda
                            # expression to find the tuple corresponding to the current timestep. Then find the index of
                            # this tuple in the current_path
                            # TODO: something goes wrong here. Current path wasn't updated correctly it seems
                            index_curr_timestep = curr_path.index(
                                list(filter(lambda node_t_pair: node_t_pair[1] == t in node_t_pair, curr_path))[0])
                            # now change the current_path of the AC from this step onwards into the newly calculated path
                            curr_path[index_curr_timestep:] = path_self
                            self.current_path = curr_path.copy()
                        else:
                            self.current_path = path_self.copy()
                        # compute travelling time and distance performance indiactors
                        self.compute_time_distance()

                    elif ac2.planned_t:
                        # if self wins biddding war, ac2 has to adjust path
                        ac2.path_to_goal = path_ac2[1:]
                        next_pos_losing = path_ac2[1][0]
                        ac2.from_to = [path_ac2[0][0], next_pos_losing]
                        # update the current path of the AC. Used for determining performance indicators
                        curr_path = ac2.current_path.copy()
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

                elif len(collisions) > 0 and ac2.planned_t and not helper_boolean:
                    # if there is a collision between self and an already planned AC, self will need to add a constraint
                    # to itself for a path construction that avoids crashing into ac2. However, self should not be
                    # considered as "planned" because it will still be able to start bidding wars with other AC
                    # add constraint which prohibits self from crashing into ac2 at next timestep
                    #collision = collisions[0]
                    #constraints = standard_splitting(collision, dt)
                    #self.constraints.append(constraints[0].copy())
                    # extract first collision
                    collision = collisions[0]
                    constraints = []

                    # first check whether one of the Ac is at an intersection, heading into the other AC which is at a
                    # node between intersections. This is a right of way situation, where the AC at the intersection
                    # HAS to move. No bidding is done in this case
                    if len(self.nodes_dict[curr_pos_self]["neighbors"]) == 2 and len(
                            self.nodes_dict[curr_pos_ac2]["neighbors"]) > 2:
                        # self is located between intersections, AC2 is located at intersection and heading into it
                        # impose constraints on AC2 such that it HAS to move away from th intersection
                        constraints.append({'acid': ac2.id, 'loc': [curr_pos_ac2], 'timestep': t + dt})
                        constraints.append({'acid': ac2.id, 'loc': [curr_pos_self], 'timestep': t + dt})

                    elif len(self.nodes_dict[curr_pos_self]["neighbors"]) > 2 and len(
                            self.nodes_dict[curr_pos_ac2]["neighbors"]) == 2:
                        # ac2 is located between intersections and self is located at intersection and heading into ac2
                        # impose constraints on self such that self HAS to move away from intersection
                        constraints.append({'acid': self.id, 'loc': [curr_pos_ac2], 'timestep': t + dt})
                        constraints.append({'acid': self.id, 'loc': [curr_pos_self], 'timestep': t + dt})


                    # if no right of way situation occurred
                    else:
                        constraints = standard_splitting(collision, dt)

                    new_constraints = self.constraints + constraints
                    self.constraints = new_constraints.copy()

                    # find alternative path
                    success, updated_path, exp_nodes = astar(self.nodes_dict, curr_pos_self, self.goal, heuristics,
                                                          self.constraints, t, dt, self.id, True, self)
                    if not success:
                        # if self is in a deadlock, the function should immediately return. It cannot plan any further
                        deadlocks += 1  # performance indicator
                        expanded_nodes += exp_nodes  # performance indicator
                        deadlock_ac.append(self)
                        deadlock_occurred = True
                        #print('!!! DEADLOCK FOR SELF !!!')
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
                    for i in range(index_ac2 + 1):
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
                                exp_nodes, deadlcks, deadlock_ac = self.perform_ind_planning(observed_ac[:index_ac2+1], t, dt, heuristics)
                                expanded_nodes += exp_nodes
                                if len(deadlock_ac) > 0:
                                    deadlocks += deadlcks
                                    return expanded_nodes, deadlocks, deadlock_ac

                    # the program gets to this line if there were no collisions with the other ac
                    # if the loop is finished without collisions the current path and necessary instance variables
                    # can be updated
                    curr_path = self.current_path.copy()
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

    def perform_ind_planning2(self, observed_ac, t, dt, heuristics, deadlock_acids, observation_size):

        # performance indicators
        expanded_nodes = 0
        deadlocks = 0

        # if there's an aircraft in a deadlock position, it will be added to this list
        deadlock_ac = []

        # the amount of detected collisions for planning this aircraft
        detected_collisions = 0

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
                deadlock_acids.append(self.id)
                #print('no base path found for ac1')

        # loop over the other AC that this AC can see in its observation space
        for ac2 in observed_ac:
            if ac2.id != self.id and ac2 not in deadlock_acids:
                # determine path for the next 2 timesteps for self
                path_self = [(curr_pos_self, t)] + self.path_to_goal[:observation_size].copy()
                # determine current position of ac2
                curr_pos_ac2 = ac2.from_to[0] if ac2.from_to[0] != 0 else ac2.start
                # path for next 2 timesteps. Included current timestep and position as wel to detect edge constraints
                path2 = [(curr_pos_ac2, t)] + ac2.path_to_goal[:observation_size].copy()
                # if AC2 doesn't have a path yet, plan it
                if len(path2) == 1 and ac2.status == "taxiing":
                    success, path, exp_nodes = astar(self.nodes_dict, curr_pos_ac2, ac2.goal, heuristics,
                                                     ac2.constraints, t, dt, ac2.id, True, ac2)
                    if success:
                        ac2.path_to_goal = path[1:]
                        ac2.from_to = [path[0][0], path[1][0]]
                        # update current path, used for travelling time and distance performance indicator
                        ac2.current_path = path.copy()
                        # performance indicator
                        expanded_nodes += exp_nodes
                        path2 = [(curr_pos_ac2, t)] + ac2.path_to_goal[:observation_size].copy()
                    else:
                        # performance indicator
                        deadlocks += 1
                        deadlock_ac.append(ac2)
                        deadlock_acids.append(ac2.id)
                        ##print('no base path found for ac2')

                # first check whether ac2 is in the loss_list of self. If this is the case, self will have to replan
                # using the instance variable constraints (self received these constraints when it encountered ac2
                # for planning )
                if ac2.id in self.loss_list:
                    success, path_updated, exp_nodes = astar(self.nodes_dict, curr_pos_self, self.goal, heuristics,
                                                     self.constraints, t, dt, self.id, True, self)
                    if success:
                        next_node = path_updated[1][0]  # node at wich self will be on next timestep
                        # add constraints to other AC in the observation space to ensure these don't crash into self
                        # when trying to plan their path
                        for acc in observed_ac:
                            if acc.id != self.id:
                                acc.constraints.append({'acid': acc.id, 'loc': [next_node], 'timestep': t + dt})

                        # now update the necessary instance variables for the newly defined path
                        self.update_path_variables(self, path_updated, t)
                        # remove this ACID from the loss list because the collision is resolved
                        self.loss_list.remove(ac2.id)

                    else:       # deadlock
                        deadlocks += 1
                        deadlock_acids.append(self.id)
                        deadlock_ac.append(self)
                        # TODO: should something else happen for a deadlock situation?
                        return expanded_nodes, deadlocks, deadlock_ac, detected_collisions

                # AC2 is not in the loss_list of self
                else:
                    # detect collisions between the AC4s current paths for the next 2 timesteps
                    collisions = detect_collisions([path_self, path2], [self.id, ac2.id], dt)
                    if collisions:
                        detected_collisions += 1
                        collision = collisions[0]
                        constraints = []       # constraints resulting from the collision
                        # indicates whether a right of way situation occurred for self
                        right_of_way_happened = False
                        # denotes whether AC2 is at the intersection. If this is the case, the AC shouldn't be treated
                        ac2_at_intersection = False
                        # check whether self is at an intersection and ac2 is not
                        if len(self.nodes_dict[curr_pos_self]["neighbors"]) > 2 and len(
                                self.nodes_dict[curr_pos_ac2]["neighbors"]) <= 2:
                            right_of_way_happened = True
                            # find constraint corresponding to self
                            # if self and ac2 want to travel in the same direction, the collision will be a vertex collision.
                            # This collision can be solved using only 1 constraint.
                            # Check if this is the case:
                            if len(collision['loc']) == 1:
                                # directly append the constraint to self, because it will have to move
                                self.constraints.append({'acid': self.id, 'loc': collision['loc'],
                                                         'timestep': collision['timestep']})

                            # else if we have a head on edge collision, the collision can be solved using 2 constraints
                            # which will force self to move away from the intersection
                            elif len(collision['loc']) == 2:
                                # check for same departure time collision, these have the same format as edge
                                # constraints but the time step on which they should work is different
                                if 1 in collision['loc'] and 2 in collision['loc']:
                                    self.constraints.append({'acid': self.id, 'loc': [collision['loc'][0]],
                                                             'timestep': collision['timestep']})
                                else:
                                    # directly append the constraints to self because it will have to move
                                    self.constraints.append({'acid': self.id, 'loc': [collision['loc'][0]],
                                                             'timestep': collision['timestep'] + dt})
                                    self.constraints.append({'acid': self.id, 'loc': [collision['loc'][1]],
                                                             'timestep': collision['timestep'] + dt})
                            else:
                                raise BaseException('something wrong with the collision location')

                        # now check whether ac2 is at an intersection and sef is not. If this is the case, we can skip
                        # this AC for planning and it will replan itself later on
                        elif len(self.nodes_dict[curr_pos_self]["neighbors"]) <= 2 and len(
                                  self.nodes_dict[curr_pos_ac2]["neighbors"]) > 2:
                            right_of_way_happened = True
                            ac2_at_intersection = True

                        # if no intersection situation occurred, determine 1 constraint for each aircraft to solve the
                        # collision. Afterwards, determine paths using new constraints and start bidding war
                        if not right_of_way_happened:
                            constraints = standard_splitting(collision, dt)

                            self_lost = False
                            ac2_lost = False

                            # now plan new path for self with with extra constraints
                            success, new_path_self, exp_nodes = astar(self.nodes_dict, curr_pos_self, self.goal,
                                                                      heuristics, self.constraints + constraints, t, dt,
                                                                      self.id, True, self)
                            expanded_nodes += exp_nodes

                            if not success:     # deadlock
                                ac2_lost = True

                            # plan new path for AC2 using extra constraints
                            success, new_path_ac2, exp_nodes = astar(self.nodes_dict, curr_pos_ac2, ac2.goal,
                                                                      heuristics, ac2.constraints + constraints, t, dt,
                                                                      ac2.id, True, ac2)
                            expanded_nodes += exp_nodes

                            if not success:     # deadlock
                                self_lost = True

                            # both AC are deadlocked?
                            if self_lost and ac2_lost:
                                deadlocks += 1
                                deadlock_acids.append(self.id)
                                deadlock_ac.append(self)
                                # TODO: should something else happen for a deadlock situation?
                                return expanded_nodes, deadlocks, deadlock_ac, detected_collisions

                            else:
                                if self_lost:   # AC2 will be in deadlock. So it will have to adjust path. No bidding happens
                                    # update the necessary instance variables
                                    next_node_self = self.update_path_variables(self, new_path_self, t)
                                    # append the constraint to self
                                    constraints_self = self.constraints + constraints
                                    self.constraints = constraints_self.copy()

                                    # now we add constraints to other AC in observed AC such that they don't collide with self
                                    for acc in observed_ac:
                                        if acc.id != self.id:
                                            acc.constraints.append({'acid': acc.id, 'loc': [next_node_self],
                                                                     'timestep': t + dt})

                                elif ac2_lost:  # self will be in deadlock, So AC2 needs to adjust path. No bidding happens
                                    # append self to loss list and update AC2 constraints
                                    ac2.update_loss_list_constraints(ac2, self, constraints)

                                elif not self_lost and not ac2_lost:           # no deadlocks occurred, let's start the bidding war
                                    # determine costs for each AC
                                    cost_self = (len(new_path_self) - 1 - len(self.path_to_goal))/len(self.path_to_goal)
                                    cost_ac2 = (len(new_path_ac2) - 1 - len(ac2.path_to_goal))/len(ac2.path_to_goal)
                                    # determine bid of each AC
                                    bid_self = cost_self * self.budget
                                    bid_ac2 = cost_ac2 * ac2.budget
                                    # compare bids and choose winner
                                    self_won = False
                                    ac2_won = False
                                    if bid_self != bid_ac2:
                                        self_won = True if bid_self > bid_ac2 else False
                                        ac2_won = not self_won

                                    elif bid_self == bid_ac2:
                                        # determine winner arbitrarily
                                        r = random.random()
                                        if r < 0.5:
                                            self_won = True
                                        else:
                                            ac2_won = True
                                    else:
                                        raise BaseException('Bids are non-numeric')

                                    if self_won:
                                        # update the remaining budget of self
                                        self.budget = self.budget - 1.1 * bid_ac2 if 1.1 * bid_ac2 < bid_self \
                                                        else self.budget - bid_self
                                        # append self to loss list and update AC2 constraints
                                        ac2.update_loss_list_constraints(ac2, self, constraints)

                                    elif ac2_won:
                                        # update remaining AC2 budget
                                        ac2.budget = ac2.budget - 1.1 * bid_self if 1.1 * bid_self < bid_ac2 \
                                                    else ac2.budget - bid_ac2
                                        # update the necessary instance variables
                                        next_node_self = self.update_path_variables(self, new_path_self, t)
                                        # append the constraint to self
                                        constraints_self = self.constraints + constraints
                                        self.constraints = constraints_self.copy()

                                        # now we add constraints to other AC in observed AC such that they don't collide with self
                                        for acc in observed_ac:
                                            if acc.id != self.id:
                                                acc.constraints.append({'acid': acc.id, 'loc': [next_node_self],
                                                                         'timestep': t + dt})
                                    else:   # something wrong
                                        raise BaseException('no winner of bidding war')

                        # right_of_way_happened is True if self is at the intersection. Self will have to adjust without
                        # bidding
                        elif right_of_way_happened and not ac2_at_intersection:
                            # plan new path for self with extra constraints
                            success, new_path_self, exp_nodes = astar(self.nodes_dict, curr_pos_self, self.goal,
                                                                      heuristics, self.constraints, t, dt,  #used to be self.constraint + constraints
                                                                      self.id, True, self)
                            expanded_nodes += exp_nodes

                            if not success:     # deadlock
                                deadlocks += 1
                                deadlock_acids.append(self.id)
                                deadlock_ac.append(self)
                                # TODO: should something else happen for a deadlock situation?
                                return expanded_nodes, deadlocks, deadlock_ac, detected_collisions

                            # update the necessary instance variables
                            self.update_path_variables(self, new_path_self, t)

                            # now we add constraints to other AC in observed AC such that they don't collide with self
                            next_node_self = new_path_self[1][0]
                            for acc in observed_ac:
                                if acc.id != self.id:
                                    acc.constraints.append({'acid': acc.id, 'loc': [next_node_self],
                                                             'timestep': t + dt})

        return expanded_nodes, deadlocks, deadlock_ac, detected_collisions

    def update_path_variables(self, ac, path, t):
        """
        updates instance variables concerning the planned path of the AC after replanning. The updated variables are
        1) path_to_goal
        2) from_to
        3) current_path
        Args:
            ac: the aircraft for which replanning was done
            path: the updated path found, starting at the current timestep
            t: current timestep

        Returns: None

        """
        next_node = path[1][0]  # node at wich AC will be on next timestep
        ac.from_to = [path[0][0], next_node]
        ac.path_to_goal = path[1:]
        # fetch the planned path before the replannning
        curr_path = self.current_path.copy()
        # if AC already has a path planned
        if len(curr_path) > 0:
            # find index in current_path that corresponds to this timestep. Us e filter with lambda
            # expression to find the tuple corresponding to the current timestep. Then find the index of
            # this tuple in the current_path
            index_curr_timestep = curr_path.index(
                list(filter(lambda node_t_pair: node_t_pair[1] == t in node_t_pair, curr_path))[0])
            # now change the current_path of the AC from this step onwards into the newly calculated path
            curr_path[index_curr_timestep:] = path
            ac.current_path = curr_path.copy()
        else:
            ac.current_path = path.copy()

        return next_node

    def update_loss_list_constraints(self, ac_lose, ac_win, constraints):
        """
        small function that updates the loss list of AC2 for individual planning. This function gets called when
        the aircraft other than the one on which the function is called has to adjust its path during individual
        planning. It also appends constraints to the losing aircraft which it will later use to plan its path in such
        a way that it doesn't collide with the path of ac_win
        Args:
            ac_lose: the losing aircraft (ac2 in perform_ind_planning)
            ac_win: the winning aircraft (self in perf_ind_planning)
            constraints: constraints to be appended

        Returns:

        """
        # append self to ac2 loss list
        ac_lose.loss_list.append(ac_win.id)
        # append constraints to AC2
        constraints_ac_lose = ac_lose.constraints + constraints
        ac_lose.constraints = constraints_ac_lose.copy()

        return None
