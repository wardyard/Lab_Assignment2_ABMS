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
        self.budget = 100
        self.constraints = []   # constraints regarding this AC
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
                #self.compute_time_distance()
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
                #self.compute_time_distance()
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
        self.path_length = len(self.visited_nodes)

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

    def perform_ind_planning(self, observed_ac, t, dt, heuristics, deadlock_acids, observation_size):

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
                #self.compute_time_distance()
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
