from single_agent_planner import simple_single_agent_astar, astar
from cbs import detect_collision
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
        self.travel_time = len(self.path_to_goal)    # total travel time
        self.visited_nodes = []     # different nodes visited by A/C. To determine path_length
        self.path_length = len(self.visited_nodes)    # total spatial distance of calculated path
        self.time_length_ratio = 0  # ratio between travel time and path length. Indicates waiting time

        # individual planning
        self.observation_space = dict()
        # TODO: find suitable value
        self.budget = 100
        self.constraints = []

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
                # determine length and time of travelled path + their ratio
                self.compute_time_distance(path)
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
        """

        if self.status == "taxiing":
            start_node = self.start  # node from which planning should be done
            goal_node = self.goal  # node to which planning should be done

            # the parameter 0 denotes that the aircraft Id is not important here. It won't be used in the A star algorithm.
            # The False parameter indicated that the A star algorithm has to construct the constraint table using
            # the prioritized constraint format
            success, path, expanded_nodes = astar(nodes_dict, start_node, goal_node, heuristics, constraints, t, dt, 0, False)

            if success:
                if path[0][1] != t:
                    raise Exception("Something is wrong with the timing of the path planning")
                self.path_to_goal = path[1:]
                next_node_id = self.path_to_goal[0][0]  # next node is first node in path_to_goal
                self.from_to = [path[0][0], next_node_id]
                print("Path AC", self.id, ":", path)
                # determine length and time of travelled path + their ratio
                self.compute_time_distance(path)
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

                print('constraints after plan_prioritized: ' + str(constraints))
                return expanded_nodes, constraints, 0

            else:
                print('No solution found for ' + str(self.id))
                return expanded_nodes, constraints, 1

    def compute_time_distance(self, path):
        """
        computes the performance indicators for travel time, travel distance and their ratio
        This function is called when the A/C already has a planned path. It then updates instance
        variables representing these performance indicators
        """
        # travel time performance indicator
        start_time = path[0][1]
        arrival_time = path[-1][1]
        self.travel_time = float(arrival_time - start_time)

        # travel distance performance indicator
        # check whether the aircraft waits at a certain node. If not, add this node to the list of different
        # visited nodes
        for position in path:
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
        for ac2 in observed_ac:
            # path for next 2 timesteps
            path2 = ac2.path_to_goal[:2]
            path = self.path_to_goal[:2]
            collision = detect_collision(path, path2, dt)
            if collision is not None:
                constraints = []
                # first check whether one of the Ac is at an intersection, heading into the other AC which is at a
                # node between intersections
                curr_pos_self = self.from_to[0] if self.from_to[0] != 0 else self.start
                curr_pos_ac2 = ac2.from_to[0] if ac2.from_to [0] != 0 else ac2.start
                if len(self.nodes_dict[curr_pos_self]["neighbors"]) == 2 and len(self.nodes_dict[curr_pos_ac2]["neighbors"]) == 4:
                    # self is located between intersections, AC2 is located at intersection and heading into it
                    # impose constraints on AC2 such that it HAS to move away from th intersection
                    constraints.append[
                        {'spawntime': ac2.spawntime, 'loc': [curr_pos_ac2], 'timestep': t + dt, 'acid': ac2.id}]
                    constraints.append[
                        {'spawntime': ac2.spawntime, 'loc': [curr_pos_self], 'timestep': t + dt, 'acid': ac2.id}]
                elif len(self.nodes_dict[curr_pos_self]["neighbors"]) == 4 and len(self.nodes_dict[curr_pos_ac2]["neighbors"]) == 2:
                    # ac2 is located between intersections and self is located at intersection and heading into ac2
                    # impose constraints on self such that self HAS to move away from intersection
                    constraints.append[
                        {'spawntime': self.spawntime, 'loc': [curr_pos_ac2], 'timestep': t + dt, 'acid': self.id}]
                    constraints.append[
                        {'spawntime': self.spawntime, 'loc': [curr_pos_self], 'timestep': t + dt, 'acid': self.id}]
                else:
                    # we have a regular collision. Create constraints
                    loc = collision['loc']
                    timestep = collision['timestep']
                    # edge collision
                    if len(loc) > 1:
                        constraints.append(
                            {'spawntime': self.spawntime, 'loc': [loc[0], loc[1]], 'timestep': timestep + dt, 'acid': self.id})
                        constraints.append(
                            {'spawntime': ac2.spawntime, 'loc': [loc[1], loc[0]], 'timestep': timestep + dt, 'acid': ac2.id})
                    # vertex collision
                    elif len(loc) == 1:
                        constraints.append(
                            {'spawntime': self.spawntime, 'loc': [loc], 'timestep': timestep, 'acid': self.id})
                        constraints.append(
                            {'spawntime': ac2.spawntime, 'loc': [loc], 'timestep': timestep, 'acid': ac2.id})

                # now, plan the paths independently for both AC
                path_self = astar(self.nodes_dict, curr_pos_self, self.goal, heuristics, constraints, t, dt, self.acid,
                                  False, self)
                path_ac2 = astar(self.nodes_dict, curr_pos_ac2, ac2.goal, heuristics, constraints, t, dt, ac2.acid,
                                 False, ac2)
                # determine cost for each of the AC involved, calculated as fraction increase w.r.t. old path
                cost_self = ((len(path_self) - len(path))/len(path) - 1)
                cost_ac2 = ((len(path_ac2) - len(path2))/len(path2) - 1)

                # calculate corresponding bids
                bid_self = cost_self * self.budget
                bid_ac2 = cost_ac2 * ac2.budget
                # TODO: constraints of losing AC 
                # if current AC wins negotiation or same bids and self has largest cost
                if bid_self > bid_ac2 or (bid_self == bid_ac2 and cost_self > cost_ac2):
                    self.budget = self.budget - 1.1*bid_ac2 if 1.1*bid_ac2 < bid_self else self.budget - bid_self
                    self.path_to_goal = path_self[1:]

                # if AC2 wins negotioation or same bids and AC2 has largest cost
                elif bid_self < bid_ac2 or (bid_self == bid_ac2 and cost_self < cost_ac2):
                    ac2.budget = ac2.budget - 1.1*bid_self if 1.1*bid_self < bid_ac2 else ac2.budget - bid_ac2
                    ac2.path_to_goal = path_ac2[1:]

                # if everything is equal, determine arbitrary
                elif bid_self == bid_ac2 and cost_self == cost_ac2:
                    if random.random() < 0.5:
                        self.budget = self.budget - 1.1 * bid_ac2 if 1.1 * bid_ac2 < bid_self else self.budget - bid_self
                        self.path_to_goal = path_self[1:]
                    else:
                        ac2.budget = ac2.budget - 1.1 * bid_self if 1.1 * bid_self < bid_ac2 else ac2.budget - bid_ac2
                        ac2.path_to_goal = path_ac2[1:]

                # something went wrong
                else:
                    raise BaseException('something wrong with bids and costs')


        return None