from single_agent_planner import simple_single_agent_astar, astar
import math


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

    def move(self, dt, t):
        """
        Moves an aircraft between from_node and to_node and checks if to_node or goal is reached.
        INPUT:
            - dt = time step delta
            - t = current time
        """

        # Determine nodes between which the ac is moving
        from_node = self.from_to[0]
        to_node = self.from_to[1]
        xy_from = self.nodes_dict[from_node]["xy_pos"]  # xy position of from node
        xy_to = self.nodes_dict[to_node]["xy_pos"]  # xy position of to node
        distance_to_move = self.speed * dt  # distance to move in this timestep

        # Update position with rounded values
        x = xy_to[0] - xy_from[0]
        y = xy_to[1] - xy_from[1]
        x_normalized = x / math.sqrt(x ** 2 + y ** 2)
        y_normalized = y / math.sqrt(x ** 2 + y ** 2)
        posx = round(self.position[0] + x_normalized * distance_to_move, 2)  # round to prevent errors
        posy = round(self.position[1] + y_normalized * distance_to_move, 2)  # round to prevent errors
        self.position = (posx, posy)
        self.get_heading(xy_from, xy_to)

        # Check if goal is reached or if to_node is reached
        if self.position == xy_to and self.path_to_goal[0][1] == t + dt:  # If with this move its current to node is reached
            if self.position == self.nodes_dict[self.goal]["xy_pos"]:  # if the final goal is reached
                self.status = "arrived"

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

    # TODO: add function plan_prioritized and plan_cbs

    def plan_prioritized(self, nodes_dict, edges_dict, heuristics, constraints, t):
        """
        Plans path for taxiing aircraft where constraints are constructed on the go in terms of priority
        Args:
            nodes_dict:
            edges_dict:
            heuristics:
            t:

        Returns:
            expanded nodes
            updated constraints
        """
        constr = constraints.copy()

        if self.status == "taxiing":
            start_node = self.start  # node from which planning should be done
            goal_node = self.goal  # node to which planning should be done

            success, path, expanded_nodes = astar(nodes_dict, start_node, goal_node, heuristics, constraints, t)

            if success:
                if path[0][1] != t:
                    raise Exception("Something is wrong with the timing of the path planning")
                self.path_to_goal = path[1:]
                next_node_id = self.path_to_goal[0][0]  # next node is first node in path_to_goal
                self.from_to = [path[0][0], next_node_id]
                print("Path AC", self.id, ":", path)
                # determine length and time of travelled path + their ratio
                self.compute_time_distance(path)
                print("travel time AC", self.id, ":", self.travel_time)
                print("travel distance AC", self.id, ":", self.path_length)
                print("travel time/distance ratio AC", self.id, ":", self.time_length_ratio)

                # now add constraints to other agents
                # constraints are not agent-specific, bu spawn time specific
                # every agent that spawns after or at the time specified in the constraint, will have
                # to obey to this constraint
                timestep = self.spawntime
                for node_time_pair in path: # path is list of (node_id, timestep) tuples
                    node = node_time_pair[0]    # find node_id
                    # vertex constraint
                    constr.append({'spawntime': self.spawntime, 'loc': [node], 'timestep': timestep})
                    # edge constraint:
                    # find previous node in AC path
                    previous_node = path[timestep-1][0] if timestep > self.spawntime else self.start
                    constr.append({'spawntime': self.spawntime, 'loc': [node, previous_node], 'timestep': timestep})

                    timestep += 1
                return expanded_nodes, constr

            else:
                raise Exception("No solution found for", self.id)


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
