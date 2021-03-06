"""
Run-me.py is the main file of the simulation. Run this file to run the simulation.
"""

import os
import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import time as timer
import pygame as pg
import numpy as np
import random
from single_agent_planner import calc_heuristics
from visualization import map_initialization, map_running
from Aircraft import Aircraft
from independent import run_independent_planner
from prioritized import run_prioritized_planner
from individual import run_individual_planner
from cbs import run_CBS

# %% SET SIMULATION PARAMETERS
# Input file names (used in import_layout) -> Do not change those unless you want to specify a new layout.
nodes_file = "nodes.xlsx"  # xlsx file with for each node: id, x_pos, y_pos, type
edges_file = "edges.xlsx"  # xlsx file with for each edge: from  (node), to (node), length

########################################################################################################################
# Adjustable parameters
########################################################################################################################

simulation_time = 40
planner = "Individual"  # choose which planner to use (currently only Independent is implemented)

# Visualization (can also be changed)
plot_graph = False  # show graph representation in NetworkX
visualization = False  # pygame visualization
visualization_speed = 0.5  # set at 0.1 as default

# number of times the simulation should be ran
NUM_OF_SIMULATIONS = 5

# specify arrival rate
arrival_rate = "high"

# for individual planning: specify the size of the observation area
# 1 means that it will only see its neighboring nodes
# 2 means that it will account for neighboring nodes and their neighbors
# ...
ind_obs_size = 2

########################################################################################################################

# %%Function definitions
def import_layout(nodes_file, edges_file):
    """
    Imports layout information from xlsx files and converts this into dictionaries.
    INPUT:
        - nodes_file = xlsx file with node input data
        - edges_file = xlsx file with edge input data
    RETURNS:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges annd edge properties
        - start_and_goal_locations = dictionary with node ids for arrival runways, departure runways and gates 
    """
    gates_xy = []  # lst with (x,y) positions of gates
    rwy_dep_xy = []  # lst with (x,y) positions of entry points of departure runways
    rwy_arr_xy = []  # lst with (x,y) positions of exit points of arrival runways

    df_nodes = pd.read_excel(os.getcwd() + "/" + nodes_file)
    df_edges = pd.read_excel(os.getcwd() + "/" + edges_file)

    # Create nodes_dict from df_nodes
    nodes_dict = {}
    for i, row in df_nodes.iterrows():
        node_properties = {"id": row["id"],
                           "x_pos": row["x_pos"],
                           "y_pos": row["y_pos"],
                           "xy_pos": (row["x_pos"], row["y_pos"]),
                           "type": row["type"],
                           "neighbors": set()
                           }
        node_id = row["id"]
        nodes_dict[node_id] = node_properties

        # Add node type
        if row["type"] == "rwy_d":
            rwy_dep_xy.append((row["x_pos"], row["y_pos"]))
        elif row["type"] == "rwy_a":
            rwy_arr_xy.append((row["x_pos"], row["y_pos"]))
        elif row["type"] == "gate":
            gates_xy.append((row["x_pos"], row["y_pos"]))

    # Specify node ids of gates, departure runways and arrival runways in a dict
    start_and_goal_locations = {"gates": gates_xy,
                                "dep_rwy": rwy_dep_xy,
                                "arr_rwy": rwy_arr_xy}

    # Create edges_dict from df_edges
    edges_dict = {}
    for i, row in df_edges.iterrows():
        edge_id = (row["from"], row["to"])
        from_node = edge_id[0]
        to_node = edge_id[1]
        start_end_pos = (nodes_dict[from_node]["xy_pos"], nodes_dict[to_node]["xy_pos"])
        edge_properties = {"id": edge_id,
                           "from": row["from"],
                           "to": row["to"],
                           "length": row["length"],
                           "weight": row["length"],
                           "start_end_pos": start_end_pos
                           }
        edges_dict[edge_id] = edge_properties

    # Add neighbor nodes to nodes_dict based on edges between nodes
    for edge in edges_dict:
        from_node = edge[0]
        to_node = edge[1]
        nodes_dict[from_node]["neighbors"].add(to_node)

    return nodes_dict, edges_dict, start_and_goal_locations


def create_graph(nodes_dict, edges_dict, plot_graph=True):
    """
    Creates networkX graph based on nodes and edges and plots 
    INPUT:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges annd edge properties
        - plot_graph = boolean (True/False) If True, function plots NetworkX graph. True by default.
    RETURNS:
        - graph = networkX graph object
    """

    graph = nx.DiGraph()  # create directed graph in NetworkX

    # Add nodes and edges to networkX graph
    for node in nodes_dict.keys():
        graph.add_node(node,
                       node_id=nodes_dict[node]["id"],
                       xy_pos=nodes_dict[node]["xy_pos"],
                       node_type=nodes_dict[node]["type"])

    for edge in edges_dict.keys():
        graph.add_edge(edge[0], edge[1],
                       edge_id=edge,
                       from_node=edges_dict[edge]["from"],
                       to_node=edges_dict[edge]["to"],
                       weight=edges_dict[edge]["length"])

    # Plot networkX graph
    if plot_graph:
        plt.figure()
        node_locations = nx.get_node_attributes(graph, 'xy_pos')
        nx.draw(graph, node_locations, with_labels=True, node_size=100, font_size=10)

    return graph


# %% RUN SIMULATION
# =============================================================================
# 0. Initialization
# =============================================================================

nodes_dict, edges_dict, start_and_goal_locations = import_layout(nodes_file, edges_file)
graph = create_graph(nodes_dict, edges_dict, plot_graph)
heuristics = calc_heuristics(graph, nodes_dict)

if visualization:
    map_properties = map_initialization(nodes_dict, edges_dict)  # visualization properties

# dataframe that will contain the values of all our performance indicators
df_kpi = pd.DataFrame({'travel_t_avg': [],  # average travel time for 1 simulation
                       'travel_t_std': [],  # stdev travel time for 1 simulation
                       'travel_d_avg': [],  # average travel distance for 1 simulation
                       'travel_d_std': [],  # stdev travel distance for 1 simulation
                       'travel_td_avg': [],     # avg travel time/distance ratio for 1 simulation
                       'travel_td_std': [],     # stdev travel time/distance ratio for 1 simulation
                       'throughput_avg': [],    # average throughput/timeste (0.5) for 1 simulation
                       'throughput_std': [],    # stdev throughput/timeste (0.5) for 1 simulation
                       'comput_t_avg': [],  # average computation time for 1 simulation
                       'comput_t_std': [],  # stdev computation time for 1 simulation
                       'exp_nodes': [],     # total amount of expanded nodes for 1 simulation
                       'deadlocks': []})    # amount of deadlocks for 1 simulation

# initialize list for stabilization of coefficients of variation
travel_time_cv = []
travel_distance_cv = []
travel_dt_ratio_cv = []
throughput_cv = []
computation_t_cv = []

# gate, departing rwy and arriving rwy node IDs, used for random AC spawning
gates_ids = []  # will contain IDs of gate nodes
rwy_a_ids = []  # will contain IDs of arrival runway nodes
rwy_d_ids = []  # will contain IDs of departure runway nodes
for node in nodes_dict:
    node_id = nodes_dict[node]["id"]
    node_type = nodes_dict[node]["type"]
    if node_type == "gate":
        gates_ids.append(node_id)
    elif node_type == "rwy_a":
        rwy_a_ids.append(node_id)
    elif node_type == "rwy_d":
        rwy_d_ids.append(node_id)

# will contain heat values for every node on the grid
heatmap = np.zeros(len(nodes_dict))
# =============================================================================
# 1. While loop and visualization
# =============================================================================
for i in range(NUM_OF_SIMULATIONS):
    if visualization:
        map_properties = map_initialization(nodes_dict, edges_dict)  # visualization properties
    # Start of while loop
    running = True
    escape_pressed = False
    time_end = simulation_time
    dt = 0.5  # should be factor of 0.5 (0.5/dt should be integer)
    t = 0
    aircraft_lst = []  # List which can contain aircraft agents
    # will be used as ID for spawning aircraft
    spawned_ac = 0
    # we start with an empty constraints list for the first spawned AC
    constraints = []
    # initialize lists for performance indicators
    travel_times = []
    travel_distances = []
    ratios = []
    throughput = dict()
    computing_times = []
    deadlocks = 0
    expanded_nodes = 0

    print("Simulation Started")
    while running:
        t = round(t, 2)

        # Check conditions for termination
        if t >= time_end or escape_pressed:
            running = False
            pg.quit()
            print("Simulation Stopped")
            break

            # Visualization: Update map if visualization is true
        if visualization:
            current_states = {}  # Collect current states of all aircraft
            for ac in aircraft_lst:
                if ac.status == "taxiing":
                    current_states[ac.id] = {"ac_id": ac.id,
                                             "xy_pos": ac.position,
                                             "heading": ac.heading}
            escape_pressed = map_running(map_properties, current_states, t)
            timer.sleep(visualization_speed)

        # Spawn aircraft for this timestep at random but make sure to not spawn them on another aircraft
        # in addition check if the spawned AC will be in deadlock, this happens when another AC is at the last 2 nodes
        # before a runway or gate. For this, all the AC paths currently in the field are checked whether at timestep t, they
        # are located at one of these positions

        if arrival_rate == "high":
            threshold = 0.35
        elif arrival_rate == "low":
            threshold = 0.2
        else:
            raise BaseException('no correct arrival rate specified')

        if random.random() < threshold:
            if random.random() < 0.5:  # departing AC
                # determine at which gate the AC starts
                start_node_index = random.randint(0, len(gates_ids) - 1)
                start_node = gates_ids[start_node_index]
                # determine at which rwy_d node the AC will take off
                goal_node_index = random.randint(0, len(rwy_d_ids) - 1)
                goal_node = rwy_d_ids[goal_node_index]
                arr_dep = 'D'

            else:  # arriving AC
                # determine at which rwy_a node the AC arrives
                start_node_index = random.randint(0, len(rwy_a_ids) - 1)
                start_node = rwy_a_ids[start_node_index]
                # determine at which gate the AC has to end
                goal_node_index = random.randint(0, len(gates_ids) - 1)
                goal_node = gates_ids[goal_node_index]
                arr_dep = 'A'

            # boolean to indicate whether the AC will spawn in a nasty situation
            spawn_on_other_ac = False
            for aircr in aircraft_lst:
                # current node of this AC
                ac_pos = aircr.from_to[0]
                # check if there's currently an AC at the spawn position
                if aircr.status == "taxiing":
                    if ac_pos == start_node:
                        spawn_on_other_ac = True
                        break

                    # if not, check if the AC will head straight into the spawned aircraft
                    # aka check if it will be located at the preceding node of the spawning node
                    # nodes_dict[start_node]["neighbors"] gives node IDs of neighbouring nodes
                    if ac_pos in nodes_dict[start_node]["neighbors"]:
                        spawn_on_other_ac = True
                        break

            # only if it's safe to spawn, add the ac to the list
            if not spawn_on_other_ac:
                ac = Aircraft(spawned_ac + 1, arr_dep, start_node, goal_node, t, nodes_dict)
                aircraft_lst.append(ac)
                spawned_ac += 1
            print('Aircraft spawned at ' + str(t) + ', position: ' + str(start_node))

        # Do planning
        if planner == "Independent":
            time_delta, exp_nodes = run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t)
            # computing time performance indicator
            computing_times.append(time_delta)
            # expanded nodes performance indicator
            expanded_nodes += exp_nodes
        elif planner == "Prioritized":
            time_delta, exp_nodes, deadlcks = run_prioritized_planner(aircraft_lst, nodes_dict, edges_dict, heuristics,
                                                                      constraints, dt, t)
            # expanded nodes performance indicator
            expanded_nodes += exp_nodes
            if deadlcks > 0:
                aircraft_lst.pop()
                # deadlocks performance indicator
                deadlocks += deadlcks
            # computing time performance indicator
            computing_times.append(time_delta)

        elif planner == "CBS":
            time_delta, exp_nodes, deadlcks = run_CBS(aircraft_lst, nodes_dict, edges_dict,
                                                            heuristics, dt, t)
            # expanded nodes performance indicator
            expanded_nodes += exp_nodes
            # computing time performance indicator
            computing_times.append(time_delta)
            # deadlocks performance indicator
            deadlocks += deadlcks

        elif planner == "Individual":
            # set the planned_t variable to False, only once per time step!
            for ac in aircraft_lst:
                ac.loss_list = []   # added for individual2
                if ac.spawntime == t:
                    ac.status = "taxiing"
                    ac.position = nodes_dict[ac.start]["xy_pos"]

            time_delta, exp_nodes, deadlcks = run_individual_planner(aircraft_lst, nodes_dict, heuristics, t, dt, ind_obs_size)
            # expanded nodes performance indicator
            expanded_nodes += exp_nodes
            # computing time performance indicator
            computing_times.append(time_delta)
            # deadlocks performance indicator
            deadlocks += deadlcks

        else:
            raise Exception("Planner:", planner, "is not defined.")

        # throughput performance indicator: A/C arrived at timestep t
        ac_arrived_t = 0

        # Move the aircraft that are taxiing
        for ac in aircraft_lst:
            if ac.status == "taxiing":
                # added constraints parameter to be able to remove constraints of arrived AC
                ac.move(dt, t, constraints)
                # if AC has reached its goal, increment the throughput value by 1, else,
                if ac.status == "arrived":
                    ac_arrived_t += 1

        # add amount of arrived AC for this timestep in throughput dict
        throughput[t] = ac_arrived_t

        t = t + dt
        # at the last time step, determine the time and distance performance indicators
        if t == simulation_time:
            for ac in aircraft_lst:
                ac.compute_time_distance()

    # =============================================================================
    # 2. Implement analysis of output data here
    # =============================================================================
    # what data do you want to show?

    # by now, all A/C have a planned path, path length, travel time and travel time/travel distance ratio
    # average these values over all aircraft and determine standard deviation

    for ac in aircraft_lst:
        travel_times.append(ac.travel_time)
        travel_distances.append(ac.path_length)
        ratios.append(ac.time_length_ratio)
        # for every aircraft, add +1 in heatmap for every node that it has visited
        # nodes which are visited multiple times get counted as well (e.g. waiting at a node gets counted twice)
        for node, time in ac.current_path:
            heatmap[int(node) - 1] += 1

    # average out results for this simulation
    avg_travel_time = np.mean(travel_times)
    avg_travel_distance = np.mean(travel_distances)
    avg_ratio = np.mean(ratios)
    avg_computing_time = np.mean(computing_times)
    throughputs = list(throughput.values())  # throughputs for every timestep t
    avg_throughput = np.mean(throughputs)
    # for throughput, the time scale can be chosen arbitrarily as well, e.g. AC arriving per 5 seconds
    # note: the interval should be a factor of the simulation time!
    interval = 5
    index = 0
    throughput_interval = []

    for i in range(int(simulation_time / interval)):
        values = throughputs[int(index):int(index + interval / dt)]
        throughput_interval.append(sum(values))
        index += interval / dt

    avg_throughput_interval = np.mean(throughput_interval)

    # calculate standard deviations between AC
    std_travel_time = np.std(travel_times)
    std_travel_distance = np.std(travel_distances)
    std_ratio = np.std(ratios)
    std_throughput = np.std(throughputs)
    std_computing_time = np.std(computing_times)

    # append these values to the KPi dataframe. Add an extra row
    df_kpi.loc[len(df_kpi.index)] = [avg_travel_time,
                                     std_travel_time,
                                     avg_travel_distance,
                                     std_travel_distance,
                                     avg_ratio,
                                     std_ratio,
                                     avg_throughput,
                                     std_throughput,
                                     avg_computing_time,
                                     std_computing_time,
                                     expanded_nodes,
                                     deadlocks]

    # coefficient of variation
    travel_time_cv.append(np.std(df_kpi['travel_t_avg'])/np.mean(df_kpi['travel_t_avg']))
    travel_distance_cv.append(np.std(df_kpi['travel_d_avg'])/np.mean(df_kpi['travel_d_avg']))
    travel_dt_ratio_cv.append(np.std(df_kpi['travel_td_avg'])/np.mean(df_kpi['travel_td_avg']))
    throughput_cv.append(np.std(df_kpi['throughput_avg'])/np.mean(df_kpi['throughput_avg']))
    computation_t_cv.append(np.std(df_kpi['comput_t_avg'])/np.mean(df_kpi['comput_t_avg']))

    # print results
    print("Average travel time: " + str(avg_travel_time) + ", standard deviation: " + str(std_travel_time))
    print("Average travel distance: " + str(avg_travel_distance) + ", standard deviation: " + str(std_travel_distance))
    print("Average time/distance ratio: " + str(avg_ratio) + ", standard deviation: " + str(std_ratio))
    print("Average throughput: " + str(avg_throughput))
    print("Average throughput per " + str(interval) + " seconds: " + str(avg_throughput_interval))
    print("Average computing time: " + str(avg_computing_time) + ' nanoseconds')
    print("Expanded nodes: " + str(expanded_nodes))
    print("Deadlocks: " + str(deadlocks))
    print('Arrived AC: ' + str(sum(throughputs)))


########################################################################################################################
# Export results to .csv file
########################################################################################################################
if planner == "Prioritized":
    if arrival_rate == "low":
        output_file = "results_prio_lo.csv"
    else:
        output_file = "results_prio_hi.csv"
elif planner == "CBS":
    if arrival_rate == "low":
        output_file = "results_cbs_lo.csv"
    else:
        output_file = "results_cbs_hi.csv"
elif planner == "Individual":
    if arrival_rate =="low":
        output_file = "results_ind_lo_.csv"
    else:
        output_file = "results_ind_hi_.csv"

df_kpi.to_csv(output_file)

########################################################################################################################
# Statistical analysis
########################################################################################################################

# the coefficients of variation are split up into 3 arrays wwhere the values are of equal order of magnitude. This is
# done for scaling reasons while plotting
# these 3 KPIs have similar coefficients of variation
coeffs_variation_1 = np.transpose([travel_time_cv, travel_distance_cv, travel_dt_ratio_cv])
# throughput KPI CV is of a different order of magnitude
coeffs_variation_2 = np.transpose([throughput_cv])
# computation time KPi CV is also of a different order of magnitude
coeffs_variation_3 = np.transpose([computation_t_cv])

# plot evolution of coefficient of variation for alll KPI except computation time
fig1 = plt.figure()
plt.plot(coeffs_variation_1, label=['avg travel time', 'avg travel distance', 'avg distance/time ratio'])
plt.xlabel('Episode')
plt.ylabel('Coefficient of variation')
plt.title('Evolution of CV for for avg travel time, distance and time/distance ratio')
plt.legend()

# plot evolution of coefficient of variation for throughput
fig2 = plt.figure()
plt.plot(coeffs_variation_2, label='avg throughput')
plt.xlabel('Episode')
plt.ylabel('Coefficient of variation')
plt.title('Evolution of CV for avg throughput')
plt.legend()

# plot evolution of coefficient of variation for computation time
fig3 = plt.figure()
plt.plot(coeffs_variation_3, label='avg computation time')
plt.xlabel('Episode')
plt.ylabel('Coefficient of variation')
plt.title('Evolution of CV for avg computation time')
plt.legend()

# plot heat map
max_heat = 3100   # experimentally, the max heat for all planners and arrival rates was around 2900. Set 3100 to be safe
# now normalize heatmap with respect to 1. The max_heat value will correspond to 1
# the nx.draw function needs a color map with floats ranging from 0-1, so that's why we don't use actual
# RGB values up until 255
heatmap = [(1, 1 - a / float(max_heat), 0) for a in heatmap]
# plot the graph (heatmap)
fig4 = plt.figure()
node_locations = nx.get_node_attributes(graph, 'xy_pos')
nx.draw(graph, node_locations, with_labels=True, node_size=100, font_size=10, node_color=heatmap)
plt.title('Heatmap showing the most often visited nodes. RGB value normalized to a max of 2000 visits')
plt.show()

# placeholder for debugging such that we can set breakpoint here
a= 5
