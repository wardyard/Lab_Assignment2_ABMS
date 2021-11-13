These results were obtained for 200 simulation runs of 40 time steps each. 

*Arrival rate*
Arrival rate was enforced via a probability of spawning an aircraft at any time step t.
If a random float was lower than 0.2 (low arrival rate) or 0.35 (high arrival rate), an aircraft is swpaned at t.
The max arrival rate was based on the CBS planner. It started having serious difficulties at an arrival rate of 0.35.

*Simulation hardware* 
All results were obtained on the same hardware in equal circumstances: 
- Intel(R) Core(TM) i7-7700HQ CPU @ 2.80GHz Octacore
- Nvidia GeForce 1050 GPU

*Heatmaps*
Heatmap is based on amount of times nodes are visited. To be able to compare heatmaps for different planners, 
the max amount of times a node was visited for all planners and arrival rate was found. This number tunred out to be 
2903. To be on the safe side, tha maximum value was chosen to be 3100. All RGB values get scaled using this maximum
in order to compare different planners based on heat maps. 

*Coefficient of variation*
The coefficient of variation figures are split in 3 parts due to their difference in order of magnitude. The first 
figure deals with travel time, travel distance an dthe travel time/distance ratio, whereas de other 2 denote the 
average throughput and computation time coefficient of variation evolution respectively. 