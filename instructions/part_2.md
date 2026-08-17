# Part 2 — UWB Anchor Global Localization (GDOP & Incorrect Anchor Placement)

## General 
Read the project level AGENTS.md 
Make sure to understand the ARP_utility_code and manifpy libraries.
Make sure to reuse the code in src from part 1 as much as possible. 

## Excercises 
| Excercise                                 | Assumptions Trajectory | Global pose estimation | Uncertainty                            |
|-------------------------------------------|-------------------------------------------|------------------------|----------------------------------------| 
| Single robot global pose                  | Random in 2D (ARP_utility_code.Simulation.trajectory_generator) | UWB anchors  (3)       | Gaussian UWB uncertainty on range      |         
| Geometric dilution of precision  heatmap  | Random in 2D (ARP_utility_code.Simulation.trajectory_generator) | UWB anchors (3)        | Gaussian UWB uncertainty on range      |
| UWB anchor localisation miss calibration  | Random in 2D (ARP_utility_code.Simulation.trajectory_generator) | UWB anchors (3)  | Gaussian UWB uncertainty on range                              |

## Cells 
| Cells | type | Content                                                                                                                   |
|-------|------|---------------------------------------------------------------------------------------------------------------------------| 
| 1     | md   | General introduction to GDOP and UWB anchor localisation                                                                  |
| 2     | py   | imports                                                                                                                   |
| 3     | py   | parameters for trajectory generation of robot 1 + trajectory generation + visualisation                                   |
| 4     | py | parameters for uwb anchors and uwb noise parameters + estimated UWB trajectory + visualisation   of GDOP over trajectory  |
| 5     | md | Explanation on GDOP                                                                                                       |
| 6     | py   | Calculation of heatmap of GDOP                                                                                            |
| 7     | py | parameters for noise on anchor location + UWB pose estimation + visualisation                                             |
| 8 | md | text to conclude and bring in reference previous part as global pose deteriorates the relative pose is affected strongly. |
