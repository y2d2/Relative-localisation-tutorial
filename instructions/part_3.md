# Part 3 — SLAM Global Localization: Accumulation of Uncertainty Through Drift

## General 
Read the project level AGENTS.md 
Make sure to understand the ARP_utility_code and manifpy libraries.
Make sure to reuse the code in src from previous parts as much as possible. 

## Excercises 
| Excercise                               | Assumptions Trajectory | Global pose estimation                 | Uncertainty                        |
|-----------------------------------------|-------------------------------------------|----------------------------------------|------------------------------------| 
| Single robot global pose  with SLAM     | Random in 2D (ARP_utility_code.Simulation.trajectory_generator) | SLAM odometry uncertainty accumulation | Gaussian noise on velocity and angular velocity |         
| Two robot relative pose with SLAM       | Random in 2D (ARP_utility_code.Simulation.trajectory_generator) | SLAM odometry uncertainty accumulation | Gaussian noise on velocity and angular velocity |

## Cells 
| Cells | type | Content                                                                                                                         |
|-------|------|---------------------------------------------------------------------------------------------------------------------------------| 
| 1     | md   | General introduction to SLAM uncertainty accumulation                                                                           |
| 2     | py   | imports                                                                                                                         |
| 3     | py   | parameters for trajectory generation of robot 1 + trajectory generation + visualisation                                         |
| 4     | py | parameters for trajectory generation of robot 2 + trajectory generation + visualisation  of relative pose and global pose errors |
| 5     | md | Conclusion on drift   |