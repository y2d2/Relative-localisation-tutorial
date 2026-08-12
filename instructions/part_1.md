# Part 1 — Global Localization (Perfect Situation)

## General 
Read the project level AGENTS.md 
Make sure to understand the ARP_utility_code and manifpy libraries.

## Excercises 
| Excercise             | Assumptions Trajectory | Global pose estimation| Uncertainty      |
|-----------------------|-------------------------------------------|-----------------------------------|-----------------------------------| 
| Single robot global pose ideal case | Random in 2D (ARP_utility_code.Simulation.trajectory_generator) |VICON like | None |
| Single robot global pose noisy case | Random in 2D (ARP_utility_code.Simulation.trajectory_generator) |VICON like | Gaussian noise on position and orientation |
| Two robots relative pose ideal case | Random in 2D (ARP_utility_code.Simulation.trajectory_generator) |VICON like | None |
| Two robots relative pose noisy case | Random in 2D (ARP_utility_code.Simulation.trajectory_generator) |VICON like | Gaussian noise on position and orientation |

## Cells 
| Cells | type | Content                                                                                                                    |
|-------|------|----------------------------------------------------------------------------------------------------------------------------| 
| 1     | md   | General introduction to localisation and relative localisation                                                             |
| 2     | py   | imports                                                                                                                    |
| 3     | py   | parameters for trajectory generation of robot 1 + trajectory generation + visualisation                                    |
| 4     | py | parameters for noise of robot 1 + noise generation + visualisation                                                         |
| 5     | md | Analysis of the effect of noise on the global pose estimation (limited)                                                    |
| 6     | py   | parameters for trajectory generation of robot 2 + trajectory generation + perfect relative pose estimation+  visualisation |
|7 | md | text on how in perfect case the relative pose can be computed from the global poses of both robots |
|8 | py | parameters for noise of robot 2 + noise generation + relative pose estimation + visualisation |
|9 | md | text on how in noisy case the relative pose can be computed from the global poses of both robots and how the noise propagates |

