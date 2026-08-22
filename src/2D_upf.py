from upf_rte.ParticleFilter.TargetTrackingUKF import TargetTrackingUKF
from upf_rte.ParticleFilter.ConnectedAgentClass import UPFConnectedAgent

# oc-todo: Make simple adjustment so that the UKF who works in 3D 4-dof, now works in 2D 3-dof.
    # The goal is not to remake all the states, but to adjust the values given to the UKF particles such that the value is constant and has small unceetainty.
    # all z-values are zero.
    # First deeply analyse the TargetTrakingUKF and UPFConnectedAgent to understand how they work.
