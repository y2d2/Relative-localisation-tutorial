import numpy as np
from arp_utility_code.UtilityCode.Trajectories import Trajectory
from arp_utility_code.UtilityCode.Manifpy_helper import se3_from_sgal3 as _se3_from_sgal3


class OdometrySensor():
    def __init__(self, T_OR, v_RR0, time, velocity_bool=False):
        self.trajectory = Trajectory(T_OR, v_RR0, time)
        # Defines whether the odometry sensor receives velocity or acceleration inputs
        self.velocity_bool = velocity_bool


    def get_new_measurement(self, a, v, w, time):
        # It is assumed that v and w are given in the body frame of the robot.
        # Truth generation belongs to the caller; this method only measures and
        # integrates the measured odometry state.
        if a is None and v is None:
            print("Please provide either acceleration or velocity input.")
            return
        self.trajectory.do_step(a, v, w, time)
        return a, v, w, time


    def replay_trajectory_measurements(self, trajectory: Trajectory):
        T_OR = _se3_from_sgal3(trajectory.X_OR)
        v_RR0 = trajectory.X_R0.linearVelocity()
        self.trajectory = Trajectory(T_OR, v_RR0, trajectory.t[0])
        for a, v, w, t in trajectory.stream_body_frame_measurements():
            self.get_new_measurement(a, v, w, t)
        return self.trajectory


class Vio(OdometrySensor):
    def __init__(self, T_OR, v_RR0, time):
        super().__init__(T_OR, v_RR0, time, velocity_bool=True)

class Imu(OdometrySensor):
    def __init__(self,T_OR, v_RR0, time):
        super().__init__(T_OR, v_RR0, time, velocity_bool=False)

##################################################################################
###  INTER ROBOT DISTANCE SENSOR
##################################################################################
class InterRobotDistanceSensor():
    def __init__(self):
        self.t = []
        self.d = []

    def get_new_measurement(self, d,  time):
        self.t.append(time)
        self.d.append(d)
        return d

    def plot(self, ax, color="b", label="Measured UWB distance", linestyle="--", **kwargs):
        return ax.plot(self.t, self.d, color=color, linestyle=linestyle, label=label, **kwargs)

########################################################################
### GT sensor (Vicon, lidar-SLAM, ... )
########################################################################
class GTSensor():
    def __init__(self, T_OR, v_RR0, time):
        self.trajectory = Trajectory(T_OR, v_RR0, time)

    def get_new_measurement(self, t, q , time):
        self.trajectory.set_current_global_pose(t, q, time)
        return t, q, time

    def get_new_transformation_measurement(self, T, time):
        self.trajectory.set_current_global_transformation(T, time)
        return T, time
