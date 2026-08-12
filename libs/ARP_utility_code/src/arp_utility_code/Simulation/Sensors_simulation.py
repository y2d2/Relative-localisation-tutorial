from arp_utility_code.UtilityCode.Sensors import InterRobotDistanceSensor, OdometrySensor
import numpy as np
from manifpy import SGal3


########################################################################
# %% Odometry sensor simulation (IMU, VIO, ...)
########################################################################
class OdometrySensorSimulation(OdometrySensor):
    def sensor_measurement(self, a, v, w, time):
        print("Please create the Sensor specific behavior.")
        a_odom = a
        w_odom = w
        v_odom = v
        return a_odom, v_odom, w_odom

    def get_new_measurement(self, a, v, w, time):
        a_odom, v_odom, w_odom = self.sensor_measurement(a, v, w, time)
        return super().get_new_measurement(a_odom, v_odom, w_odom, time)

class VioSimulation(OdometrySensorSimulation):
    def __init__(self, T_OR, v_RR0, time, sig_v, sig_w):
        super().__init__(T_OR, v_RR0, time, velocity_bool=True)
        self.sig_v = sig_v
        self.sig_w = sig_w

    def sensor_measurement(self, a, v, w, time):
        v_noise = v + np.random.randn(3) * self.sig_v
        w_noise = w + np.random.randn(3) * self.sig_w
        return None, v_noise, w_noise

class ImuSimulation(OdometrySensorSimulation):
    def __init__(
            self,
            T_OR,
            v_RR0,
            time,
            sig_w,
            sig_a,
            tau_ba,
            tau_bw,
            gravity_vector=None,
    ):
        super().__init__(T_OR, v_RR0, time, velocity_bool=False)

        # IMU noise parameters
        self.sig_w = sig_w
        self.sig_a = sig_a
        if gravity_vector is None:
            gravity_vector = np.array([0.0, 0.0, -9.81])
        self.gravity_vector = np.asarray(gravity_vector, dtype=float)

        # Slow chanign biases
        self.tau_ba = tau_ba
        self.tau_bw = tau_bw

        # Randomised bias for IMU. Zero bias noise gives deterministic pass-through behavior.
        self.b_acc = [np.random.randn(3) * self.tau_ba]
        self.b_gyro = [np.random.randn(3) * self.tau_bw]

    def sensor_measurement(self, a, v, w, time):
        a_imu = a + self.gravity_vector + self.b_acc[-1] + np.random.randn(3) * self.sig_a
        w_imu = w + self.b_gyro[-1] + np.random.randn(3) * self.sig_w
        # random walk of biases:
        self.b_acc.append(self.b_acc[-1] + np.random.randn(3) * self.tau_ba)
        self.b_gyro.append(self.b_gyro[-1] + np.random.randn(3) * self.tau_bw)
        return a_imu, None, w_imu


class Imu2DSimulation(ImuSimulation):
    def sensor_measurement(self, a, v, w, time):
        a_imu, _, w_imu = super().sensor_measurement(a, v, w, time)
        a_imu[2] = 0
        w_imu[0:2] = 0
        return a_imu, None, w_imu

class Vio2DSimulation(VioSimulation):
    def sensor_measurement(self, a, v, w, time):
        _, v_imu, w_imu = super().sensor_measurement(a, v, w, time)
        v_imu[2] = 0
        w_imu[0:2] = 0
        return None, v_imu, w_imu


########################################################################
# %% Interdistance sensor simulation (UWB, ... )
########################################################################
class InterRobotDistanceSensorSimulation(InterRobotDistanceSensor):
    def __init__(self, sig_d):
        super().__init__()
        self.sig_d = sig_d

    def get_new_measurement(self, d, time):
        d_noise = d + np.random.randn() * self.sig_d
        return super().get_new_measurement(d_noise, time)

    def get_new_measurment_from_SGAL3(self, X_ORi: SGal3, X_ORj: SGal3, time):
        X_RiRj = X_ORi.inverse().compose(X_ORj)
        d = np.linalg.norm(X_RiRj.translation())
        return self.get_new_measurement(d, time)
########################################################################
# %% GT sensor simulation (Vicon, lidar-SLAM, ... )
########################################################################
class GTfromOdomSensorSimulation(OdometrySensorSimulation):
    def sensor_measurement(self, a, v, w, time):
        return a, v, w
