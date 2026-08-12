import numpy as np

class uwb_noise_filter():
    def new_measurement(self, measurement, t)-> bool:
        pass


class Radial_vel_threshold(uwb_noise_filter):
    def __init__(self, threshold =0.5):
        self.last_measurement = None
        self.last_t = None
        self.threshold = threshold
        self.speed = None

    def new_measurement(self, measurement, t)-> bool:
        los = False
        if self.last_measurement is not None:
            dt = t - self.last_t
            self.speed = np.linalg.norm(measurement - self.last_measurement)/dt
            los = self.speed > self.threshold
        self.last_measurement = measurement
        self.last_t = t
        return los

class Radial_vel_threshold_variance(Radial_vel_threshold):
    def __init__(self, threshold=0.5, history=20):
        super(uwb_noise_filter, Radial_vel_threshold).__init__(threshold=threshold)
        self.history = history
        self.variance = None
        self.speeds = []

    def new_measurement(self, measurement, t)-> bool:
        los = False
        if self.last_measurement is not None:
            dt = t - self.last_t
            self.speed = np.linalg.norm(measurement - self.last_measurement) / dt
            self.speeds.append(self.speed)
            if len(self.speeds) > self.history:
                self.speeds.pop(0)
            self.variance = np.var(self.speeds)
            los = self.variance < self.threshold
        self.last_measurement = measurement
        self.last_t = t
        return los

class Variance_nlos_tracking(uwb_noise_filter):
    def __init__(self, history= 20, threshold =0.01):
        super(uwb_noise_filter, self).__init__()
        self.history = history
        self.threshold = threshold
        self.measurements = []

    def new_measurement(self, measurement, t)-> bool:
        self.measurements.append(measurement)
        if len(self.measurements) > self.history:
            self.measurements.pop(0)
        var = np.var(self.measurements)
        los  = var < self.threshold
        return los

