import copy
import numpy as np
from typing import List

from upf_rte.ParticleFilter.TargetTrackingUKF import TargetTrackingUKF
from upf_rte.ParticleFilter.TargetTrackingParticle import TargetTrackingParticle
from upf_rte.ParticleFilter.ConnectedAgentClass import UPFConnectedAgent


class _UPFParticleWrapper_2D(TargetTrackingParticle):
    """
    Particle wrapper for UPFConnectedAgent_2D that uses TargetTrackingUKF_2D as the RPEA.
    
    Adapts the particle interface to work with 2D UKF (3-state: [r, θ, h] instead of [r, θ, φ, h]).
    """

    def __init__(self, x_ha_0=np.zeros(4), weight=1., drift_correction_bool=True, parent=None):
        super().__init__(weight, parent)
        self.rpea: TargetTrackingUKF_2D = TargetTrackingUKF_2D(x_ha_0, weight, drift_correction_bool, parent)
        self.drift_correction_bool = drift_correction_bool
        self.get_states()

    def run_model(self, dt_i, q_i, t_i, P_i, dt_j, q_j, d_ij, sig_uwb, time_i):
        self.rpea.run_filter(dt_j, q_j, t_i, P_i, d_ij, sig_uwb, 
                           self.drift_correction_bool, True, time_i)
        self.likelihood = self.rpea.kf.likelihood
        self.weight = self.weight * self.likelihood
        self.get_states()

    def get_states(self):
        self.t_si_sj = self.rpea.t_si_sj
        self.P_t_si_sj = self.rpea.P_t_si_sj

    def copy(self):
        rpea_copy = self.rpea.copy()
        return _UPFParticleWrapper_2D(x_ha_0=self.rpea.kf.x, weight=self.weight,
                                      drift_correction_bool=self.drift_correction_bool)


class TargetTrackingUKF_2D(TargetTrackingUKF):
    """
    Wrapper for TargetTracking UKF that constrains it to 2D operation (z=0 plane).
    
    Forces phi_0=0 and z_odom=0 with minimal uncertainty (1e-6) to make UKF behave
    as 2D 3-dof system instead of full 3D 4-dof.
    """

    def __init__(self, x_ha_0=np.zeros(4), weight=1., drift_correction_bool=True, parent=None):
        super().__init__(x_ha_0, weight, drift_correction_bool, parent)
        
        self._t_si_sj_2d = None
        
    def _to_2d(self, arr_4d):
        """Convert 4D [r, theta, phi, h] to 2D [r, theta, h]."""
        if arr_4d is None:
            return None
        if len(arr_4d) == 4 or hasattr(arr_4d, '__len__') and arr_4d.shape[0] == 4:
            return np.array([arr_4d[0], arr_4d[1], arr_4d[3]])
        return arr_4d

    def set_initial_state(self, t_j, sigma_t):
        """
        Convert 2D input [r, theta, h] to UKF's expected format [r, θ, φ=0, h].
        """
        t_j_4d = np.array([t_j[0], t_j[1], 0.0, t_j[2]])
        sigma_t_4d = np.array([sigma_t[0], sigma_t[1], 1e-6, sigma_t[2]])
        
        super().set_initial_state(t_j_4d, sigma_t_4d)
        
        self.kf.P[2, 2] = (1e-6) ** 2

    def run_filter(self, dt_j, q_j, t_i, P_i, d_ij, sig_d, bool_drift=True, update_bool=True, time_i=None):
        super().run_filter(dt_j, q_j, t_i, P_i, d_ij, sig_d, bool_drift, update_bool, time_i)
        
    def calculate_x_ca(self):
        super().calculate_x_ca()
        if hasattr(self, 't_si_sj') and self.t_si_sj is not None and len(self.t_si_sj) >= 4:
            self._t_si_sj_2d = np.array([self.t_si_sj[0], self.t_si_sj[1], self.t_si_sj[3]])

    def calculate_P_x_ca(self):
        super().calculate_P_x_ca()
        
        self.P_t_si_sj = np.zeros((3, 3))
        if hasattr(self, 'kf') and self.kf is not None:
            stds = np.sqrt(np.diag(self.kf.P))
            self.P_t_si_sj[0, 0] = stds[0] ** 2
            self.P_t_si_sj[1, 1] = stds[1] ** 2  
            self.P_t_si_sj[2, 2] = self.kf.P[-2, -2]

    def __getattribute__(self, name):
        val = super().__getattribute__(name)
        
        if name == 't_si_sj' and super().__getattribute__('_t_si_sj_2d') is not None:
            return super().__getattribute__('_t_si_sj_2d')
            
        if name in ('sigma_x_ca_0', 'sigma_x_ca', 'sigma_h_ca'):
            stds = np.sqrt(np.diag(super().__getattribute__('kf').P))
            if name == 'sigma_x_ca_0':
                return stds[0]
            elif name == 'sigma_x_ca':
                return np.sqrt(stds[0] ** 2 + np.linalg.norm(super().__getattribute__('kf').P[4:-2, 4:-2]))
            elif name == 'sigma_h_ca':
                return np.sqrt(super().__getattribute__('kf').P[-2, -2])
                
        if name == 'P_t_si_sj':
            p = super().__getattribute__('P_t_si_sj')
            if p is not None and len(p) >= 4:
                result = np.zeros((3, 3))
                result[0, 0] = p[0, 0]
                result[1, 1] = p[1, 1]
                result[2, 2] = p[-1, -1]
                return result
            return p
        
        return val


class UPFConnectedAgent_2D(UPFConnectedAgent):
    """
    2D version of UPFConnectedAgent that uses TargetTrackingUKF_2D as particles.
    
    Inherits all functionality from UPFConnectedAgent but creates _UPFParticleWrapper_2D
    particles instead of regular UKF particles. The 2D UKF handles all 3D↔2D conversions internally.
    """

    def __init__(self, list_of_particles=None, x_ha_0=np.zeros(4), drift_correction_bool=True,
                 sigma_uwb=0.1, sigma_uwb_factor=1.0, resample_factor=0.1, id="0x000"):
        if list_of_particles is None:
            list_of_particles = []
        super().__init__(list_of_particles, x_ha_0, drift_correction_bool,
                        sigma_uwb, sigma_uwb_factor, resample_factor, id)
        
        self.particle_type = _UPFParticleWrapper_2D

    def create_particle(self):
        weight = 1. / self.n_azimuth / self.n_altitude / self.n_heading
        particle = self.particle_type(x_ha_0=self.ha.x_ha_0, weight=weight,
                                      drift_correction_bool=self.drift_correction_bool)
        particle.rpea.set_uwb_extrinsicity(self.t_si_uwb, self.t_sj_uwb)
        particle.rpea.set_ukf_properties(self.kappa, self.alpha, self.beta)
        return particle


# oc-todo: Create a new UPFconnectedAgent that use the TargetTrackingUKF_2D as particle. Keep it as simple as possible.