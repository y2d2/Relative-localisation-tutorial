import numpy as np

from src._2d_upf import UPFConnectedAgent_2D, TargetTrackingUKF_2D, _UPFParticleWrapper_2D


def test_2d_upf_initialization():
    """Test that UPFConnectedAgent_2D initializes correctly with no particles."""
    upf = UPFConnectedAgent_2D()
    
    assert len(upf.particles) == 0
    assert hasattr(upf, 'ha')
    assert upf.particle_type == _UPFParticleWrapper_2D


def test_2d_upf_particle_creation():
    """Test that UPFConnectedAgent_2D creates _UPFParticleWrapper_2D particles."""
    upf = UPFConnectedAgent_2D()
    upf.set_ukf_parameters(kappa=-1, alpha=1, beta=2)
    upf.n_altitude = 1
    upf.n_azimuth = 1
    upf.n_heading = 1
    
    particle = upf.create_particle()
    
    assert isinstance(particle, _UPFParticleWrapper_2D)
    assert hasattr(particle, 'rpea')
    assert isinstance(particle.rpea, TargetTrackingUKF_2D)


def test_2d_upf_with_initial_particles():
    """Test UPFConnectedAgent_2D with pre-created particles."""
    ukf = TargetTrackingUKF_2D()
    ukf.set_ukf_properties(alpha=1, beta=2, kappa=-1)
    
    t_j = np.array([5.0, 0.0, 0.0])
    sigma_t = np.array([0.1, 0.01, 0.001])
    ukf.set_initial_state(t_j, sigma_t)
    
    particle = _UPFParticleWrapper_2D(x_ha_0=np.zeros(4), weight=1.0)
    particle.rpea = ukf
    particle.get_states()
    
    upf = UPFConnectedAgent_2D(list_of_particles=[particle], x_ha_0=np.zeros(4))
    
    assert len(upf.particles) == 1
    assert isinstance(upf.particles[0], type(particle))


def test_2d_upf_particle_properties():
    """Test that particles have correct properties after creation."""
    upf = UPFConnectedAgent_2D(x_ha_0=np.zeros(4))
    upf.set_ukf_parameters(kappa=-1, alpha=1, beta=2)
    upf.n_altitude = 1
    upf.n_azimuth = 1
    upf.n_heading = 1
    
    particle = upf.create_particle()
    
    assert hasattr(particle, 'weight')
    assert hasattr(particle, 'likelihood')
    assert hasattr(particle, 't_si_sj')
    assert hasattr(particle, 'P_t_si_sj')
    
    assert isinstance(particle.t_si_sj, np.ndarray)
    assert particle.t_si_sj.shape == (4,)
    assert particle.P_t_si_sj.shape == (3, 3)
