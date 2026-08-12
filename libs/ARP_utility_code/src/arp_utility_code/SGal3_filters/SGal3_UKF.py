from __future__ import annotations

from collections.abc import Callable, Iterable
from copy import deepcopy

import numpy as np
from filterpy.kalman import MerweScaledSigmaPoints, UnscentedKalmanFilter
from manifpy import SGal3, SGal3Tangent


SGAL3_DOF = 10
POSITION_DIM = 3


class SGal3UKF(UnscentedKalmanFilter):
    """Unscented Kalman filter with an SGal3 nominal state.

    The public ``x`` is a ``manifpy.SGal3``. Sigma points, covariance, and
    corrections live in the 10D tangent space with coefficient order:
    ``[px, py, pz, vx, vy, vz, wx, wy, wz, dt]``.
    """

    def __init__(
        self,
        dim_z: int,
        hx: Callable[[SGal3], np.ndarray],
        state: SGal3 | None = None,
        covariance: np.ndarray | None = None,
        process_noise: np.ndarray | None = None,
        measurement_noise: np.ndarray | None = None,
        alpha: float = 0.1,
        beta: float = 2.0,
        kappa: float = 0.0,
        z_mean_fn: Callable[[np.ndarray, np.ndarray], np.ndarray] | None = None,
        residual_z: Callable[[np.ndarray, np.ndarray], np.ndarray] | None = None,
    ) -> None:
        points = MerweScaledSigmaPoints(
            SGAL3_DOF,
            alpha=alpha,
            beta=beta,
            kappa=kappa,
            subtract=np.subtract,
        )
        super().__init__(
            dim_x=SGAL3_DOF,
            dim_z=dim_z,
            dt=1.0,
            hx=lambda x: x,
            fx=lambda x, dt: x,
            points=points,
            z_mean_fn=z_mean_fn,
            residual_z=residual_z,
        )
        self.hx = hx
        self.x = self._copy_state(state if state is not None else SGal3.Identity())
        self.P = self._as_square_matrix(
            covariance if covariance is not None else np.eye(SGAL3_DOF),
            SGAL3_DOF,
            "covariance",
        )
        self.Q = self._as_square_matrix(
            process_noise if process_noise is not None else np.eye(SGAL3_DOF),
            SGAL3_DOF,
            "process_noise",
        )
        self.R = self._as_square_matrix(
            measurement_noise if measurement_noise is not None else np.eye(dim_z),
            dim_z,
            "measurement_noise",
        )
        self.x_prior = self._copy_state(self.x)
        self.x_post = self._copy_state(self.x)
        self.P_prior = self.P.copy()
        self.P_post = self.P.copy()
        self.sigmas_f = []
        self.last_predict_start = self._copy_state(self.x)
        self.last_predict_end = self._copy_state(self.x)
        self.last_u_total = SGal3Tangent.Zero()
        self.last_Q_total = np.zeros((SGAL3_DOF, SGAL3_DOF))
        self.last_predict_count = 0

    @classmethod
    def position_filter(
        cls,
        state: SGal3 | None = None,
        covariance: np.ndarray | None = None,
        process_noise: np.ndarray | None = None,
        measurement_noise: np.ndarray | None = None,
        **kwargs,
    ) -> "SGal3UKF":
        return cls(
            dim_z=POSITION_DIM,
            hx=lambda x: np.array([x.x(), x.y(), x.z()], dtype=float),
            state=state,
            covariance=covariance,
            process_noise=process_noise,
            measurement_noise=measurement_noise,
            **kwargs,
        )

    @property
    def filter(self) -> "SGal3UKF":
        """Expose a FilterPy-like diagnostic object."""

        return self

    def predict(self, u: SGal3Tangent | np.ndarray, Q: np.ndarray | None = None, **kwargs) -> SGal3:
        """Propagate the filter by one SGal3 tangent increment."""

        return self.predict_many([u], Q=Q)

    def predict_many(
        self,
        inputs: Iterable[SGal3Tangent | np.ndarray],
        Q: np.ndarray | list[np.ndarray] | tuple[np.ndarray, ...] | None = None,
    ) -> SGal3:
        increments = [self._as_tangent(u) for u in inputs]
        if not increments:
            raise ValueError("inputs must contain at least one SGal3 tangent increment")
        process_noises = self._process_noise_sequence(Q, len(increments))

        batch_start = self._copy_state(self.x)
        self.x_prior = self._copy_state(self.x)
        self.P_prior = self.P.copy()
        Q_total = np.zeros((SGAL3_DOF, SGAL3_DOF))

        for u, Q_step in zip(increments, process_noises):
            sigma_states = self._sigma_states(self.x, self.P)
            propagated = [sigma.rplus(u) for sigma in sigma_states]
            predicted_mean = self._state_mean(propagated, initial=self.x.rplus(u))
            self.P = self._state_covariance(predicted_mean, propagated, Q_step)
            self.x = predicted_mean
            self.sigmas_f = propagated
            Q_total = self._transition_covariance(sigma_states, propagated, Q_total, Q_step)

        self.last_predict_start = batch_start
        self.last_predict_end = self._copy_state(self.x)
        self.last_u_total = batch_start.between(self.x).log()
        self.last_Q_total = Q_total
        self.last_predict_count = len(increments)
        self.x_post = self._copy_state(self.x)
        self.P_post = self.P.copy()
        return self.x

    def update(self, z, R: np.ndarray | None = None, hx: Callable[[SGal3], np.ndarray] | None = None, **kwargs) -> SGal3:
        """Update with a caller-supplied measurement model on SGal3 states."""

        if z is None:
            self.z = np.array([[None] * self._dim_z]).T
            self.x_post = self._copy_state(self.x)
            self.P_post = self.P.copy()
            return self.x

        measurement = self._as_vector(z, self._dim_z, "z")
        R_matrix = self.R if R is None else self._as_square_matrix(R, self._dim_z, "R")
        measurement_fn = self.hx if hx is None else hx
        if not self.sigmas_f:
            self.sigmas_f = self._sigma_states(self.x, self.P)

        sigmas_h = np.asarray([self._as_vector(measurement_fn(sigma), self._dim_z, "hx(state)") for sigma in self.sigmas_f])
        z_pred = self._measurement_mean(sigmas_h)
        self.S = self._measurement_covariance(z_pred, sigmas_h, R_matrix)
        self.SI = self.inv(self.S)
        Pxz = self._cross_covariance(self.x, z_pred, self.sigmas_f, sigmas_h)
        self.K = Pxz @ self.SI
        self.y = self.residual_z(measurement, z_pred)
        correction = self.K @ self.y

        self.x = self.x.rplus(SGal3Tangent(correction))
        self.P = self._symmetrize(self.P - self.K @ self.S @ self.K.T)
        self.z = deepcopy(measurement)
        self.x_post = self._copy_state(self.x)
        self.P_post = self.P.copy()
        self.sigmas_f = self._sigma_states(self.x, self.P)
        self._log_likelihood = None
        self._likelihood = None
        self._mahalanobis = None
        return self.x

    def update_position(self, measurement: SGal3 | np.ndarray, R: np.ndarray | None = None) -> SGal3:
        return self.update(self._position_from_measurement(measurement), R=R)

    def position(self) -> np.ndarray:
        return np.array([self.x.x(), self.x.y(), self.x.z()], dtype=float)

    def velocity(self) -> np.ndarray:
        return np.array([self.x.vx(), self.x.vy(), self.x.vz()], dtype=float)

    def as_vector(self) -> np.ndarray:
        return self.x.coeffs_copy()

    def _sigma_states(self, mean: SGal3, covariance: np.ndarray) -> list[SGal3]:
        sigma_vectors = self.points_fn.sigma_points(np.zeros(SGAL3_DOF), covariance)
        return [mean.rplus(SGal3Tangent(sigma)) for sigma in sigma_vectors]

    def _state_mean(self, sigma_states: list[SGal3], initial: SGal3 | None = None) -> SGal3:
        mean = self._copy_state(initial if initial is not None else sigma_states[0])
        for _ in range(20):
            delta = np.zeros(SGAL3_DOF)
            for weight, sigma in zip(self.Wm, sigma_states):
                delta += weight * mean.between(sigma).log().coeffs()
            if np.linalg.norm(delta) < 1e-12:
                break
            mean = mean.rplus(SGal3Tangent(delta))
        return mean

    def _state_covariance(self, mean: SGal3, sigma_states: list[SGal3], noise: np.ndarray) -> np.ndarray:
        covariance = noise.copy()
        for weight, sigma in zip(self.Wc, sigma_states):
            residual = mean.between(sigma).log().coeffs()
            covariance += weight * np.outer(residual, residual)
        return self._symmetrize(covariance)

    def _measurement_mean(self, sigmas_h: np.ndarray) -> np.ndarray:
        if self.z_mean is not None:
            return self.z_mean(sigmas_h, self.Wm)
        return np.sum(self.Wm[:, None] * sigmas_h, axis=0)

    def _measurement_covariance(self, z_mean: np.ndarray, sigmas_h: np.ndarray, noise: np.ndarray) -> np.ndarray:
        covariance = noise.copy()
        for weight, sigma_h in zip(self.Wc, sigmas_h):
            residual = self.residual_z(sigma_h, z_mean)
            covariance += weight * np.outer(residual, residual)
        return self._symmetrize(covariance)

    def _cross_covariance(
        self,
        state_mean: SGal3,
        z_mean: np.ndarray,
        sigma_states: list[SGal3],
        sigmas_h: np.ndarray,
    ) -> np.ndarray:
        covariance = np.zeros((SGAL3_DOF, self._dim_z))
        for weight, sigma_state, sigma_h in zip(self.Wc, sigma_states, sigmas_h):
            dx = state_mean.between(sigma_state).log().coeffs()
            dz = self.residual_z(sigma_h, z_mean)
            covariance += weight * np.outer(dx, dz)
        return covariance

    def _transition_covariance(
        self,
        sigma_states: list[SGal3],
        propagated: list[SGal3],
        prior_covariance: np.ndarray,
        noise: np.ndarray,
    ) -> np.ndarray:
        previous_mean = self._state_mean(sigma_states)
        predicted_mean = self._state_mean(propagated)
        transition = np.zeros((SGAL3_DOF, SGAL3_DOF))
        previous_covariance = np.zeros((SGAL3_DOF, SGAL3_DOF))
        for weight, before, after in zip(self.Wc, sigma_states, propagated):
            dx_before = previous_mean.between(before).log().coeffs()
            dx_after = predicted_mean.between(after).log().coeffs()
            transition += weight * np.outer(dx_after, dx_before)
            previous_covariance += weight * np.outer(dx_before, dx_before)
        if np.linalg.matrix_rank(previous_covariance) == SGAL3_DOF:
            F = transition @ np.linalg.inv(previous_covariance)
            return self._symmetrize(F @ prior_covariance @ F.T + noise)
        return self._symmetrize(prior_covariance + noise)

    def _process_noise_sequence(self, Q, count: int) -> list[np.ndarray]:
        if Q is None:
            return [self.Q.copy() for _ in range(count)]
        if isinstance(Q, np.ndarray) and Q.shape == (SGAL3_DOF, SGAL3_DOF):
            Q_step = self._as_square_matrix(Q, SGAL3_DOF, "Q")
            return [Q_step.copy() for _ in range(count)]
        process_noises = [self._as_square_matrix(value, SGAL3_DOF, "Q") for value in Q]
        if len(process_noises) != count:
            raise ValueError(f"Q must contain {count} process-noise matrices, got {len(process_noises)}")
        return process_noises

    def _as_tangent(self, value) -> SGal3Tangent:
        if isinstance(value, SGal3Tangent):
            return value
        return SGal3Tangent(self._as_vector(value, SGAL3_DOF, "u"))

    @staticmethod
    def _copy_state(state: SGal3) -> SGal3:
        return SGal3(state.coeffs_copy())

    @staticmethod
    def _position_from_measurement(measurement: SGal3 | np.ndarray) -> np.ndarray:
        if isinstance(measurement, SGal3):
            return np.array([measurement.x(), measurement.y(), measurement.z()], dtype=float)
        return SGal3UKF._as_vector(measurement, POSITION_DIM, "measurement")

    @staticmethod
    def _as_vector(value, size: int, name: str) -> np.ndarray:
        vector = np.asarray(value, dtype=float).reshape(-1)
        if vector.shape != (size,):
            raise ValueError(f"{name} must contain {size} values, got shape {np.asarray(value).shape}")
        return vector

    @staticmethod
    def _as_square_matrix(value, size: int, name: str) -> np.ndarray:
        matrix = np.asarray(value, dtype=float)
        if matrix.shape != (size, size):
            raise ValueError(f"{name} must have shape {(size, size)}, got {matrix.shape}")
        return matrix.copy()

    @staticmethod
    def _symmetrize(matrix: np.ndarray) -> np.ndarray:
        return 0.5 * (matrix + matrix.T)
