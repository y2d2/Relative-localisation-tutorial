from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from manifpy import SGal3, SGal3Tangent


SGAL3_DOF = 10
POSITION_DIM = 3


@dataclass
class SGal3EKFState:
    """Snapshot of the manifold state and tangent covariance."""

    x: SGal3
    P: np.ndarray


class SGal3EKF:
    """Error-state EKF on ``manifpy.SGal3``.

    The nominal state is kept as an ``SGal3`` element. The covariance and
    Kalman correction live in the 10D tangent space with coefficient order:
    ``[px, py, pz, vx, vy, vz, wx, wy, wz, dt]``.

    Implementation Details (Error-State formulation):
    -------------------------------------------------
    This filter implements an Error-State Extended Kalman Filter (ES-EKF).
    The nominal state is stored in `self._state` (an `SGal3` manifold element),
    while the error state is tracked by the underlying `self._filter.x`.

    Because it is an error-state filter, the error state is intentionally
    reset to zero after every propagation and update step.

    1. During Prediction (`predict` / `_predict_step`):
       The manifold propagation is handled by `self._state = self._state.rplus(u, F, Ju)`.
       The filter's covariance (`self._filter.P`) is updated using the Jacobian `F`:
       `self._filter.P = F @ self._filter.P @ F.T + Ju @ Q @ Ju.T`.
       The error state `self._filter.x` is reset to zero.

    2. During Update (`update_position`):
       The Kalman update calculates the error correction in `self._filter.x`.
       This correction is applied to the nominal state:
       `self._state = self._state.rplus(SGal3Tangent(correction))`.
       The error state is then reset to zero, ready for the next iteration.

    The `self._filter.x` state appears "static" (mostly zero) because it only
    holds the local error during the measurement update. Once that error is
    injected into the manifold state `self._state`, the filter's state is cleared.
    The filter's covariance `self._filter.P` is updated continuously during
    both prediction and update steps.
    """

    def __init__(
        self,
        state: SGal3 | None = None,
        covariance: np.ndarray | None = None,
        process_noise: np.ndarray | None = None,
        measurement_noise: np.ndarray | None = None,
    ) -> None:
        self._state = self._copy_state(state if state is not None else SGal3.Identity())
        self._filter = ExtendedKalmanFilter(dim_x=SGAL3_DOF, dim_z=POSITION_DIM)
        self._filter.x = np.zeros((SGAL3_DOF, 1))
        self._filter.P = self._as_square_matrix(
            covariance if covariance is not None else np.eye(SGAL3_DOF),
            SGAL3_DOF,
            "covariance",
        )
        self._filter.Q = self._as_square_matrix(
            process_noise if process_noise is not None else np.eye(SGAL3_DOF),
            SGAL3_DOF,
            "process_noise",
        )
        self._filter.R = self._as_square_matrix(
            measurement_noise if measurement_noise is not None else np.eye(POSITION_DIM),
            POSITION_DIM,
            "measurement_noise",
        )
        self.x_prior = self._copy_state(self._state)
        self.x_post = self._copy_state(self._state)
        self.P_prior = self._filter.P.copy()
        self.P_post = self._filter.P.copy()
        self.last_predict_start = self._copy_state(self._state)
        self.last_predict_end = self._copy_state(self._state)
        self.last_u_total = SGal3Tangent.Zero()
        self.last_Q_total = np.zeros((SGAL3_DOF, SGAL3_DOF))
        self.last_predict_count = 0

    @property
    def x(self) -> SGal3:
        return self._state

    @property
    def P(self) -> np.ndarray:
        return self._filter.P

    @P.setter
    def P(self, value: np.ndarray) -> None:
        self._filter.P = self._as_square_matrix(value, SGAL3_DOF, "P")

    @property
    def Q(self) -> np.ndarray:
        return self._filter.Q

    @Q.setter
    def Q(self, value: np.ndarray) -> None:
        self._filter.Q = self._as_square_matrix(value, SGAL3_DOF, "Q")

    @property
    def R(self) -> np.ndarray:
        return self._filter.R

    @R.setter
    def R(self, value: np.ndarray) -> None:
        self._filter.R = self._as_square_matrix(value, POSITION_DIM, "R")

    @property
    def K(self) -> np.ndarray:
        return self._filter.K

    @property
    def y(self) -> np.ndarray:
        return self._filter.y

    @property
    def S(self) -> np.ndarray:
        return self._filter.S

    @property
    def filter(self) -> ExtendedKalmanFilter:
        """Expose the underlying FilterPy object for diagnostics."""

        return self._filter

    def predict(self, u: SGal3Tangent, Q: np.ndarray | None = None) -> SGal3:
        """Propagate the SGal3 state by the supplied tangent increment."""
        return self.predict_many([u], Q=Q)

    def predict_many(self, inputs, Q: np.ndarray | list[np.ndarray] | tuple[np.ndarray, ...] | None = None) -> SGal3:
        """Propagate through multiple tangent increments before an update."""

        increments = [self._as_tangent(u) for u in inputs]
        if not increments:
            raise ValueError("inputs must contain at least one SGal3 tangent increment")
        process_noises = self._process_noise_sequence(Q, len(increments))
        batch_start = self._copy_state(self._state)
        self.x_prior = self._copy_state(self._state)
        self.P_prior = self._filter.P.copy()
        Q_total = np.zeros((SGAL3_DOF, SGAL3_DOF))

        for u, Q_step in zip(increments, process_noises):
            F = self._predict_step(u, Q_step)
            Q_total = F @ Q_total @ F.T + Q_step

        self.last_predict_start = batch_start
        self.last_predict_end = self._copy_state(self._state)
        self.last_u_total = batch_start.between(self._state).log()
        self.last_Q_total = Q_total
        self.last_predict_count = len(increments)

        self.x_post = self._copy_state(self._state)
        self.P_post = self._filter.P.copy()
        return self._state

    def update(self, measurement: SGal3 | np.ndarray, R: np.ndarray | None = None) -> SGal3:
        """Alias for ``update_position``."""

        return self.update_position(measurement, R=R)

    def update_position(self, measurement: SGal3 | np.ndarray, R: np.ndarray | None = None) -> SGal3:
        """Update with a measurement where only position is observed."""

        self.x_prior = self._copy_state(self._state)
        self.P_prior = self._filter.P.copy()

        residual = self.residual_position(measurement).reshape(POSITION_DIM, 1)
        H = np.zeros((POSITION_DIM, SGAL3_DOF))
        H[:, :POSITION_DIM] = np.eye(POSITION_DIM)
        R_matrix = None if R is None else self._as_square_matrix(R, POSITION_DIM, "R")

        self._filter.update(
            residual,
            HJacobian=lambda _x: H,
            Hx=lambda _x: np.zeros((POSITION_DIM, 1)),
            R=R_matrix,
        )

        correction = self._filter.x.reshape(SGAL3_DOF)
        self._state = self._state.rplus(SGal3Tangent(correction))
        self._reset_error_state()

        self.x_post = self._copy_state(self._state)
        self.P_post = self._filter.P.copy()
        return self._state

    def residual_position(self, measurement: SGal3 | np.ndarray) -> np.ndarray:
        return self._position_from_measurement(measurement) - self.position()

    def position(self) -> np.ndarray:
        return np.array([self._state.x(), self._state.y(), self._state.z()], dtype=float)

    def velocity(self) -> np.ndarray:
        return np.array([self._state.vx(), self._state.vy(), self._state.vz()], dtype=float)

    def as_vector(self) -> np.ndarray:
        return self._state.coeffs_copy()

    def snapshot(self) -> SGal3EKFState:
        return SGal3EKFState(self._copy_state(self._state), self._filter.P.copy())

    def _reset_error_state(self) -> None:
        self._filter.x = np.zeros((SGAL3_DOF, 1))
        self._filter.x_prior = self._filter.x.copy()
        self._filter.x_post = self._filter.x.copy()

    def _predict_step(self, u: SGal3Tangent, Q: np.ndarray) -> np.ndarray:
        F = np.empty((SGAL3_DOF, SGAL3_DOF), dtype=float, order="C")
        Ju = np.empty((SGAL3_DOF, SGAL3_DOF), dtype=float, order="C")
        self._state = self._state.rplus(u, F, Ju)
        self._filter.F = F
        self._filter.P = F @ self._filter.P @ F.T + Ju @ Q @ Ju.T
        self._filter.P_prior = self._filter.P.copy()
        self._reset_error_state()
        return F

    def _process_noise_sequence(self, Q, count: int) -> list[np.ndarray]:
        if Q is None:
            return [self._filter.Q.copy() for _ in range(count)]
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
        return SGal3EKF._as_vector(measurement, POSITION_DIM, "measurement")

    @staticmethod
    def _as_vector(value: np.ndarray, size: int, name: str) -> np.ndarray:
        vector = np.asarray(value, dtype=float).reshape(-1)
        if vector.shape != (size,):
            raise ValueError(f"{name} must contain {size} values, got shape {np.asarray(value).shape}")
        return vector

    @staticmethod
    def _as_square_matrix(value: np.ndarray, size: int, name: str) -> np.ndarray:
        matrix = np.asarray(value, dtype=float)
        if matrix.shape != (size, size):
            raise ValueError(f"{name} must have shape {(size, size)}, got {matrix.shape}")
        return matrix.copy()
