"""Kinematic ACC MPC controller — adas-exact formulation.

Plant model is purely kinematic (no mass / friction): only acceleration lag
with a single time constant τ. Control input is `a_cmd` (acceleration
command); the published velocity command is integrated by the ROS node
(strategy "B2" — see docs).

State (5 dimensions):
    x = [dx_err, v, v_rel, a, j]
        dx_err = dx − d0          (offset-free formulation)

Control:
    u = a_cmd                     (acceleration command, [m/s²])

Continuous-time dynamics (after Euler with step ts):
    dx_err_next   = dx_err + ts·v_rel  − ½·ts²·a       (analytical)
    v_next        = v + ts·a
    v_rel_next    = v_rel − ts·a       (peer CV over horizon)
    a_next        = (1 − ts/τ)·a + (ts/τ)·u            (1st-order lag)
    j_next        = −(1/τ)·a + (1/τ)·u                 (= (u−a)/τ, observed jerk)

The `j` row has NO dependency on previous `j` — adas keeps `j` in the state
vector solely so that jerk² can be put directly into the cost function.

Output for cost:
    y = [dx_err − th·v, v_rel, a, j]                   (n_out = 4)

Constraints:
    a_cmd ∈ [a_min, a_max]                            (input bound)
    dx ≥ gap_safe                                     (hard collision avoidance)

Compared to the Newton-MPC variant (`SwarmAccController`):
  - Fewer parameters: only τ instead of (m, b, α, τ_F).
  - Easier identification: a single step-response experiment.
  - No physical interpretation of force / mass — purely an acceleration
    tracker. Less expressive but more robust to plant mismatch.
"""

import numpy as np
from scipy.sparse import csc_matrix
import osqp


class SwarmKinAccController:
    """Kinematic ACC MPC for a swarm follower (adas-style 5-state)."""

    n_in = 5   # state: [dx_err, v, v_rel, a, j]
    n_out = 4  # output: [y_gap, v_rel, a, j]

    def __init__(
        self,
        tau: float,
        d0: float, th: float,
        ts: float, p: int, c: int, s: float,
        phi_vals, q_vals,
        u_limits,
        gap_safe: float = 0.0,
    ):
        self.tau = float(tau)
        self.d0 = float(d0)
        self.th = float(th)
        self.ts = float(ts)
        self.p = int(p)
        self.c = int(c)
        self.s = float(s)
        self.u_min, self.u_max = float(u_limits[0]), float(u_limits[1])
        self.gap_safe = float(gap_safe)

        n_in, n_out, p_, c_ = self.n_in, self.n_out, self.p, self.c

        phi = np.asarray(phi_vals, dtype=float)
        q   = np.asarray(q_vals,   dtype=float)
        assert phi.shape == (n_out,), f"phi_vals must have length {n_out}"
        assert q.shape == (n_out,), f"q_vals must have length {n_out}"
        assert c_ <= p_, "control horizon c must not exceed prediction horizon p"

        # output matrix C (4×5):  y = C·x   (no Z thanks to dx_err state)
        #  state cols:    dx_err  v   v_rel  a   j
        C = np.zeros((n_out, n_in))
        C[0, 0] = 1.0;   C[0, 1] = -self.th     # y_gap = dx_err − th·v
        C[1, 2] = 1.0                            # v_rel
        C[2, 3] = 1.0                            # a
        C[3, 4] = 1.0                            # j
        self.C = C

        self.H = np.eye(n_in)
        self.F_mat = C @ self.H

        # constant dynamics
        self.A, self.B = self._build_dynamics()

        # stacked error-correction matrix over horizon
        self.F_hat = np.tile(self.F_mat, (p_, 1))

        # reference shaping FA: y_ref(k+i) = Phi^i · y(k)
        Phi = np.diag(phi)
        FA = np.zeros((p_ * n_out, n_out))
        Phi_pow = np.eye(n_out)
        for i in range(p_):
            Phi_pow = Phi_pow @ Phi
            FA[i * n_out:(i + 1) * n_out, :] = Phi_pow
        self.FA = FA

        # block-diagonal Q over horizon
        Qblock = np.diag(q)
        Qmat = np.zeros((p_ * n_out, p_ * n_out))
        for i in range(p_):
            Qmat[i * n_out:(i + 1) * n_out, i * n_out:(i + 1) * n_out] = Qblock
        self.Qmat = Qmat

        # state-prediction stacks A_hat, C_hat
        A_hat = np.zeros((p_ * n_in, n_in))
        C_hat = np.zeros((p_ * n_out, n_in))
        A_pow = np.eye(n_in)
        for i in range(p_):
            A_pow = A_pow @ self.A
            A_hat[i * n_in:(i + 1) * n_in, :] = A_pow
            C_hat[i * n_out:(i + 1) * n_out, :] = self.C @ A_pow
        self.A_hat = A_hat
        self.C_hat = C_hat

        # impulse response blocks C·A^k·B, k = 0..p−1
        CAkB = np.zeros((p_, n_out))
        A_pow = np.eye(n_in)
        for k in range(p_):
            CAkB[k] = self.C @ A_pow @ self.B
            A_pow = A_pow @ self.A
        # convolution matrix D_hat (p·n_out × c)
        D_hat = np.zeros((p_ * n_out, c_))
        for i in range(p_):
            for m in range(min(i + 1, c_)):
                k = i - m
                D_hat[i * n_out:(i + 1) * n_out, m] = CAkB[k]
        self.D_hat = D_hat

        # selector for dx_err state (index 0) — for safety constraint
        e_dx = np.zeros(n_in); e_dx[0] = 1.0
        M_free_dx = A_hat[0::n_in, :]
        e_AkB_dx = np.zeros(p_)
        A_pow = np.eye(n_in)
        for k in range(p_):
            e_AkB_dx[k] = float(e_dx @ A_pow @ self.B)
            A_pow = A_pow @ self.A
        D_dx = np.zeros((p_, c_))
        for i in range(p_):
            for m in range(min(i + 1, c_)):
                k = i - m
                D_dx[i, m] = e_AkB_dx[k]
        self.M_free_dx = M_free_dx
        self.D_dx = D_dx

        # move-suppression Hessian H1 (penalty on Δu)
        H1 = np.zeros((c_, c_))
        for i in range(c_):
            H1[i, i] = self.s if i == c_ - 1 else 2.0 * self.s
            if i > 0:
                H1[i, i - 1] = -self.s
            if i < c_ - 1:
                H1[i, i + 1] = -self.s
        self.H1 = H1

        # M2 (linear coupling u_prev → u_0)
        self.M2 = np.zeros(c_)
        self.M2[0] = self.s

        # CONSTANT QP Hessian
        Hqp = self.H1 + self.D_hat.T @ self.Qmat @ self.D_hat
        Hqp = 0.5 * (Hqp + Hqp.T)
        self.Hqp = Hqp

        # state predictor + warm start
        self._x_predicted = None
        self._u_prev = 0.0

        self._prob = osqp.OSQP()
        self._initialized = False

    def _build_dynamics(self):
        """Build A (5×5) and B (5×1) — adas-exact Euler discretisation."""
        ts, tau = self.ts, self.tau
        A = np.array([
            #  dx_err  v    v_rel    a              j
            [   1.0,   0.0, ts,     -0.5 * ts * ts, 0.0 ],   # dx_err: + ½·ts²·a (analytical)
            [   0.0,   1.0, 0.0,     ts,            0.0 ],   # v
            [   0.0,   0.0, 1.0,    -ts,            0.0 ],   # v_rel  (peer CV)
            [   0.0,   0.0, 0.0,     1.0 - ts/tau,  0.0 ],   # a       (1st-order to u)
            [   0.0,   0.0, 0.0,    -1.0/tau,       0.0 ],   # j       (= (u−a)/τ)
        ])
        B = np.array([0.0, 0.0, 0.0, ts/tau, 1.0/tau])
        return A, B

    def calculate_control(self, dx: float, v: float, v_rel: float):
        """Solve one MPC step.

        Args:
            dx     — current distance to peer [m]
            v      — own longitudinal speed   [m/s]   (from odom)
            v_rel  — peer_v − v               [m/s]   (from telemetry)

        Returns:
            (a_cmd, y) — optimal acceleration command and current output vector
                         [y_gap, v_rel, a_pred, j_pred]
        """
        n_in, c_, p_ = self.n_in, self.c, self.p

        if self._x_predicted is not None:
            a_est = float(self._x_predicted[3])
            j_est = float(self._x_predicted[4])
        else:
            a_est = 0.0
            j_est = 0.0

        dx_err = dx - self.d0
        x = np.array([dx_err, v, v_rel, a_est, j_est])

        ex = (x - self._x_predicted) if self._x_predicted is not None else np.zeros(n_in)

        y  = self.C @ x
        Yr = self.FA @ y
        F2 = self.C_hat @ x + self.F_hat @ ex
        b1 = Yr - F2

        g = -self.M2 * self._u_prev - self.D_hat.T @ self.Qmat @ b1

        # ── constraints ─────────────────────────────────────────────────────
        # 1. input bound:  a_min ≤ u ≤ a_max
        # 2. SAFETY (if gap_safe > 0):
        #      dx_err_k+i ≥ gap_safe − d0   over horizon
        rows = [np.eye(c_)]
        lb_parts = [np.full(c_, self.u_min)]
        ub_parts = [np.full(c_, self.u_max)]

        if self.gap_safe > 0:
            dx_err_free = self.M_free_dx @ x
            rows.append(self.D_dx)
            lb_parts.append(np.full(p_, self.gap_safe - self.d0) - dx_err_free)
            ub_parts.append(np.full(p_, np.inf))

        A_const = np.vstack(rows)
        u_lb = np.concatenate(lb_parts)
        u_ub = np.concatenate(ub_parts)

        if not self._initialized:
            P_sparse = csc_matrix(self.Hqp)
            A_sparse = csc_matrix(A_const)
            self._prob.setup(
                P=P_sparse, q=g,
                A=A_sparse, l=u_lb, u=u_ub,
                verbose=False, time_limit=self.ts * 0.8,
                warm_starting=False,
            )
            self._initialized = True
        else:
            self._prob.update(q=g, l=u_lb, u=u_ub)

        result = self._prob.solve()
        if result.info.status not in ('solved', 'solved_inaccurate'):
            # infeasible (e.g. peer braked harder than we can react) → max brake
            u_opt = self.u_min
            self._prob.update_settings(warm_starting=False)
        else:
            u_opt = float(result.x[0])
            self._prob.update_settings(warm_starting=True)

        u_opt = float(np.clip(u_opt, self.u_min, self.u_max))

        # advance predictor
        self._x_predicted = self.A @ x + self.B * u_opt + self.H @ ex
        self._u_prev = u_opt

        return u_opt, y.copy()

    def reset(self):
        """Reset state predictor + warm start."""
        self._x_predicted = None
        self._u_prev = 0.0
        if self._initialized:
            self._prob = osqp.OSQP()
            self._initialized = False
