"""ACC MPC controller for swarm follower (acceleration-input + integral action).

Plant model (Newton + force lag + integrator on commanded speed + integral
action on gap error):

    state:    x = [dx_err, v, v_rel, F, v_cmd, e_int]    (n_in = 6)
              dx_err = dx − d0                           (offset-free state)
    control:  u = a_cmd                                  (acceleration command)

    dx_err_dot  = v_rel
    v_dot       = (F − b·v) / m
    v_rel_dot   = −v_dot                                 (peer CV over horizon)
    F_dot       = (α·(v_cmd − v) + b·v − F) / τ_F
    v_cmd_dot   = u                                      (integrator on speed cmd)
    e_int_dot   = y_gap = dx_err − th·v                  (integrator on gap error)

Output for cost:
    y = [y_gap, v_rel, a, e_int]                          (n_out = 4)

    a = (F − b·v) / m   — physical acceleration, linear in state, ZERO in
                          steady state. Penalising a² gives clean damping
                          (no static offset). With q_a = 0 the term is
                          inert; with q_a > 0 it adds smoothing similar
                          to the kinematic MPC.

    e_int               — accumulator of y_gap (dx_err − th·v). This is
                          THE integrator action that the kinematic MPC
                          gets implicitly through its 5-state dx_err →
                          v_rel feedback path; the Newton plant has more
                          dynamics (force lag) and without an explicit
                          q_int term the cascade ends up with a steady
                          velocity bias of a few mm/s that integrates
                          over a long run into a 10–30 cm gap offset.
                          (Empirically verified: q_int = 0 → +27 cm
                          offset; q_int = 2 → +1.2 cm; q_int >> 2 has
                          diminishing returns and starts oscillating.)

The `v_cmd` integrator (4th state) ensures published velocity is smooth;
move suppression on Δa_cmd directly bounds jerk = Δa · ts.

Constraints:
    a_cmd ∈ [a_min, a_max]                                (input bound)
    v_cmd ∈ [v_cmd_min, v_cmd_max]                        (state bound on horizon)
    dx ≥ gap_safe                                         (hard collision-avoidance:
                                                           proactive braking the
                                                           moment the predictor
                                                           shows a violation —
                                                           PID/sliding can't do this)

Notes for future extension to lateral / 2-D MPC:
  * `n_in`, `n_out` are class attributes; everything else is computed from them.
    Adding cross-track / heading states only requires extending the matrices.
  * Integral action plumbing (state row + cost weight) is a generic mechanism —
    the same pattern is reused for cross-track integral when we go 2-D.
  * The dynamics matrices A, B are constant ONLY because the plant is linear
    and time-invariant. For lateral MPC with linearisation around the current
    state, A and B will need to be rebuilt every cycle (LTV-MPC) — `_build_dynamics`
    is already isolated to make that change cleanly.
"""

import numpy as np
from scipy.sparse import csc_matrix
import osqp


class SwarmAccController:
    """Adaptive Cruise Control MPC for a swarm follower."""

    n_in = 6   # state: [dx_err, v, v_rel, F, v_cmd, e_int]
    n_out = 4  # output: [y_gap, v_rel, a, e_int]   (a = (F-b·v)/m)

    def __init__(
        self,
        m: float, b: float, alpha: float, tau_F: float,
        d0: float, th: float,
        ts: float, p: int, c: int, s: float,
        phi_vals, q_vals,
        a_limits, v_cmd_limits, F_limits,
        gap_safe: float = 0.0,
    ):
        # plant
        self.m = float(m)
        self.b = float(b)
        self.alpha = float(alpha)
        self.tau_F = float(tau_F)
        # gap policy
        self.d0 = float(d0)
        self.th = float(th)
        # MPC tuning
        self.ts = float(ts)
        self.p = int(p)
        self.c = int(c)
        self.s = float(s)
        # constraints
        self.a_min, self.a_max = float(a_limits[0]), float(a_limits[1])
        self.v_cmd_min, self.v_cmd_max = float(v_cmd_limits[0]), float(v_cmd_limits[1])
        self.F_min, self.F_max = float(F_limits[0]), float(F_limits[1])  # reserved
        self.gap_safe = float(gap_safe)        # hard: dx ≥ gap_safe (collision avoidance)

        n_in, n_out, p_, c_ = self.n_in, self.n_out, self.p, self.c

        phi = np.asarray(phi_vals, dtype=float)
        q = np.asarray(q_vals, dtype=float)
        assert phi.shape == (n_out,), f"phi_vals must have length {n_out}"
        assert q.shape == (n_out,), f"q_vals must have length {n_out}"
        assert c_ <= p_, "control horizon c must not exceed prediction horizon p"

        # output matrix C (4×6):  y = C @ x   (no Z offset thanks to dx_err state)
        #  state cols:    dx_err  v             v_rel  F        v_cmd  e_int
        C = np.zeros((n_out, n_in))
        C[0, 0] = 1.0;   C[0, 1] = -self.th     # y_gap = dx_err − th·v
        C[1, 2] = 1.0                            # v_rel
        C[2, 1] = -self.b / self.m               # a = (F − b·v) / m  ← zero in
        C[2, 3] = 1.0 / self.m                   #     steady state (clean cost)
        C[3, 5] = 1.0                            # e_int (provides integral
                                                 # action; cost q_int forces
                                                 # mean(y_gap)→0 without which
                                                 # the 6-state plant accumulates
                                                 # mm/s velocity bias into cm
                                                 # offset over time — verified
                                                 # empirically in cascade)
        self.C = C

        # error correction H = I, F_mat = C @ H
        self.H = np.eye(n_in)
        self.F_mat = C @ self.H

        # constant dynamics (Euler discretisation)
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

        # state-prediction stacks A_hat (rows × p), C_hat (rows × p)
        A_hat = np.zeros((p_ * n_in, n_in))
        C_hat = np.zeros((p_ * n_out, n_in))
        A_pow = np.eye(n_in)
        for i in range(p_):
            A_pow = A_pow @ self.A
            A_hat[i * n_in:(i + 1) * n_in, :] = A_pow
            C_hat[i * n_out:(i + 1) * n_out, :] = self.C @ A_pow
        self.A_hat = A_hat
        self.C_hat = C_hat

        # impulse-response blocks C @ A^k @ B for k = 0..p−1
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

        # selector for v_cmd state (index 4) — used for state-bound constraint
        e_vcmd = np.zeros(n_in); e_vcmd[4] = 1.0
        # free response of v_cmd over horizon
        M_free_v = A_hat[4::n_in, :]                       # (p, n_in)
        # forced response of v_cmd: scalar e_vcmd^T · A^k · B
        e_AkB = np.zeros(p_)
        A_pow = np.eye(n_in)
        for k in range(p_):
            e_AkB[k] = float(e_vcmd @ A_pow @ self.B)
            A_pow = A_pow @ self.A
        D_v = np.zeros((p_, c_))
        for i in range(p_):
            for m in range(min(i + 1, c_)):
                k = i - m
                D_v[i, m] = e_AkB[k]
        self.M_free_v = M_free_v
        self.D_v = D_v

        # selector for dx_err state (index 0) — used for safety constraint
        e_dx = np.zeros(n_in); e_dx[0] = 1.0
        M_free_dx = A_hat[0::n_in, :]                      # (p, n_in)
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

        # M2: linear coupling u_prev → u_0
        self.M2 = np.zeros(c_)
        self.M2[0] = self.s

        # CONSTANT QP Hessian
        Hqp = self.H1 + self.D_hat.T @ self.Qmat @ self.D_hat
        Hqp = 0.5 * (Hqp + Hqp.T)
        self.Hqp = Hqp

        # state predictor and warm start
        self._x_predicted = None     # stores 6-vector
        self._u_prev = 0.0

        # OSQP setup once
        self._prob = osqp.OSQP()
        self._initialized = False

    def _build_dynamics(self):
        """Build A (6×6) and B (6×1) — Euler discretisation, constant in plant params.

        State order: [dx_err, v, v_rel, F, v_cmd, e_int].
        """
        ts, m, b, alpha, tau_F, th = (
            self.ts, self.m, self.b, self.alpha, self.tau_F, self.th)
        A = np.array([
            #  dx_err  v                       v_rel  F                v_cmd               e_int
            [  1.0,    0.0,                    ts,    0.0,             0.0,                0.0  ],  # dx_err
            [  0.0,    1.0 - ts * b / m,       0.0,   ts / m,          0.0,                0.0  ],  # v
            [  0.0,    ts * b / m,             1.0,  -ts / m,          0.0,                0.0  ],  # v_rel
            [  0.0,    ts * (b - alpha) / tau_F, 0.0, 1.0 - ts / tau_F, ts * alpha / tau_F, 0.0  ],  # F
            [  0.0,    0.0,                    0.0,   0.0,             1.0,                0.0  ],  # v_cmd
            [  ts,    -ts * th,                0.0,   0.0,             0.0,                1.0  ],  # e_int
        ])
        # u (= a_cmd) integrates only into v_cmd state
        B = np.array([0.0, 0.0, 0.0, 0.0, ts, 0.0])
        return A, B

    def calculate_control(self, dx: float, v: float, v_rel: float):
        """Solve one MPC step.

        Args:
            dx     — current distance to peer [m]
            v      — own longitudinal speed   [m/s]   (from odom)
            v_rel  — peer_v − v               [m/s]   (from telemetry)

        Returns:
            (v_cmd, a_cmd, y) where:
                v_cmd  — what to publish in Twist.linear.x  (state, smooth)
                a_cmd  — optimal acceleration command        (control input)
                y      — current output vector
                          [y_gap, v_rel, F_pred, e_int_pred]
        """
        n_in, c_, p_ = self.n_in, self.c, self.p

        # estimated F, v_cmd, e_int from predictor (or 0 first call)
        if self._x_predicted is not None:
            F_est = float(self._x_predicted[3])
            v_cmd_est = float(self._x_predicted[4])
            e_int_est = float(self._x_predicted[5])
        else:
            F_est = 0.0
            v_cmd_est = 0.0
            e_int_est = 0.0

        dx_err = dx - self.d0
        x = np.array([dx_err, v, v_rel, F_est, v_cmd_est, e_int_est])

        # state error correction (only on directly-measured states; integrator
        # state e_int is internal, ex on it stays 0 by construction since the
        # measurement copy in x[5] equals the predictor's prior estimate)
        if self._x_predicted is not None:
            ex = x - self._x_predicted
        else:
            ex = np.zeros(n_in)

        # current output, reference, free response, tracking residual
        y = self.C @ x
        Yr = self.FA @ y
        F2 = self.C_hat @ x + self.F_hat @ ex
        b1 = Yr - F2

        # gradient
        g = -self.M2 * self._u_prev - self.D_hat.T @ self.Qmat @ b1

        # ── constraints ─────────────────────────────────────────────────────
        # 1. input bound on a_cmd:                a_min ≤ u_k ≤ a_max
        # 2. SAFETY: dx ≥ gap_safe (so dx_err ≥ gap_safe − d0) over horizon:
        #      dx_err_k+i = (M_free_dx · x_now)_i + (D_dx · u)_i ≥ gap_safe − d0
        #    Disabled (no row added) when gap_safe ≤ 0 — keeps the QP smaller
        #    when collision avoidance isn't requested.
        #
        # NB: a v_cmd state bound used to be enforced over the horizon, but
        # this kinematic-style anti-windup is achieved instead by clipping
        # the published v_cmd to [v_cmd_min, v_cmd_max] (see end of method).
        # Enforcing it as a horizon-wide QP constraint was redundant with
        # that clip AND artificially throttled control authority: at
        # v_cmd_est ≈ v_peer ≈ 0.4 the available v_cmd headroom (0.1) had to
        # absorb the cumulative integral of u over up to p=20 steps, with
        # the far-horizon rows binding first even though receding-horizon
        # only ever applies u[0]. Removing it brings Newton MPC's QP
        # structurally in line with the kinematic MPC's QP (only input bound
        # + safety).  M_free_v / D_v are kept in __init__ for easy A/B.
        rows = [np.eye(c_)]
        lb_parts = [np.full(c_, self.a_min)]
        ub_parts = [np.full(c_, self.a_max)]
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
            # QP infeasible → typically means safety constraint cannot be met
            # at the current state (peer has braked harder than we can react).
            # Best safe action: maximum deceleration. Holding u_prev would
            # ignore the impending collision.
            a_opt = self.a_min
            self._prob.update_settings(warm_starting=False)
        else:
            a_opt = float(result.x[0])
            self._prob.update_settings(warm_starting=True)

        a_opt = float(np.clip(a_opt, self.a_min, self.a_max))

        # advance state predictor
        self._x_predicted = self.A @ x + self.B * a_opt + self.H @ ex
        self._u_prev = a_opt

        # anti-windup: clip the integrator state v_cmd to the published
        # range so the predictor cannot drift above what the plant actually
        # receives.  Mirrors the kinematic node's clip on `_v_cmd_published`
        # — necessary now that the v_cmd state bound is no longer enforced
        # inside the QP.
        self._x_predicted[4] = float(np.clip(
            self._x_predicted[4], self.v_cmd_min, self.v_cmd_max))

        # publish v_cmd from predictor (clamped for safety)
        v_cmd_published = float(self._x_predicted[4])

        return v_cmd_published, a_opt, y.copy()

    def reset(self):
        """Reset state predictor (incl. integrator) and warm start.

        Called on follower (re)activation: clears accumulated e_int so the
        controller doesn't act on stale integrated error after a stop.
        """
        self._x_predicted = None
        self._u_prev = 0.0
        if self._initialized:
            self._prob = osqp.OSQP()
            self._initialized = False
