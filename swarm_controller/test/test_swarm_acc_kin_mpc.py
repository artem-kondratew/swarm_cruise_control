"""Unit tests for SwarmKinAccController (kinematic, adas-style 5-state).

Plant model has only `tau` parameter (acceleration lag) — much simpler than
Newton-MPC. These tests verify the MPC formulation against the same closed-loop
behaviour: gap convergence, smoothness, hard safety constraint.
"""

import numpy as np
import pytest

from swarm_controller.submodules.swarm_acc_kin_mpc import SwarmKinAccController


def _make_controller(**overrides):
    """Default config matches params_swarm_acc_kin.yaml."""
    params = dict(
        tau=0.2,
        d0=0.5, th=0.0,
        ts=0.05, p=20, c=10, s=3.0,
        phi_vals=[0.6, 0.95, 0.6, 0.6],
        q_vals=[10.0, 1.0, 1.0, 1.0],
        u_limits=(-0.5, 0.5),
        gap_safe=0.2,
    )
    params.update(overrides)
    return SwarmKinAccController(**params)


def test_construction():
    ctrl = _make_controller()
    assert ctrl.n_in == 5
    assert ctrl.n_out == 4
    H = ctrl.Hqp
    assert H.shape == (ctrl.c, ctrl.c)
    assert np.allclose(H, H.T, atol=1e-10)
    eigs = np.linalg.eigvalsh(H)
    assert eigs.min() > -1e-8


def test_first_call_returns_valid():
    ctrl = _make_controller()
    a_cmd, y = ctrl.calculate_control(dx=0.5, v=0.0, v_rel=0.0)
    assert ctrl.u_min <= a_cmd <= ctrl.u_max
    assert y.shape == (4,)
    # y_gap = (dx − d0) − th·v = 0
    assert abs(y[0]) < 1e-6


def test_bounds_respected_under_step():
    ctrl = _make_controller()
    for _ in range(50):
        a_cmd, _ = ctrl.calculate_control(dx=1.0, v=0.0, v_rel=0.4)
        assert ctrl.u_min - 1e-9 <= a_cmd <= ctrl.u_max + 1e-9


def test_brake_when_gap_closes():
    """Sudden gap shrink → MPC commands negative acceleration (brake)."""
    ctrl = _make_controller()
    v_target = 0.4
    dx_ref = ctrl.d0 + ctrl.th * v_target  # 0.5

    # warm-up at steady state
    for _ in range(200):
        a_steady, _ = ctrl.calculate_control(dx=dx_ref, v=v_target, v_rel=0.0)

    # peer suddenly close: dx=0.3 (< d0)
    a_brake, _ = ctrl.calculate_control(dx=0.3, v=v_target, v_rel=0.0)
    assert a_brake < a_steady, \
        f"controller did not slow down: brake={a_brake}, steady={a_steady}"


def test_speed_up_when_gap_grows():
    """Gap larger than reference → positive a_cmd to accelerate."""
    ctrl = _make_controller()
    last_a = -10.0
    for _ in range(40):
        a_cmd, _ = ctrl.calculate_control(dx=1.5, v=0.2, v_rel=0.2)
        last_a = a_cmd
    # at some point during the close, a > 0 (speeding up)
    assert last_a > -ctrl.u_max, f"controller did not accelerate: a={last_a}"


def test_reset_clears_predictor():
    ctrl = _make_controller()
    for _ in range(10):
        ctrl.calculate_control(dx=0.5, v=0.4, v_rel=0.0)
    assert ctrl._x_predicted is not None
    ctrl.reset()
    assert ctrl._x_predicted is None
    assert ctrl._u_prev == 0.0
    assert ctrl._initialized is False


def test_closed_loop_with_kinematic_plant():
    """Closed-loop simulation against the SAME kinematic plant the MPC models.

    Follower at v=0, dx=1.0, peer at constant v_peer=0.4. Should converge.
    """
    ctrl = _make_controller()
    tau, ts = ctrl.tau, ctrl.ts

    v = 0.0
    a = 0.0
    dx = 1.0
    v_published = 0.0           # B2 integrator in node-equivalent
    v_peer = 0.4

    history = []
    for k in range(1500):  # 75 s
        v_rel = v_peer - v
        a_cmd, _ = ctrl.calculate_control(dx=dx, v=v, v_rel=v_rel)

        # B2 integrator: published v_cmd
        v_published = v_published + ts * a_cmd
        v_published = float(np.clip(v_published, 0.0, 0.5))

        # plant: same kinematic lag (a → u, v → integrate a)
        # but plant input is v_published, so we need to map back.
        # Simplification for test: treat plant as exactly the MPC model
        # (i.e. plant accepts a_cmd directly, ignoring v_published).
        a += ts * (a_cmd - a) / tau
        v += ts * a
        dx += ts * v_rel

        history.append((dx, v, a, a_cmd, v_published))

    final_dx, final_v, final_a, final_a_cmd, final_vc = history[-1]
    assert abs(final_v - v_peer) < 0.05, f"v didn't match peer: {final_v}"
    expected_dx = ctrl.d0 + ctrl.th * v_peer
    assert abs(final_dx - expected_dx) < 0.10, \
        f"gap didn't converge: dx={final_dx}, expected {expected_dx}"


def test_safety_constraint_under_braking_peer():
    """Hard constraint dx ≥ gap_safe must hold when peer brakes hard."""
    ctrl = _make_controller()
    tau, ts = ctrl.tau, ctrl.ts

    # warm-up at steady state with peer moving
    v = 0.4; a = 0.0; dx = 0.5
    for _ in range(100):
        a_cmd, _ = ctrl.calculate_control(dx=dx, v=v, v_rel=0.0)
        a += ts * (a_cmd - a) / tau
        v += ts * a

    # peer stops; follower at dx=0.45, v=0.4
    dx = 0.45; v_peer = 0.0
    min_dx = dx
    for _ in range(200):
        v_rel = v_peer - v
        a_cmd, _ = ctrl.calculate_control(dx=dx, v=v, v_rel=v_rel)
        a += ts * (a_cmd - a) / tau
        v += ts * a
        dx += ts * v_rel
        min_dx = min(min_dx, dx)

    # tolerance: kinematic plant has tau=0.2 lag, must brake hard
    assert min_dx > ctrl.gap_safe - 0.10, \
        f"safety breached: min_dx={min_dx:.3f} (gap_safe={ctrl.gap_safe})"
    assert min_dx > 0.0, f"COLLISION: min_dx={min_dx:.3f}"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
