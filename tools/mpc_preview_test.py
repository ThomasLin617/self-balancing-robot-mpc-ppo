import numpy as np

from mpc_design import design, HORIZON, SAMPLE_TIME, solve_unconstrained_qp


def main():
    Ad, Bd, Qd, Rd, P, Sx, Su, H, F = design()
    Qbar = np.kron(np.eye(HORIZON), Qd)
    Qbar[-6:, -6:] = P
    G = 2.0 * Su.T @ Qbar
    state = np.array([0.03, -0.02, 0.01, -0.015, 0.1, 0.03])
    reference = np.array([0.1, 0.0, 0.0, 0.0, 0.2, 0.0])
    constant = np.tile(reference, HORIZON)
    old_gradient = F @ (state - reference)
    new_gradient = G @ (Sx @ state - constant)
    np.testing.assert_allclose(new_gradient, old_gradient, atol=1e-10)
    constant_action = solve_unconstrained_qp(H, new_gradient)[:2]
    print("Constant reference, old and preview first action:", constant_action)

    future = np.zeros((HORIZON, 6))
    for i in range(HORIZON):
        t = (i + 1) * SAMPLE_TIME
        q = np.clip((t - 0.15) / 0.35, 0.0, 1.0)
        future[i, 4] = 0.2 * q * q * (3.0 - 2.0 * q)
        future[i, 5] = 0.2 * 6.0 * q * (1.0 - q) / 0.35
    gradient = -G @ future.ravel()
    preview = solve_unconstrained_qp(H, gradient)
    frozen = solve_unconstrained_qp(H, np.zeros(2 * HORIZON))
    assert np.max(np.abs(preview)) < 2.0
    print("Current reference held:", frozen[:2])
    print("Turn starts in 0.15 s, preview first action:", preview[:2])
    assert np.linalg.norm(preview[:2] - frozen[:2]) > 1e-6

    # Evaluate the original tracking cost independently of its QP expansion.
    U = np.linspace(-0.1, 0.1, 2 * HORIZON)
    error = Sx @ state + Su @ U - future.ravel()
    Rbar = np.kron(np.eye(HORIZON), Rd)
    analytic = H @ U + G @ (Sx @ state - future.ravel())
    for index in (0, 1, 137, 499):
        direction = np.zeros(2 * HORIZON)
        direction[index] = 1e-5
        plus_error = error + Su @ direction
        minus_error = error - Su @ direction
        plus = plus_error @ Qbar @ plus_error + (U + direction) @ Rbar @ (U + direction)
        minus = minus_error @ Qbar @ minus_error + (U - direction) @ Rbar @ (U - direction)
        np.testing.assert_allclose((plus - minus) / 2e-5, analytic[index], atol=1e-6)

    for name, index, value in (("velocity", 1, 0.1), ("pitch", 2, 0.02), ("yaw rate", 5, 0.1)):
        moving = reference.copy()
        moving[index] = value
        physical_gradient = G @ (Sx @ state - np.tile(moving, HORIZON))
        assert np.linalg.norm(F @ (state - moving) - physical_gradient) > 1e-3
        action = solve_unconstrained_qp(H, physical_gradient)[:2]
        print("Constant nonzero", name, "physical tracking action:", action)
    print("PASS constant equilibrium regression, future turn, tracking-cost gradient")


if __name__ == "__main__":
    np.set_printoptions(precision=15)
    main()
