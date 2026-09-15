import numpy as np
from scipy.linalg import expm, solve_discrete_are
from scipy.optimize import minimize


SAMPLE_TIME = 0.002
HORIZON = 250
TORQUE_LIMIT = 2.0

A = np.array([
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, -14.06236, 0.0],
    [0.0, 0.0, 0.0, 1.0],
    [0.0, 0.0, 76.44735, 0.0],
])

B = np.array([
    [0.0],
    [22.32344],
    [0.0],
    [-85.33125],
])

Q_CONTINUOUS = np.diag([
    11.11111,
    1.0,
    32.82806,
    0.25,
])

R_CONTINUOUS = np.array([
    [0.25]
])


def discretize_zoh(A, B, dt):
    state_dim = A.shape[0]
    input_dim = B.shape[1]

    augmented = np.zeros((
        state_dim + input_dim,
        state_dim + input_dim
    ))

    augmented[:state_dim, :state_dim] = A
    augmented[:state_dim, state_dim:] = B

    discrete = expm(augmented * dt)

    Ad = discrete[:state_dim, :state_dim]
    Bd = discrete[:state_dim, state_dim:]

    return Ad, Bd


def build_prediction_matrices(Ad, Bd, horizon):
    state_dim = Ad.shape[0]
    input_dim = Bd.shape[1]

    Sx = np.zeros((
        state_dim * horizon,
        state_dim
    ))

    Su = np.zeros((
        state_dim * horizon,
        input_dim * horizon
    ))

    powers = [np.eye(state_dim)]

    for _ in range(horizon):
        powers.append(powers[-1] @ Ad)

    for i in range(horizon):
        state_start = i * state_dim
        state_end = (i + 1) * state_dim

        Sx[state_start:state_end, :] = powers[i + 1]

        for j in range(i + 1):
            input_start = j * input_dim
            input_end = (j + 1) * input_dim

            Su[
                state_start:state_end,
                input_start:input_end
            ] = powers[i - j] @ Bd

    return Sx, Su


def build_qp_matrices(Su, Qbar, Rbar):
    H = 2.0 * (
        Su.T @ Qbar @ Su
        + Rbar
    )

    return H


def build_qp_gradient(Sx, Su, Qbar, state):
    f = (
        2.0
        * Su.T
        @ Qbar
        @ Sx
        @ state
    )

    return f


def solve_unconstrained_qp(H, f):
    return np.linalg.solve(H, -f)


def solve_box_constrained_qp(H, f, torque_limit):
    horizon = H.shape[0]

    unconstrained_solution = solve_unconstrained_qp(H, f)

    initial_guess = np.clip(
        unconstrained_solution,
        -torque_limit,
        torque_limit
    )

    bounds = [
        (-torque_limit, torque_limit)
        for _ in range(horizon)
    ]

    def objective(U):
        return 0.5 * U @ H @ U + f @ U

    def gradient(U):
        return H @ U + f

    result = minimize(
        objective,
        initial_guess,
        jac=gradient,
        method="L-BFGS-B",
        bounds=bounds,
        options={
            "ftol": 1e-12,
            "gtol": 1e-10,
            "maxiter": 1000,
            "maxls": 50,
        },
    )

    if not result.success:
        raise RuntimeError(
            f"Constrained QP failed: {result.message}"
        )

    return result.x, result


def main():
    Ad, Bd = discretize_zoh(
        A,
        B,
        SAMPLE_TIME
    )

    Qd = SAMPLE_TIME * Q_CONTINUOUS
    Rd = SAMPLE_TIME * R_CONTINUOUS

    P = solve_discrete_are(
        Ad,
        Bd,
        Qd,
        Rd
    )

    K = np.linalg.solve(
        Rd + Bd.T @ P @ Bd,
        Bd.T @ P @ Ad
    )

    closed_loop_poles = np.linalg.eigvals(
        Ad - Bd @ K
    )

    Sx, Su = build_prediction_matrices(
        Ad,
        Bd,
        HORIZON
    )

    state_dim = Ad.shape[0]

    Qbar = np.kron(
        np.eye(HORIZON),
        Qd
    )

    Qbar[
        -state_dim:,
        -state_dim:
    ] = P

    Rbar = np.kron(
        np.eye(HORIZON),
        Rd
    )

    H = build_qp_matrices(
        Su,
        Qbar,
        Rbar
    )

    np.set_printoptions(
        precision=15,
        suppress=True
    )

    print("Ad:")
    print(Ad)

    print("\nBd:")
    print(Bd)

    print("\nQd:")
    print(Qd)

    print("\nRd:")
    print(Rd)

    print("\nP:")
    print(P)

    print("\nDiscrete LQR gain:")
    print(K)

    print("\nClosed-loop poles:")
    print(closed_loop_poles)

    print("\nPrediction matrix shapes:")
    print("Sx:", Sx.shape)
    print("Su:", Su.shape)
    print("H:", H.shape)

    small_error_state = np.array([
        -0.1,
        0.0,
        0.0,
        0.0,
    ])

    f_small = build_qp_gradient(
        Sx,
        Su,
        Qbar,
        small_error_state
    )

    U_unconstrained = solve_unconstrained_qp(
        H,
        f_small
    )

    mpc_first_input = U_unconstrained[0]

    lqr_input = float(
        (-K @ small_error_state.reshape(-1, 1))[0, 0]
    )

    print("\nUnconstrained MPC validation:")
    print(
        "MPC first input:",
        mpc_first_input
    )
    print(
        "Discrete LQR input:",
        lqr_input
    )
    print(
        "Difference:",
        abs(mpc_first_input - lqr_input)
    )

    large_error_state = np.array([
        -0.5,
        0.0,
        0.0,
        0.0,
    ])

    f_large = build_qp_gradient(
        Sx,
        Su,
        Qbar,
        large_error_state
    )

    U_large_unconstrained = solve_unconstrained_qp(
        H,
        f_large
    )

    U_constrained, solver_result = solve_box_constrained_qp(
        H,
        f_large,
        TORQUE_LIMIT
    )

    lqr_large_input = float(
        (-K @ large_error_state.reshape(-1, 1))[0, 0]
    )

    print("\nConstrained MPC validation:")
    print(
        "Unconstrained MPC first input:",
        U_large_unconstrained[0]
    )
    print(
        "Discrete LQR input:",
        lqr_large_input
    )
    print(
        "Constrained MPC first input:",
        U_constrained[0]
    )

    print(
        "Minimum MPC torque:",
        np.min(U_constrained)
    )

    print(
        "Maximum MPC torque:",
        np.max(U_constrained)
    )

    print(
        "Solver iterations:",
        solver_result.nit
    )

    print(
        "Solver status:",
        solver_result.message
    )


if __name__ == "__main__":
    main()