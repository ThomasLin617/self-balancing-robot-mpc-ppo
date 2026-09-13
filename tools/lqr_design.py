import numpy as np
from scipy.linalg import solve_continuous_are


G = 9.81

# Robot parameters
BODY_MASS = 2.065
COM_HEIGHT = 0.2240339
BODY_PITCH_INERTIA = 0.04082156

WHEEL_MASS = 0.15
WHEEL_RADIUS = 0.12
WHEEL_INERTIA = 0.5 * WHEEL_MASS * WHEEL_RADIUS**2


def build_state_space():
    m = BODY_MASS
    l = COM_HEIGHT
    r = WHEEL_RADIUS

    effective_mass = (
        m
        + 2.0 * WHEEL_MASS
        + 2.0 * WHEEL_INERTIA / r**2
    )

    pitch_inertia = BODY_PITCH_INERTIA + m * l**2
    determinant = effective_mass * pitch_inertia - (m * l) ** 2

    # State: [position_error, forward_velocity, pitch, pitch_rate]
    # Input: equal torque applied to each wheel
    A = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, -(m**2 * G * l**2) / determinant, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, (effective_mass * m * G * l) / determinant, 0.0],
    ])

    B = np.array([
        [0.0],
        [(2.0 * pitch_inertia / r + 2.0 * m * l) / determinant],
        [0.0],
        [-(2.0 * m * l / r + 2.0 * effective_mass) / determinant],
    ])

    return A, B


def controllability_matrix(A, B):
    n = A.shape[0]
    return np.hstack([
        np.linalg.matrix_power(A, i) @ B
        for i in range(n)
    ])


def solve_lqr(A, B, Q, R):
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    return K, P


def main():
    A, B = build_state_space()

    position_limit = 0.30
    velocity_limit = 1.00
    pitch_limit = np.deg2rad(10.0)
    pitch_rate_limit = 2.00
    torque_limit = 2.00

    Q = np.diag([
        1.0 / position_limit**2,
        1.0 / velocity_limit**2,
        1.0 / pitch_limit**2,
        1.0 / pitch_rate_limit**2,
    ])

    R = np.array([
        [1.0 / torque_limit**2]
    ])

    controllability = controllability_matrix(A, B)
    rank = np.linalg.matrix_rank(controllability)

    K, P = solve_lqr(A, B, Q, R)

    open_loop_poles = np.linalg.eigvals(A)
    closed_loop_poles = np.linalg.eigvals(A - B @ K)

    np.set_printoptions(precision=5, suppress=True)

    print("A:")
    print(A)

    print("\nB:")
    print(B)

    print(f"\nControllability rank: {rank}/{A.shape[0]}")

    print("\nQ:")
    print(Q)

    print("\nR:")
    print(R)

    print("\nP:")
    print(P)

    print("\nK:")
    print(K)

    print("\nOpen-loop poles:")
    print(open_loop_poles)

    print("\nClosed-loop poles:")
    print(closed_loop_poles)


if __name__ == "__main__":
    main()