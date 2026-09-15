#include <controller/mpc_controller.hpp>
#include <stdexcept>
#include <vector>
namespace controller {

MPCController::MPCController(
    int horizon,
    double torque_limit
)
    : horizon_(horizon),
      torque_limit_(torque_limit),
      Rd_(0.0005)
{
    if (horizon_ <= 0) {
        throw std::invalid_argument(
            "MPC horizon must be positive."
        );
    }

    if (torque_limit_ <= 0.0) {
        throw std::invalid_argument(
            "Torque limit must be positive."
        );
    }

    Ad_ <<
        1.0, 0.002, -0.000028125436694, -0.000000018750100,
        0.0, 1.0, -0.028126153395458, -0.000028125436694,
        0.0, 0.0, 1.000152898596171, 0.002000101931358,
        0.0, 0.0, 0.152902492382236, 1.000152898596171;

    Bd_ <<
        0.000044647679981,
        0.044648479969472,
        -0.000170666848943,
        -0.170671197930232;


    Qd_.setZero();
    Qd_.diagonal() <<
        0.02222222,
        0.002,
        0.06565612,
        0.0005;

    P_ <<
        10.970920248655020, 4.905278720982848, 5.018046445836401, 1.302898335322889,
        4.905278720982848, 3.994263046826552, 3.646811455928461, 1.064037637729081,
        5.018046445836401, 3.646811455928461, 6.868041879728636, 1.008104289912070,
        1.302898335322889, 1.064037637729081, 1.008104289912070, 0.287476041587268;
        

    build_prediction_matrices();
    build_qp_matrices();
    initialize_solver();
}

void MPCController::build_prediction_matrices() {
    Sx_.resize(4 * horizon_, 4);
    Su_.setZero(4 * horizon_, horizon_);

    std::vector<Eigen::Matrix4d> powers(
        horizon_ + 1
    );

    powers[0] = Eigen::Matrix4d::Identity();

    for (int i = 1; i <= horizon_; ++i) {
        powers[i] = powers[i - 1] * Ad_;
    }

    for (int i = 0; i < horizon_; ++i) {
        Sx_.block<4, 4>(
            4 * i,
            0
        ) = powers[i + 1];

        for (int j = 0; j <= i; ++j) {
            Su_.block<4, 1>(
                4 * i,
                j
            ) = powers[i - j] * Bd_;
        }
    }
}

void MPCController::build_qp_matrices() {
    const int state_dim = 4;

    Eigen::MatrixXd Qbar =
        Eigen::MatrixXd::Zero(
            state_dim * horizon_,
            state_dim * horizon_
        );

    for (int i = 0; i < horizon_ - 1; ++i) {
        Qbar.block<4, 4>(
            state_dim * i,
            state_dim * i
        ) = Qd_;
    }

    Qbar.block<4, 4>(
        state_dim * (horizon_ - 1),
        state_dim * (horizon_ - 1)
    ) = P_;

    Eigen::MatrixXd Rbar =
        Rd_ * Eigen::MatrixXd::Identity(
            horizon_,
            horizon_
        );

    H_dense_ =
        2.0 * (
            Su_.transpose() * Qbar * Su_
            + Rbar
        );

    F_ =
        2.0
        * Su_.transpose()
        * Qbar
        * Sx_;

    H_ = H_dense_.sparseView();
    H_.makeCompressed();

    constraint_matrix_.resize(
        horizon_,
        horizon_
    );

    constraint_matrix_.setIdentity();
    constraint_matrix_.makeCompressed();

    gradient_ =
        Eigen::VectorXd::Zero(horizon_);

    lower_bound_ =
        Eigen::VectorXd::Constant(
            horizon_,
            -torque_limit_
        );

    upper_bound_ =
        Eigen::VectorXd::Constant(
            horizon_,
            torque_limit_
        );
}
void MPCController::initialize_solver() {
    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);

    solver_.data()->setNumberOfVariables(horizon_);
    solver_.data()->setNumberOfConstraints(horizon_);

    if (!solver_.data()->setHessianMatrix(H_)) {
        throw std::runtime_error(
            "Failed to set MPC Hessian."
        );
    }

    if (!solver_.data()->setGradient(gradient_)) {
        throw std::runtime_error(
            "Failed to set MPC gradient."
        );
    }

    if (!solver_.data()->setLinearConstraintsMatrix(
            constraint_matrix_)) {
        throw std::runtime_error(
            "Failed to set MPC constraint matrix."
        );
    }

    if (!solver_.data()->setLowerBound(lower_bound_)) {
        throw std::runtime_error(
            "Failed to set MPC lower bound."
        );
    }

    if (!solver_.data()->setUpperBound(upper_bound_)) {
        throw std::runtime_error(
            "Failed to set MPC upper bound."
        );
    }

    if (!solver_.initSolver()) {
        throw std::runtime_error(
            "Failed to initialize MPC solver."
        );
    }
}


Eigen::Vector4d MPCController::build_error_state(
    const robot_interface::RobotState& state,
    const robot_interface::Reference& reference
) const {
    Eigen::Vector4d error_state;

    error_state <<
        state.forward_position
            - reference.forward_position,
        state.forward_velocity
            - reference.forward_velocity,
        state.pitch
            - reference.pitch,
        state.pitch_rate;

    return error_state;
}


robot_interface::RobotCommand
MPCController::compute_command(
    const robot_interface::RobotState& state,
    const robot_interface::Reference& reference
) {
    const Eigen::Vector4d error_state =
        build_error_state(state, reference);

    gradient_ = F_ * error_state;

    if (!solver_.updateGradient(gradient_)) {
        throw std::runtime_error(
            "Failed to update MPC gradient."
        );
    }

    const auto solve_status =
        solver_.solveProblem();

    if (solve_status !=
        OsqpEigen::ErrorExitFlag::NoError) {
        throw std::runtime_error(
            "MPC solver failed to run."
        );
    }

    if (solver_.getStatus() != OsqpEigen::Status::Solved) {
        throw std::runtime_error(
            "MPC optimization did not converge to a solved solution."
        );
    }

    const auto& solution =
        solver_.getSolution();

    if (solution.size() == 0) {
        throw std::runtime_error(
            "MPC solver returned empty solution."
        );
    }


    const double torque = solution[0];

    robot_interface::RobotCommand command;
    command.timestamp = state.timestamp;
    command.left_wheel_torque = torque;
    command.right_wheel_torque = torque;

    return command;
}
}// namespace controller
