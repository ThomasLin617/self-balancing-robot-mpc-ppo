#include <controller/mpc_controller.hpp>
#include <controller/mpc_model.hpp>
#include <robot_interface/angle.hpp>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <cmath>
namespace controller {

MPCController::MPCController(
    int horizon,
    double torque_limit
)
    : horizon_(horizon),
      torque_limit_(torque_limit)
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

    mpc_model::initialize(Ad_, Bd_, Qd_, Rd_, P_);

    build_prediction_matrices();
    build_qp_matrices();
    initialize_solver();
}

void MPCController::build_prediction_matrices() {
    Sx_.resize(6 * horizon_, 6);
    Su_.setZero(6 * horizon_, 2 * horizon_);

    std::vector<Eigen::Matrix<double, 6, 6>> powers(
        horizon_ + 1
    );

    powers[0] = Eigen::Matrix<double, 6, 6>::Identity();

    for (int i = 1; i <= horizon_; ++i) {
        powers[i] = powers[i - 1] * Ad_;
    }

    for (int i = 0; i < horizon_; ++i) {
        Sx_.block<6, 6>(
            6 * i,
            0
        ) = powers[i + 1];

        for (int j = 0; j <= i; ++j) {
            Su_.block<6, 2>(
                6 * i,
                2 * j
            ) = powers[i - j] * Bd_;
        }
    }
}

void MPCController::build_qp_matrices() {
    const int state_dim = 6;

    Eigen::MatrixXd Qbar =
        Eigen::MatrixXd::Zero(
            state_dim * horizon_,
            state_dim * horizon_
        );

    for (int i = 0; i < horizon_ - 1; ++i) {
        Qbar.block<6, 6>(
            state_dim * i,
            state_dim * i
        ) = Qd_;
    }

    Qbar.block<6, 6>(
        state_dim * (horizon_ - 1),
        state_dim * (horizon_ - 1)
    ) = P_;

    Eigen::MatrixXd Rbar = Eigen::MatrixXd::Zero(
        2 * horizon_, 2 * horizon_
    );
    for (int i = 0; i < horizon_; ++i) {
        Rbar.block<2, 2>(2 * i, 2 * i) = Rd_;
    }

    H_dense_ =
        2.0 * (
            Su_.transpose() * Qbar * Su_
            + Rbar
        );

    G_ = 2.0 * Su_.transpose() * Qbar;
    F_ = G_ * Sx_;
    reference_vector_.resize(6 * horizon_);
    constant_references_.resize(horizon_);

    H_ = H_dense_.sparseView();
    H_.makeCompressed();

    constraint_matrix_.resize(
        2 * horizon_,
        2 * horizon_
    );

    constraint_matrix_.setIdentity();
    constraint_matrix_.makeCompressed();

    gradient_ =
        Eigen::VectorXd::Zero(2 * horizon_);

    lower_bound_ =
        Eigen::VectorXd::Constant(
            2 * horizon_,
            -torque_limit_
        );

    upper_bound_ =
        Eigen::VectorXd::Constant(
            2 * horizon_,
            torque_limit_
        );
}
void MPCController::initialize_solver() {
    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);
    solver_.settings()->setAbsoluteTolerance(1e-7);
    solver_.settings()->setRelativeTolerance(1e-7);

    solver_.data()->setNumberOfVariables(2 * horizon_);
    solver_.data()->setNumberOfConstraints(2 * horizon_);

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


int MPCController::horizon_size() const {
    return horizon_;
}

double MPCController::sample_time() const {
    return mpc_model::sample_time;
}

robot_interface::RobotCommand MPCController::compute_command(
    const robot_interface::RobotState& state,
    const robot_interface::Reference& reference
) {
    std::fill(constant_references_.begin(), constant_references_.end(), reference);
    return compute_command(state, constant_references_);
}

robot_interface::RobotCommand MPCController::compute_command(
    const robot_interface::RobotState& state,
    const robot_interface::ReferenceHorizon& references
) {
    if (references.size() != static_cast<std::size_t>(horizon_)) {
        throw std::invalid_argument("MPC reference horizon has incorrect length.");
    }
    Eigen::Matrix<double, 6, 1> current_state;
    current_state << state.forward_position, state.forward_velocity,
                     state.pitch, state.pitch_rate, state.yaw, state.yaw_rate;
    if (!current_state.allFinite()) {
        throw std::invalid_argument("MPC state must be finite.");
    }

    double previous_yaw = state.yaw;
    for (int i = 0; i < horizon_; ++i) {
        const auto& reference = references[i];
        const double yaw = previous_yaw + robot_interface::wrap_to_pi(reference.yaw - previous_yaw);
        reference_vector_.segment<6>(6 * i) <<
            reference.forward_position, reference.forward_velocity,
            reference.pitch, 0.0, yaw, reference.yaw_rate;
        previous_yaw = yaw;
    }
    if (!reference_vector_.allFinite()) {
        throw std::invalid_argument("MPC references must be finite.");
    }
    gradient_.noalias() = F_ * current_state;
    gradient_.noalias() -= G_ * reference_vector_;

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

    if (solution.size() != 2 * horizon_) {
        throw std::runtime_error(
            "MPC solver returned empty solution."
        );
    }



    robot_interface::RobotCommand command;
    command.timestamp = state.timestamp;
    command.left_wheel_torque = solution[0];
    command.right_wheel_torque = solution[1];

    return command;
}
}// namespace controller
