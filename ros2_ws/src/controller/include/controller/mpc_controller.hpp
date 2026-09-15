#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>

#include <robot_interface/reference.hpp>
#include <robot_interface/robot_command.hpp>
#include <robot_interface/robot_state.hpp>

namespace controller {

class MPCController {
public:
    MPCController(
        int horizon,
        double torque_limit
    );

    robot_interface::RobotCommand compute_command(
        const robot_interface::RobotState& state,
        const robot_interface::Reference& reference
    );

private:
    void build_prediction_matrices();
    void build_qp_matrices();
    void initialize_solver();

    Eigen::Vector4d build_error_state(
        const robot_interface::RobotState& state,
        const robot_interface::Reference& reference
    ) const;

    int horizon_;
    double torque_limit_;

    Eigen::Matrix4d Ad_;
    Eigen::Vector4d Bd_;

    Eigen::Matrix4d Qd_;
    double Rd_;
    Eigen::Matrix4d P_;

    Eigen::MatrixXd Sx_;
    Eigen::MatrixXd Su_;

    Eigen::MatrixXd H_dense_;
    Eigen::MatrixXd F_;

    Eigen::SparseMatrix<double> H_;
    Eigen::SparseMatrix<double> constraint_matrix_;

    Eigen::VectorXd gradient_;
    Eigen::VectorXd lower_bound_;
    Eigen::VectorXd upper_bound_;

    OsqpEigen::Solver solver_;
};

}