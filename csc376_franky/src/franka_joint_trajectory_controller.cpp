#include <chrono>
#include <cstdlib>
#include <cassert>
#include <functional>
#include <iostream>

#include <franka/rate_limiting.h>

#include "csc376_franky/franka_joint_trajectory_controller.hpp"

namespace csc376_franky
{

std::atomic<bool> FrankaJointTrajectoryController::stop_command_{false};

FrankaJointTrajectoryController::FrankaJointTrajectoryController(const std::string& fci_host_ip)
{
    robot_ = std::make_unique<franka::Robot>(fci_host_ip);

    // Locking is better but should be fine
    auto robot_state = robot_->readOnce();
    current_robot_state_ = robot_state;
    for (int i = 0; i < 7; ++i)
    {
        command_joint_positions_(i) = robot_state.q[i];
        command_joint_velocities_(i) = 0.0;
    }
    current_joint_positions_ = robot_state.q;

    // k_gains_ = {{800.0, 1100.0, 1200.0, 1000.0, 750.0, 400.0, 250.0}};
    k_gains_ = {{600.0, 900.0, 600.0, 300.0, 300.0, 200.0, 100.0}};
    d_gains_ = {{50.0,   70.0,  70.0,  60.0, 30.0, 15.0, 10.0}};
    // d_gains_ = {{35.0,    35.0,   35.0,  35.0, 15.0, 10.0, 10.0}};
    robot_->setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    // robot_->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    // robot_->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
    model_ = std::make_unique<franka::Model>(robot_->loadModel());

    auto robot_control_func = [this]() {
        this->robot_->control(std::bind(
            &FrankaJointTrajectoryController::impedanceControlCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
    };
    trajectory_callback_ = [](int){}; // Default empty callback
    control_thread_ = std::thread(robot_control_func);
}

FrankaJointTrajectoryController::~FrankaJointTrajectoryController()
{
    this->stop();
}

void FrankaJointTrajectoryController::join()
{
    control_thread_.join();
}

void FrankaJointTrajectoryController::setupSignalHandler() {
    std::signal(SIGINT, handleSignal);
}

void FrankaJointTrajectoryController::handleSignal(int signal) {
    if (signal == SIGINT) {
        stop_command_.store(true);
        // std::cout << "\n[InterruptHandler] SIGINT received. Flag set to true.\n";
        // Re-install default handler so Python can raise KeyboardInterrupt
        std::signal(SIGINT, SIG_DFL);
        // Re-raise signal to allow Python to handle it and terminate if needed
        std::raise(SIGINT);
    }
}

void FrankaJointTrajectoryController::setTrajectoryCallback(
    const std::function<void(int)>& trajectory_callback)
{
    std::lock_guard<std::mutex> lock(trajectory_callback_mtx_);
    trajectory_callback_ = trajectory_callback;
    return;
}

void FrankaJointTrajectoryController::stop()
{
    stop_command_ = true;
    robot_->stop();
    control_thread_.join();
}

Eigen::MatrixXd computeRowDifferences(const Eigen::MatrixXd& trajectory) {
    if (trajectory.rows() < 2) {
        return Eigen::MatrixXd::Zero(0, trajectory.cols());
    }
    
    Eigen::MatrixXd row_differences = (trajectory.bottomRows(trajectory.rows() - 1) - 
                                  trajectory.topRows(trajectory.rows() - 1));
    return row_differences;
}

ErrorCodes FrankaJointTrajectoryController::runJointTrajectory(
    const Eigen::Ref<const Eigen::MatrixXd>& joint_trajectory, float dt)
{
    // Argument validation
    if (dt <= 0.0) {
        std::cout << "dt must be larger than 0" << std::endl;
        return ErrorCodes::DtIsNegative;
    }
    if (dt < 0.01) {
        std::cout << "dt of less than 0.01 not supported" << std::endl;
        return ErrorCodes::DtIsSmallerThanAllowed;   
    }
    if (joint_trajectory.cols() != 7) {
        std::cout << "Trajectory must have 7 columns (joints)" << std::endl;
        return ErrorCodes::JointSizeNotSeven;
    }
    
    const auto current_jps = getCurrentJointPositions();
    for (int i = 0; i < 7; ++i)
    {
        if (abs(joint_trajectory(0, i) - current_jps[i]) > MAX_JOINT_DIFFERENCE_)
        {
            std::cout << "Start of trajectory is exceeding max_joint_difference to current position of the robot" << std::endl;
            std::cout << "Joint: " << i << std::endl;
            std::cout << "start trajectory joint position: " << joint_trajectory(0, i) << std::endl;
            std::cout << "current robot joint position: " << current_jps[i] << std::endl;
            std::cout << "Maximum joint difference: " << MAX_JOINT_DIFFERENCE_<< std::endl;
            return ErrorCodes::JointTrajectoryDoesNotStartAtCurrentPosition;
        }
    }

    // Trajectory validation
    Eigen::MatrixXd joint_differences = computeRowDifferences(joint_trajectory);



    Eigen::Matrix<double, 7, 1> max_joint_differences = joint_differences.colwise().maxCoeff();
    for (int i = 0; i < 7; ++i)
    {
        if (max_joint_differences(i) > MAX_JOINT_DIFFERENCE_)
        {
            std::cout << "Maximum joint difference exceeded for joint: " << i << std::endl;
            std::cout << "Maximum joint difference: " << MAX_JOINT_DIFFERENCE_<< std::endl;
            return ErrorCodes::JointDistancesAboveMaxJointDifference;
        }
    }

    Eigen::MatrixXd joint_velocities(joint_differences.rows() + 1, joint_differences.cols());
    joint_velocities.row(0).setZero();
    joint_velocities.bottomRows(joint_differences.rows()) = joint_differences / dt;
    joint_velocities.row(joint_velocities.rows() - 1).setZero();

    Eigen::Matrix<double, 7, 1> max_joint_velocities = joint_velocities.colwise().maxCoeff();
    for (int i = 0; i < 7; ++i)
    {
        if (max_joint_velocities(i) > MAX_JOINT_VELOCITIES_[i] * 0.3)
        {
            std::cout << "Maximum joint velocity exceeded for joint: " << i << std::endl;
            std::cout << "User desired velocity: " << max_joint_velocities(i) << std::endl;
            std::cout << "Maximum joint velocity: " << MAX_JOINT_VELOCITIES_[i] * 0.3 << std::endl;
            return ErrorCodes::JointVelocitiesAboveMaximumAllowed;
        }
    }

    // std::cout << joint_trajectory.rows() << std::endl;
    // std::cout << joint_velocities.rows() << std::endl;
    // std::cout << joint_velocities << std::endl;
    // return ErrorCodes::JointTrajectoryDoesNotStartAtCurrentPosition;

    // Run trajectory
    std::lock_guard<std::mutex> lock(trajectory_callback_mtx_);
    for (int i = 0; i < joint_trajectory.rows(); ++i) 
    {
        setCommandJointPositions(joint_trajectory.row(i), joint_velocities.row(i));
        trajectory_callback_(i);
        std::this_thread::sleep_for(std::chrono::duration<float>(dt));
        if(stop_command_.load())
        {
            return ErrorCodes::Interrupted;
        }
    }
    return ErrorCodes::Success;
}

std::array<double, 7> FrankaJointTrajectoryController::getCurrentJointPositions()
{
    std::lock_guard<std::mutex> lock(current_state_mtx_);
    return current_joint_positions_;
}


franka::RobotState FrankaJointTrajectoryController::getCurrentRobotState()
{
    std::lock_guard<std::mutex> lock(current_state_mtx_);
    return current_robot_state_;    
}

void FrankaJointTrajectoryController::setCommandJointPositions(
    const Eigen::Ref<const Eigen::Matrix<double, 7, 1>>& joint_positions)
{
    std::lock_guard<std::mutex> lock(command_state_mtx_);
    command_joint_positions_ = joint_positions;
}

// Big assumption that joint_velocities can end at zero. 
// idk what happens if we leave joint velocities non zero.
void FrankaJointTrajectoryController::setCommandJointPositions(
    const Eigen::Ref<const Eigen::Matrix<double, 7, 1>>& joint_positions, 
    const Eigen::Ref<const Eigen::Matrix<double, 7, 1>>& joint_velocities)
{
    std::lock_guard<std::mutex> lock(command_state_mtx_);
    command_joint_positions_ = joint_positions;
    command_joint_velocities_ = joint_velocities;
}

franka::Torques FrankaJointTrajectoryController::impedanceControlCallback(
    const franka::RobotState& state, franka::Duration /*period*/)
{
    {
        std::lock_guard<std::mutex> lock(current_state_mtx_);
        current_robot_state_ = state;
        current_joint_positions_ = state.q;
    }

    std::array<double, 7> coriolis = model_->coriolis(state);
    std::array<double, 7> tau_d_calculated;
    {
        // --- DO NOT MODIFY THIS, this is for safety --- //
        std::lock_guard<std::mutex> lock(command_state_mtx_);
        // validate command and state
        assert(command_joint_positions_.size() == 7);
        for (size_t i = 0; i < 7; i++)
        {
            // Checked twice doeesn't hurt
            if (abs(command_joint_positions_(i) - state.q[i]) > MAX_JOINT_DIFFERENCE_IMPEDENCE) 
            {
                throw std::runtime_error(
                    "Desired joint [" + std::to_string(i) + 
                    "] position [" + std::to_string(command_joint_positions_(i)) + 
                    " - actual position [" + std::to_string(state.q[i]) + "] is larger than tolerance: " +
                    std::to_string(MAX_JOINT_DIFFERENCE_IMPEDENCE));
            }
        }
        // ----------------------------------------------- //

        for (size_t i = 0; i < 7; i++)
        {
            tau_d_calculated[i] =
                k_gains_[i] * (command_joint_positions_(i) - state.q[i]) +
                d_gains_[i] * (command_joint_velocities_(i) - state.dq[i]) + coriolis[i];
        }
    }

    // Limit rate torque
    std::array<double, 7> tau_d_rate_limited = franka::limitRate(
        franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);

    return tau_d_rate_limited;
}

} // namespace csc376_franky
