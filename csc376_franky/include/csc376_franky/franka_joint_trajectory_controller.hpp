#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <thread>
#include <eigen3/Eigen/Dense>

#include <franka/model.h>
#include <franka/robot.h>

namespace csc376_franky
{

enum class ErrorCodes
{
    Success  = 1,
    JointSizeNotSeven = -1,
    JointDistancesAboveMaxJointDifference = -2,
    JointVelocitiesAboveMaximumAllowed = -3,
};

class FrankaJointTrajectoryController
{
public:
    FrankaJointTrajectoryController(const std::string& fci_host_ip);
    ErrorCodes runTrajectory(const Eigen::Ref<const Eigen::MatrixXd>& joint_trajectory, float dt);
    franka::RobotState getCurrentRobotState();
    std::array<double, 7> getCurrentJointPositions();
    void join();
private:
    std::unique_ptr<franka::Model> model_;
    std::unique_ptr<franka::Robot> robot_;
    std::array<double, 7> k_gains_;
    std::array<double, 7> d_gains_;
    
    void setCommandJointPositions(const Eigen::Ref<const Eigen::Vector<double, 7>>&  joint_positions);
    franka::Torques impedanceControlCallback(
        const franka::RobotState& state, franka::Duration /*period*/);
    std::thread control_thread_;

    std::mutex current_state_mtx_;
    franka::RobotState current_robot_state_;
    std::array<double, 7> current_joint_positions_;

    std::mutex command_state_mtx_;
    Eigen::Vector<double, 7> command_joint_positions_;

    static constexpr double MAX_JOINT_DIFFERENCE_ = 0.05;
    const std::array<double, 7> MAX_JOINT_VELOCITIES_ = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1}; // rad/s 

};

} // namespace csc376_franky
