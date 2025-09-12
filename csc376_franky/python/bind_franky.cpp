#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>
#include <franka/robot_state.h>
#include <franka/gripper_state.h>
#include "csc376_franky/franka_joint_trajectory_controller.hpp"
#include "csc376_franky/gripper.hpp"
#include "macros.hpp"

namespace py = pybind11;
using namespace pybind11::literals;  // to bring in the '_a' literal

#define ADD_FIELD_RO(obj_type, unused, name) .def_readonly(#name, &obj_type::name)
#define ADD_FIELDS_RO(obj, obj_type, ...) obj MAP_C1(ADD_FIELD_RO, obj_type, __VA_ARGS__);


// Define RobotState fields to expose (following your macro pattern)
// #define ROBOT_STATE_FIELDS \
//     q, dq, ddq_d, tau_J, dtau_J, tau_J_d, tau_ext_hat_filtered, \
//     O_T_EE, O_T_EE_d, F_T_EE, EE_T_K, m_ee, I_ee, F_x_Cee, \
//     elbow, elbow_d, elbow_c, delbow_c, ddelbow_c, \
//     joint_contact, cartesian_contact, joint_collision, cartesian_collision, \
//     K_F_ext_hat_K, O_F_ext_hat_K, theta, dtheta, \
//     current_errors, last_motion_errors, control_command_success_rate, \
//     robot_mode, time

#define GRIPPER_STATE_FIELDS width, max_width, is_grasped, temperature, time

void bind_error_codes(py::module &m) {
    py::enum_<csc376_franky::ErrorCodes>(m, "ErrorCodes")
        .value("Success", csc376_franky::ErrorCodes::Success)
        .value("JointSizeNotSeven", csc376_franky::ErrorCodes::JointSizeNotSeven)
        .value("JointDistancesAboveMaxJointDifference", csc376_franky::ErrorCodes::JointDistancesAboveMaxJointDifference)
        .value("JointVelocitiesAboveMaximumAllowed", csc376_franky::ErrorCodes::JointVelocitiesAboveMaximumAllowed)
        .value("DtIsNegative", csc376_franky::ErrorCodes::DtIsNegative)
        .value("DtIsSmallerThanAllowed", csc376_franky::ErrorCodes::DtIsSmallerThanAllowed)
        .export_values();
}

void bind_robot_state(py::module &m) {
    py::class_<franka::RobotState>(m, "RobotState")
        // Joint states
        .def_readonly("q", &franka::RobotState::q, 
                     "Measured joint position [rad] (7-element array)")
        .def_readonly("dq", &franka::RobotState::dq, 
                     "Measured joint velocity [rad/s] (7-element array)")
        .def_readonly("ddq_d", &franka::RobotState::ddq_d, 
                     "Desired joint acceleration [rad/s²] (7-element array)")
        
        // Torques
        .def_readonly("tau_J", &franka::RobotState::tau_J, 
                     "Measured link-side joint torque [Nm] (7-element array)")
        .def_readonly("dtau_J", &franka::RobotState::dtau_J, 
                     "Derivative of joint torque [Nm/s] (7-element array)")
        .def_readonly("tau_J_d", &franka::RobotState::tau_J_d, 
                     "Desired link-side joint torque [Nm] (7-element array)")
        .def_readonly("tau_ext_hat_filtered", &franka::RobotState::tau_ext_hat_filtered, 
                     "External torque, filtered [Nm] (7-element array)")
        
        // Cartesian poses (4x4 transformation matrices as 16-element arrays)
        .def_readonly("O_T_EE", &franka::RobotState::O_T_EE, 
                     "End-effector pose in base frame (16-element array)")
        .def_readonly("O_T_EE_d", &franka::RobotState::O_T_EE_d, 
                     "Desired end-effector pose (16-element array)")
        .def_readonly("F_T_EE", &franka::RobotState::F_T_EE, 
                     "Flange to end-effector transform (16-element array)")
        .def_readonly("EE_T_K", &franka::RobotState::EE_T_K, 
                     "End-effector to stiffness frame (16-element array)")
        
        // End-effector properties
        .def_readonly("m_ee", &franka::RobotState::m_ee, 
                     "End effector mass [kg]")
        .def_readonly("I_ee", &franka::RobotState::I_ee, 
                     "End effector inertia (9-element array)")
        .def_readonly("F_x_Cee", &franka::RobotState::F_x_Cee, 
                     "End effector center of mass (3-element array)")
        
        // Elbow configuration  
        .def_readonly("elbow", &franka::RobotState::elbow, 
                     "Elbow configuration (2-element array)")
        .def_readonly("elbow_d", &franka::RobotState::elbow_d, 
                     "Desired elbow configuration (2-element array)")
        .def_readonly("elbow_c", &franka::RobotState::elbow_c, 
                     "Commanded elbow configuration (2-element array)")
        .def_readonly("delbow_c", &franka::RobotState::delbow_c, 
                     "Commanded elbow velocity (2-element array)")
        .def_readonly("ddelbow_c", &franka::RobotState::ddelbow_c, 
                     "Commanded elbow acceleration (2-element array)")
        
        // Contact and collision (boolean arrays)
        .def_readonly("joint_contact", &franka::RobotState::joint_contact, 
                     "Joint contact states (7-element boolean array)")
        .def_readonly("cartesian_contact", &franka::RobotState::cartesian_contact, 
                     "Cartesian contact states (6-element boolean array)")
        .def_readonly("joint_collision", &franka::RobotState::joint_collision, 
                     "Joint collision states (7-element boolean array)")
        .def_readonly("cartesian_collision", &franka::RobotState::cartesian_collision, 
                     "Cartesian collision states (6-element boolean array)")
        
        // External forces
        .def_readonly("K_F_ext_hat_K", &franka::RobotState::K_F_ext_hat_K, 
                     "External wrench in stiffness frame (6-element array)")
        .def_readonly("O_F_ext_hat_K", &franka::RobotState::O_F_ext_hat_K, 
                     "External wrench in base frame (6-element array)")
        
        // Motor states
        .def_readonly("theta", &franka::RobotState::theta, 
                     "Motor positions [rad] (7-element array)")
        .def_readonly("dtheta", &franka::RobotState::dtheta, 
                     "Motor velocities [rad/s] (7-element array)")
        
        // Status and errors
        .def_readonly("current_errors", &franka::RobotState::current_errors, 
                     "Current robot errors")
        .def_readonly("last_motion_errors", &franka::RobotState::last_motion_errors, 
                     "Last motion errors")
        .def_readonly("control_command_success_rate", &franka::RobotState::control_command_success_rate, 
                     "Control command success rate")
        .def_readonly("robot_mode", &franka::RobotState::robot_mode, 
                     "Current robot mode")
        .def_readonly("time", &franka::RobotState::time, 
                     "Timestamp");
}

void bind_gripper_state(py::module &m){
    py::class_<franka::GripperState> gripper_state(m, "GripperState");
    ADD_FIELDS_RO(gripper_state, franka::GripperState, GRIPPER_STATE_FIELDS)
    gripper_state.def(
        py::pickle(
            [](const franka::GripperState &state) {  // __getstate__
                return PACK_TUPLE(state, GRIPPER_STATE_FIELDS);
            },
            [](const py::tuple &t) {  // __setstate__
                if (t.size() != COUNT(GRIPPER_STATE_FIELDS)) throw std::runtime_error("Invalid state!");
                return UNPACK_TUPLE(franka::GripperState, t, GRIPPER_STATE_FIELDS);
            }));
}

void bind_franka_trajectory_controller(py::module &m) {
    py::class_<csc376_franky::FrankaJointTrajectoryController>(m, "FrankaJointTrajectoryController")
        .def(py::init<const std::string&>(), 
             py::arg("fci_host_ip"),
             "Constructor - connects to robot at specified IP address")
        .def_static("setupSignalHandler", &csc376_franky::FrankaJointTrajectoryController::setupSignalHandler)
        // Control methods
        .def("join", &csc376_franky::FrankaJointTrajectoryController::join,
             "Wait for the control thread to finish")
        .def("stop", &csc376_franky::FrankaJointTrajectoryController::stop,
             "Stop all robot functions, including the currently running trajectory")
        .def("run_trajectory", &csc376_franky::FrankaJointTrajectoryController::runTrajectory,
             py::arg("joint_trajectory"), py::arg("dt"),
             "Execute joint trajectory (N×7 matrix, dt in seconds). Returns ErrorCode.")
        
        // State access methods
        .def("get_current_joint_positions", 
             [](csc376_franky::FrankaJointTrajectoryController& self) {
                 auto pos = self.getCurrentJointPositions();
                 return py::cast(pos);  // Let pybind11 handle the conversion
             },
             "Get current joint positions")
        .def("get_current_robot_state", 
             &csc376_franky::FrankaJointTrajectoryController::getCurrentRobotState,
             "Get the current complete robot state");
}

void bind_gripper(py::module &m)
{
    py::class_<franky::Gripper>(m, "Gripper")
    .def(py::init<const std::string &>(), "fci_hostname"_a)
    .def(
        "grasp",
        &franky::Gripper::grasp,
        "width"_a,
        "speed"_a,
        "force"_a,
        "epsilon_inner"_a = 0.005,
        "epsilon_outer"_a = 0.005,
        py::call_guard<py::gil_scoped_release>())
    .def(
        "grasp_async",
        &franky::Gripper::graspAsync,
        "width"_a,
        "speed"_a,
        "force"_a,
        "epsilon_inner"_a = 0.005,
        "epsilon_outer"_a = 0.005,
        py::call_guard<py::gil_scoped_release>())
    .def("move", &franky::Gripper::move, "width"_a, "speed"_a, py::call_guard<py::gil_scoped_release>())
    .def("move_async", &franky::Gripper::moveAsync, "width"_a, "speed"_a, py::call_guard<py::gil_scoped_release>())
    .def("open", &franky::Gripper::open, "speed"_a, py::call_guard<py::gil_scoped_release>())
    .def("open_async", &franky::Gripper::openAsync, "speed"_a, py::call_guard<py::gil_scoped_release>())
    .def("homing", &franky::Gripper::homing, py::call_guard<py::gil_scoped_release>())
    .def("homing_async", &franky::Gripper::homingAsync, py::call_guard<py::gil_scoped_release>())
    .def("stop", &franky::Gripper::stop, py::call_guard<py::gil_scoped_release>())
    .def("stop_async", &franky::Gripper::stopAsync, py::call_guard<py::gil_scoped_release>())
    .def_property_readonly("state", &franky::Gripper::state, py::call_guard<py::gil_scoped_release>())
    .def_property_readonly("server_version", reinterpret_cast<uint16_t (franky::Gripper::*)()>(&franky::Gripper::serverVersion))
    .def_property_readonly("width", &franky::Gripper::width, py::call_guard<py::gil_scoped_release>())
    .def_property_readonly("is_grasped", &franky::Gripper::is_grasped, py::call_guard<py::gil_scoped_release>())
    .def_property_readonly("max_width", &franky::Gripper::max_width, py::call_guard<py::gil_scoped_release>());
}

PYBIND11_MODULE(csc376_franky, m) 
{
    m.doc() = "Python bindings for CSC376 Franka Joint Trajectory Controller";
    bind_error_codes(m);    
    bind_robot_state(m);
    bind_franka_trajectory_controller(m);
    bind_gripper_state(m);
    bind_gripper(m);
}
