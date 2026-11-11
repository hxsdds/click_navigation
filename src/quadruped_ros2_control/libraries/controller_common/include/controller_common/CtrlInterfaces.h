//
// Created by biao on 24-9-10.
//

#ifndef INTERFACE_H
#define INTERFACE_H

#include <vector>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <control_input_msgs/msg/inputs.hpp>

/**
 * Tunable limits of the trot gait.
 *
 * Every field is exposed as a ROS parameter of unitree_guide_controller
 * (prefix "trot_"), so the gait can be re-tuned at runtime with
 * `ros2 param set` instead of rebuilding. Defaults are the conservative
 * values that were verified in Gazebo; see 机器狗运动稳定性修复报告.md.
 */
struct TrotParams
{
    double vx_max = 0.30;            // m/s, forward/backward body speed
    double vy_max = 0.08;            // m/s, lateral body speed
    double yaw_max = 0.15;           // rad/s, yaw rate
    double lin_accel = 0.35;         // m/s^2, slew limit on the linear command
    double entry_ramp = 1.50;        // s, ramp from zero after entering trot
    double yaw_accel = 2.50;         // rad/s^2, slew limit on the yaw command
    double turn_fade_speed = 0.50;   // fraction of vx_max where turning fades to zero
    double turn_lean = 0.0;          // m/s of lateral weight shift per rad/s of yaw
    double gait_period = 0.70;       // s, one full trot cycle
    double gait_duty = 0.75;         // stance ratio of the gait cycle
    double gait_height = 0.05;       // m, swing foot clearance
    double yaw_step_ratio = 0.10;    // start stepping above this fraction of yaw_max
    double yaw_error_max = 0.30;     // rad, band that bounds the yaw reference
    double turn_guard_tilt = 0.25;   // rad, pause turning above this body tilt
    double turn_guard_release = 0.12; // rad, resume turning below this tilt
    double turn_budget = 0.45;       // rad of rotation allowed per continuous turn
    double tilt_soft = 0.20;         // rad, start derating the command
    double tilt_hard = 0.45;         // rad, fall back to fixed stand
};

struct CtrlInterfaces
{
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_torque_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_position_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_velocity_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_kp_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_kd_command_interface_;


    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_effort_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_position_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_velocity_state_interface_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    imu_state_interface_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    foot_force_state_interface_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    odom_state_interface_;


    control_input_msgs::msg::Inputs control_inputs_;
    int frequency_{};

    /** Limits of the trot gait, kept in sync with the controller parameters. */
    TrotParams trot_;

    CtrlInterfaces() = default;

    void clear()
    {
        joint_torque_command_interface_.clear();
        joint_position_command_interface_.clear();
        joint_velocity_command_interface_.clear();
        joint_kd_command_interface_.clear();
        joint_kp_command_interface_.clear();

        joint_effort_state_interface_.clear();
        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();

        imu_state_interface_.clear();
        imu_state_interface_.clear();
        foot_force_state_interface_.clear();
    }
};

#endif //INTERFACE_H
