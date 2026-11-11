//
// Created by tlab-uav on 24-9-18.
//

#include "unitree_guide_controller/FSM/StateTrotting.h"

#include <algorithm>
#include <cmath>

#include <unitree_guide_controller/common/mathTools.h>
#include <unitree_guide_controller/control/CtrlComponent.h>
#include <unitree_guide_controller/control/Estimator.h>
#include <unitree_guide_controller/gait/WaveGenerator.h>

StateTrotting::StateTrotting(CtrlInterfaces &ctrl_interfaces,
                             CtrlComponent &ctrl_component) : FSMState(FSMStateName::TROTTING, "trotting",
                                                                       ctrl_interfaces),
                                                              estimator_(ctrl_component.estimator_),
                                                              robot_model_(ctrl_component.robot_model_),
                                                              balance_ctrl_(ctrl_component.balance_ctrl_),
                                                              wave_generator_(ctrl_component.wave_generator_),
                                                              gait_generator_(ctrl_component) {
    gait_height_ = ctrl_interfaces_.trot_.gait_height;
    Kpp = Vec3(70, 70, 70).asDiagonal();
    Kdp = Vec3(10, 10, 10).asDiagonal();
    kp_w_ = 780;
    Kd_w_ = Vec3(70, 70, 70).asDiagonal();
    Kp_swing_ = Vec3(400, 400, 400).asDiagonal();
    Kd_swing_ = Vec3(10, 10, 10).asDiagonal();

    // The limits are re-read from ctrl_interfaces_.trot_ on every cycle (see
    // getUserCmd); the values here only seed the command-axis vectors.
    const auto &trot = ctrl_interfaces_.trot_;
    v_x_limit_ << -trot.vx_max, trot.vx_max;
    v_y_limit_ << -trot.vy_max, trot.vy_max;
    w_yaw_limit_ << -trot.yaw_max, trot.yaw_max;
    dt_ = 1.0 / ctrl_interfaces_.frequency_;
}

void StateTrotting::enter() {
    // Pick up the current gait clock before restart(): FeetEndCalc caches the
    // stance and swing durations when it is restarted.
    const auto &trot = ctrl_interfaces_.trot_;
    gait_height_ = trot.gait_height;
    wave_generator_->setGaitParams(trot.gait_period, trot.gait_duty);

    pcd_ = estimator_->getPosition();
    pcd_(2) = -estimator_->getFeetPos2Body()(2, 0);
    v_cmd_body_.setZero();
    yaw_cmd_ = estimator_->getYaw();
    d_yaw_cmd_ = 0.0;
    d_yaw_cmd_past_ = 0.0;
    turn_guard_active_ = false;
    turn_budget_used_ = 0.0;
    turn_reference_active_ = false;
    entry_time_ = 0.0;
    step_latch_ = 0.0;
    Rd = rotz(yaw_cmd_);
    w_cmd_global_.setZero();

    ctrl_interfaces_.control_inputs_.command = 0;
    wave_generator_->resetToStance();
    gait_generator_.restart();
}

void StateTrotting::run(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/) {
    entry_time_ += dt_;
    pos_body_ = estimator_->getPosition();
    vel_body_ = estimator_->getVelocity();

    B2G_RotMat = estimator_->getRotation();
    G2B_RotMat = B2G_RotMat.transpose();

    getUserCmd();
    calcCmd();

    gait_generator_.setGait(vel_target_.segment(0, 2), w_cmd_global_(2), gait_height_);
    gait_generator_.generate(pos_feet_global_goal_, vel_feet_global_goal_);

    calcTau();
    calcQQd();

    // Hysteresis: once the gait starts stepping it keeps stepping for at least
    // one full cycle. Without it, the stop condition (position/velocity error
    // falling below threshold) makes the status flicker between WAVE_ALL and
    // STANCE_ALL at the end of a manoeuvre, and that mid-stride collapse is
    // what tips the robot over.
    const double cycle = ctrl_interfaces_.trot_.gait_period;
    if (checkStepOrNot()) {
        step_latch_ = std::max(step_latch_, cycle);
        wave_generator_->status_ = WaveStatus::WAVE_ALL;
    } else {
        step_latch_ = std::max(0.0, step_latch_ - dt_);
        wave_generator_->status_ = (step_latch_ > 0.0) ? WaveStatus::WAVE_ALL : WaveStatus::STANCE_ALL;
    }

    calcGain();
}

void StateTrotting::exit() {
    wave_generator_->status_ = WaveStatus::SWING_ALL;
}

FSMStateName StateTrotting::checkChange() {
    // Failsafe: if the body attitude leaves the recoverable range, return to
    // fixed stand instead of continuing to apply a walking gait.
    const Vec3 rpy = rotMatToRPY(estimator_->getRotation());
    const double tilt_hard = ctrl_interfaces_.trot_.tilt_hard;
    if (fabs(rpy(0)) > tilt_hard || fabs(rpy(1)) > tilt_hard) {
        return FSMStateName::FIXEDSTAND;
    }

    switch (ctrl_interfaces_.control_inputs_.command) {
        case 1:
            return FSMStateName::PASSIVE;
        case 2:
            return FSMStateName::FIXEDSTAND;
        default:
            return FSMStateName::TROTTING;
    }
}

void StateTrotting::getUserCmd() {
    const auto &trot = ctrl_interfaces_.trot_;
    v_x_limit_ << -trot.vx_max, trot.vx_max;
    v_y_limit_ << -trot.vy_max, trot.vy_max;
    w_yaw_limit_ << -trot.yaw_max, trot.yaw_max;

    // Back the whole command off as soon as the body starts to tilt. The gait
    // keeps the centre of mass inside its support polygon only while roll and
    // pitch stay small, so derating early is what allows a high top speed
    // without falling over at the first bump.
    const Vec3 rpy = rotMatToRPY(estimator_->getRotation());
    const double tilt = std::max(std::fabs(rpy(0)), std::fabs(rpy(1)));
    double tilt_scale = 1.0;
    if (tilt > trot.tilt_soft) {
        const double span = std::max(1e-3, trot.tilt_hard - trot.tilt_soft);
        tilt_scale = std::clamp(1.0 - 0.9 * (tilt - trot.tilt_soft) / span, 0.1, 1.0);
    }

    /* Movement */
    // Ramp the command in over the first moments of trot: stepping straight to
    // full speed right after gait entry is what tips the robot over at high
    // vx_max, even though the same speed is stable in steady state.
    const double entry_scale = std::min(1.0, entry_time_ / std::max(1e-3, trot.entry_ramp));
    const double target_vx = tilt_scale * entry_scale * invNormalize(ctrl_interfaces_.control_inputs_.ly,
                                                       v_x_limit_(0), v_x_limit_(1));
    const double target_vy = tilt_scale * entry_scale * -invNormalize(ctrl_interfaces_.control_inputs_.lx,
                                                        v_y_limit_(0), v_y_limit_(1));

    // Slew-rate limit the body command to avoid a step impulse at gait entry
    // or when the direction is reversed.
    v_cmd_body_(0) = saturation(target_vx,
                                Vec2(v_cmd_body_(0) - trot.lin_accel * dt_,
                                     v_cmd_body_(0) + trot.lin_accel * dt_));
    v_cmd_body_(1) = saturation(target_vy,
                                Vec2(v_cmd_body_(1) - trot.lin_accel * dt_,
                                     v_cmd_body_(1) + trot.lin_accel * dt_));
    v_cmd_body_(2) = 0;

    /* Turning */
    const double linear_command_ratio = std::min(
            1.0, std::hypot(v_cmd_body_(0) / std::max(1e-3, trot.vx_max),
                            v_cmd_body_(1) / std::max(1e-3, trot.vy_max)));
    // Turning while translating is by far the most demanding case for this
    // gait: the diagonal support line is already loaded laterally, and adding
    // yaw on top of that is what tips the robot over. Turning therefore fades
    // out with speed and is fully suppressed above turn_fade_speed * vx_max;
    // a walking robot steers like a car (slow down to turn), which measured
    // stable where simultaneous fast walking and turning was not.
    const double turn_scale =
            std::clamp(1.0 - linear_command_ratio / std::max(1e-3, trot.turn_fade_speed), 0.0, 1.0);

    // Keep a configurable turn budget as a secondary limit. The simulator's
    // Go2 profile permits a full reorientation after correcting the estimator
    // coordinate frames. Releasing the turn input refills the budget.
    if (tilt > trot.turn_guard_tilt) {
        turn_guard_active_ = true;
    } else if (turn_guard_active_ && tilt < trot.turn_guard_release) {
        turn_guard_active_ = false;
    }
    const bool turning_input = std::fabs(ctrl_interfaces_.control_inputs_.rx) > 0.05;
    if (!turning_input) {
        turn_budget_used_ = 0.0;   // released: a fresh turn may use the full budget
    }
    const bool budget_exceeded = turn_budget_used_ >= trot.turn_budget;
    const double yaw_authority = (turn_guard_active_ || budget_exceeded) ? 0.0 : 1.0;

    const double target_yaw_cmd = tilt_scale * turn_scale * yaw_authority *
                                  -invNormalize(ctrl_interfaces_.control_inputs_.rx,
                                                w_yaw_limit_(0), w_yaw_limit_(1));
    // A stopped turn must not keep chasing up to 0.30 rad of accumulated
    // reference lead while the navigator starts translating. Rebase only on
    // the falling edge: during an active turn retain bounded position feedback.
    const bool turn_reference_active = std::fabs(target_yaw_cmd) > 1e-4;
    if (turn_reference_active_ && !turn_reference_active) {
        yaw_cmd_ = estimator_->getYaw();
    }
    turn_reference_active_ = turn_reference_active;
    d_yaw_cmd_ = saturation(target_yaw_cmd,
                            Vec2(d_yaw_cmd_past_ - trot.yaw_accel * dt_,
                                 d_yaw_cmd_past_ + trot.yaw_accel * dt_));
    d_yaw_cmd_past_ = d_yaw_cmd_;
    turn_budget_used_ += std::fabs(d_yaw_cmd_) * dt_;
}

void StateTrotting::calcCmd() {
    /* Movement */
    // Turning lean: while yawing, shift the commanded body position sideways
    // (toward the support diagonal) by a fraction of the yaw command. Measured
    // motivation: a trot that yaws while walking lifts its roll excursion from
    // ~0.03 rad to >0.45 rad within a couple of seconds, which is what tips it
    // over; a lateral weight shift is the usual way to buy roll margin.
    Vec3 v_cmd_body = v_cmd_body_;
    if (std::fabs(d_yaw_cmd_) > 1e-3) {
        // turn_lean is the peak lateral speed (m/s); it ramps in with the yaw
        // command, reaching full value at 0.05 rad/s.
        const double lean_scale = std::clamp(d_yaw_cmd_ / 0.05, -1.0, 1.0);
        v_cmd_body(1) += std::clamp(ctrl_interfaces_.trot_.turn_lean * lean_scale, -0.08, 0.08);
    }
    vel_target_ = B2G_RotMat * v_cmd_body;

    vel_target_(0) =
            saturation(vel_target_(0), Vec2(vel_body_(0) - 0.2, vel_body_(0) + 0.2));
    vel_target_(1) =
            saturation(vel_target_(1), Vec2(vel_body_(1) - 0.2, vel_body_(1) + 0.2));

    pcd_(0) = saturation(pcd_(0) + vel_target_(0) * dt_,
                         Vec2(pos_body_(0) - 0.05, pos_body_(0) + 0.05));
    pcd_(1) = saturation(pcd_(1) + vel_target_(1) * dt_,
                         Vec2(pos_body_(1) - 0.05, pos_body_(1) + 0.05));

    vel_target_(2) = 0;

    /* Turning */
    // Integrate the yaw reference, then keep it inside a band around the
    // measured yaw. Two failure modes bracket this design:
    //   * a free-running integral winds up as soon as the body lags the
    //     command, and the attitude controller ends up fighting the
    //     accumulated error;
    //   * a pure rate reference (yaw_cmd = measured + lead) removes the
    //     position feedback entirely, so the gait over-rotates and the robot
    //     trips after about 60 degrees.
    // The band keeps a position reference the attitude loop can push back
    // against while capping how far it can run away.
    yaw_cmd_ = yaw_cmd_ + d_yaw_cmd_ * dt_;
    const double yaw_est = estimator_->getYaw();
    const double yaw_error = std::atan2(std::sin(yaw_cmd_ - yaw_est), std::cos(yaw_cmd_ - yaw_est));
    const double yaw_band = ctrl_interfaces_.trot_.yaw_error_max;
    if (yaw_error > yaw_band) {
        yaw_cmd_ = yaw_est + yaw_band;
    } else if (yaw_error < -yaw_band) {
        yaw_cmd_ = yaw_est - yaw_band;
    }
    Rd = rotz(yaw_cmd_);
    w_cmd_global_(2) = d_yaw_cmd_;
}

void StateTrotting::calcTau() {
    pos_error_ = pcd_ - pos_body_;
    vel_error_ = vel_target_ - vel_body_;

    Vec3 dd_pcd = Kpp * pos_error_ + Kdp * vel_error_;
    Vec3 d_wbd = kp_w_ * rotMatToExp(Rd * G2B_RotMat) +
                 Kd_w_ * (w_cmd_global_ - estimator_->getGyroGlobal());

    dd_pcd(0) = saturation(dd_pcd(0), Vec2(-3, 3));
    dd_pcd(1) = saturation(dd_pcd(1), Vec2(-3, 3));
    dd_pcd(2) = saturation(dd_pcd(2), Vec2(-5, 5));

    d_wbd(0) = saturation(d_wbd(0), Vec2(-40, 40));
    d_wbd(1) = saturation(d_wbd(1), Vec2(-40, 40));
    d_wbd(2) = saturation(d_wbd(2), Vec2(-10, 10));

    const Vec34 pos_feet_body_global = estimator_->getFeetPos2Body();
    Vec34 force_feet_global =
            -balance_ctrl_->calF(dd_pcd, d_wbd, B2G_RotMat, pos_feet_body_global, wave_generator_->contact_);


    Vec34 pos_feet_global = estimator_->getFeetPos();
    Vec34 vel_feet_global = estimator_->getFeetVel();

    for (int i(0); i < 4; ++i) {
        if (wave_generator_->contact_(i) == 0) {
            force_feet_global.col(i) =
                    Kp_swing_ * (pos_feet_global_goal_.col(i) - pos_feet_global.col(i)) +
                    Kd_swing_ * (vel_feet_global_goal_.col(i) - vel_feet_global.col(i));
        }
    }

    Vec34 force_feet_body_ = G2B_RotMat * force_feet_global;

    std::vector<KDL::JntArray> current_joints = robot_model_->current_joint_pos_;
    for (int i = 0; i < 4; i++) {
        KDL::JntArray torque = robot_model_->getTorque(force_feet_body_.col(i), i);
        for (int j = 0; j < 3; j++) {
            std::ignore = ctrl_interfaces_.joint_torque_command_interface_[i * 3 + j].get().set_value(torque(j));
        }
    }
}

void StateTrotting::calcQQd() {
    const std::vector<KDL::Frame> pos_feet_body = robot_model_->getFeet2BPositions();

    Vec34 pos_feet_target, vel_feet_target;
    for (int i(0); i < 4; ++i) {
        pos_feet_target.col(i) = G2B_RotMat * (pos_feet_global_goal_.col(i) - pos_body_);
        vel_feet_target.col(i) = G2B_RotMat * (vel_feet_global_goal_.col(i) - vel_body_);
    }

    Vec12 q_goal = robot_model_->getQ(pos_feet_target);
    Vec12 qd_goal = robot_model_->getQd(pos_feet_body, vel_feet_target);
    for (int i = 0; i < 12; i++) {
        std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(q_goal(i));
        std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(qd_goal(i));
    }
}

void StateTrotting::calcGain() const {
    for (int i(0); i < 4; ++i) {
        if (wave_generator_->contact_(i) == 0) {
            // swing gain
            for (int j = 0; j < 3; j++) {
                std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i * 3 + j].get().set_value(3.0);
                std::ignore = ctrl_interfaces_.joint_kd_command_interface_[i * 3 + j].get().set_value(2.0);
            }
        } else {
            // stable gain
            for (int j = 0; j < 3; j++) {
                std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i * 3 + j].get().set_value(0.8);
                std::ignore = ctrl_interfaces_.joint_kd_command_interface_[i * 3 + j].get().set_value(0.8);
            }
        }
    }
}

bool StateTrotting::checkStepOrNot() {
    // The yaw threshold is expressed against the configured yaw limit: it used
    // to be a fixed 0.05 rad/s against a 0.03 rad/s limit, so a pure turn
    // command never lifted a foot and the robot could only be dragged around
    // by stance friction.
    const auto &trot = ctrl_interfaces_.trot_;
    const double yaw_step = std::max(0.02, trot.yaw_step_ratio * trot.yaw_max);
    if (fabs(v_cmd_body_(0)) > 0.01 || fabs(v_cmd_body_(1)) > 0.01 ||
        fabs(pos_error_(0)) > 0.05 || fabs(pos_error_(1)) > 0.05 ||
        fabs(vel_error_(0)) > 0.03 || fabs(vel_error_(1)) > 0.03 ||
        fabs(d_yaw_cmd_) > yaw_step) {
        return true;
    }
    return false;
}
