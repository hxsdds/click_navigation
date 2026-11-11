//
// Created by tlab-uav on 24-9-6.
//

#include "unitree_guide_controller/UnitreeGuideController.h"

#include <algorithm>

#include <unitree_guide_controller/common/mathTools.h>
#include <unitree_guide_controller/gait/WaveGenerator.h>
#include "unitree_guide_controller/robot/QuadrupedRobot.h"

namespace unitree_guide_controller
{
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration UnitreeGuideController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : command_interface_types_)
            {
                if (!command_prefix_.empty())
                {
                    conf.names.push_back(command_prefix_ + "/" + joint_name + "/" += interface_type);
                }
                else
                {
                    conf.names.push_back(joint_name + "/" += interface_type);
                }
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration UnitreeGuideController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }

        for (const auto& interface_type : imu_interface_types_)
        {
            conf.names.push_back(imu_name_ + "/" += interface_type);
        }

        return conf;
    }

    controller_interface::return_type UnitreeGuideController::
    update(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // auto now = std::chrono::steady_clock::now();
        // std::chrono::duration<double> time_diff = now - last_update_time_;
        // last_update_time_ = now;
        //
        // // Calculate the frequency
        // update_frequency_ = 1.0 / time_diff.count();
        // RCLCPP_INFO(get_node()->get_logger(), "Update frequency: %f Hz", update_frequency_);
        if (ctrl_component_.robot_model_ == nullptr)
        {
            return controller_interface::return_type::OK;
        }

        // A callback stages a whole command. Only the update thread writes
        // the FSM input, so it cannot observe half an old turn and half a
        // new drive command. Copy only on a new message: the FSM consumes
        // command events by resetting command to zero.
        {
            std::lock_guard<std::mutex> lock(control_input_mutex_);
            if (has_pending_control_input_)
            {
                const auto &msg = pending_control_input_;
                ctrl_interfaces_.control_inputs_.command = msg.command;
                ctrl_interfaces_.control_inputs_.lx = msg.lx;
                ctrl_interfaces_.control_inputs_.ly = msg.ly;
                ctrl_interfaces_.control_inputs_.rx = msg.rx;
                ctrl_interfaces_.control_inputs_.ry = msg.ry;
                has_pending_control_input_ = false;
            }
        }

        ctrl_component_.robot_model_->update();
        ctrl_component_.wave_generator_->setTime(time.seconds());
        ctrl_component_.wave_generator_->update();
        ctrl_component_.estimator_->update();

        if (mode_ == FSMMode::NORMAL)
        {
            current_state_->run(time, period);
            next_state_name_ = current_state_->checkChange();
            if (next_state_name_ != current_state_->state_name)
            {
                mode_ = FSMMode::CHANGE;
                next_state_ = getNextState(next_state_name_);
                RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s",
                            current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str());
            }
        }
        else if (mode_ == FSMMode::CHANGE)
        {
            current_state_->exit();
            current_state_ = next_state_;

            current_state_->enter();
            mode_ = FSMMode::NORMAL;
        }

        if (debug_publisher_ && ctrl_component_.estimator_ && (++debug_counter_ % 10) == 0)
        {
            const Vec3 rpy = rotMatToRPY(ctrl_component_.estimator_->getRotation());
            std_msgs::msg::Float64MultiArray message;
            message.data = {
                ctrl_component_.estimator_->getYaw(),
                ctrl_component_.estimator_->getDYaw(),
                rpy(0),
                rpy(1),
                static_cast<double>(static_cast<int>(current_state_->state_name)),
                static_cast<double>(ctrl_interfaces_.control_inputs_.rx),
                static_cast<double>(static_cast<int>(ctrl_component_.wave_generator_->status_)),
                ctrl_component_.wave_generator_->phase_(0),
                ctrl_component_.wave_generator_->phase_(1),
                ctrl_component_.wave_generator_->phase_(2),
                ctrl_component_.wave_generator_->phase_(3),
                static_cast<double>(ctrl_component_.wave_generator_->contact_(0)),
                static_cast<double>(ctrl_component_.wave_generator_->contact_(1)),
                static_cast<double>(ctrl_component_.wave_generator_->contact_(2)),
                static_cast<double>(ctrl_component_.wave_generator_->contact_(3))
            };
            debug_publisher_->publish(message);
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn UnitreeGuideController::on_init()
    {
        try
        {
            joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
            command_interface_types_ =
                auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            state_interface_types_ =
                auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

            // imu sensor
            imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
            base_name_ = auto_declare<std::string>("base_name", base_name_);
            imu_interface_types_ = auto_declare<std::vector<std::string>>("imu_interfaces", state_interface_types_);
            command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
            feet_names_ =
                auto_declare<std::vector<std::string>>("feet_names", feet_names_);

            // pose parameters
            down_pos_ = auto_declare<std::vector<double>>("down_pos", down_pos_);
            stand_pos_ = auto_declare<std::vector<double>>("stand_pos", stand_pos_);
            stand_kp_ = auto_declare<double>("stand_kp", stand_kp_);
            stand_kd_ = auto_declare<double>("stand_kd", stand_kd_);

            get_node()->get_parameter("update_rate", ctrl_interfaces_.frequency_);
            RCLCPP_INFO(get_node()->get_logger(), "Controller Manager Update Rate: %d Hz", ctrl_interfaces_.frequency_);

            declareTrotParameters();

            ctrl_component_.estimator_ = std::make_shared<Estimator>(ctrl_interfaces_, ctrl_component_);
        }
        catch (const std::exception& e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UnitreeGuideController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        control_input_subscription_ = get_node()->create_subscription<control_input_msgs::msg::Inputs>(
            "/control_input", 10, [this](const control_input_msgs::msg::Inputs::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(control_input_mutex_);
                pending_control_input_ = *msg;
                has_pending_control_input_ = true;
            });

        robot_description_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                ctrl_component_.robot_model_ = std::make_shared<QuadrupedRobot>(
                    ctrl_interfaces_, msg->data, feet_names_, base_name_);
                ctrl_component_.balance_ctrl_ = std::make_shared<BalanceCtrl>(ctrl_component_.robot_model_);
            });

        // Gait clock defaults come from the trot_* parameters so the gait can be
        // re-tuned without touching the source. The generator re-reads them on
        // every trot entry (see StateTrotting::enter).
        ctrl_component_.wave_generator_ = std::make_shared<WaveGenerator>(
            ctrl_interfaces_.trot_.gait_period, ctrl_interfaces_.trot_.gait_duty,
            Vec4(0, 0.5, 0.5, 0));

        // Re-read the trot limits whenever a parameter changes, so the gait can
        // be tuned live with `ros2 param set` while the simulation runs.
        trot_parameter_callback_ = get_node()->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &parameters)
            {
                return applyTrotParameters(parameters);
            });

        // Introspection topic: compare the controller's own attitude/yaw with
        // the ground-truth /odom when diagnosing turning behaviour.
        debug_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/trot_state", rclcpp::QoS(10));

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    UnitreeGuideController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // clear out vectors in case of restart
        ctrl_interfaces_.clear();

        // assign command interfaces
        for (auto& interface : command_interfaces_)
        {
            std::string interface_name = interface.get_interface_name();
            if (const size_t pos = interface_name.find('/'); pos != std::string::npos)
            {
                command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
            }
            else
            {
                command_interface_map_[interface_name]->push_back(interface);
            }
        }

        // assign state interfaces
        for (auto& interface : state_interfaces_)
        {
            if (interface.get_prefix_name() == imu_name_)
            {
                ctrl_interfaces_.imu_state_interface_.emplace_back(interface);
            }
            else
            {
                state_interface_map_[interface.get_interface_name()]->push_back(interface);
            }
        }

        // Create FSM List
        state_list_.passive = std::make_shared<StatePassive>(ctrl_interfaces_);
        state_list_.fixedDown = std::make_shared<StateFixedDown>(ctrl_interfaces_, down_pos_, stand_kp_, stand_kd_);
        state_list_.fixedStand = std::make_shared<StateFixedStand>(ctrl_interfaces_, stand_pos_, stand_kp_, stand_kd_);
        state_list_.swingTest = std::make_shared<StateSwingTest>(ctrl_interfaces_, ctrl_component_);
        state_list_.freeStand = std::make_shared<StateFreeStand>(ctrl_interfaces_, ctrl_component_);
        state_list_.balanceTest = std::make_shared<StateBalanceTest>(ctrl_interfaces_, ctrl_component_);
        state_list_.trotting = std::make_shared<StateTrotting>(ctrl_interfaces_, ctrl_component_);

        // Initialize FSM
        current_state_ = state_list_.passive;
        current_state_->enter();
        next_state_ = current_state_;
        next_state_name_ = current_state_->state_name;
        mode_ = FSMMode::NORMAL;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UnitreeGuideController::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    UnitreeGuideController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    UnitreeGuideController::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    UnitreeGuideController::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    void UnitreeGuideController::declareTrotParameters()
    {
        auto &trot = ctrl_interfaces_.trot_;
        trot.vx_max = auto_declare<double>("trot_vx_max", trot.vx_max);
        trot.vy_max = auto_declare<double>("trot_vy_max", trot.vy_max);
        trot.yaw_max = auto_declare<double>("trot_yaw_max", trot.yaw_max);
        trot.lin_accel = auto_declare<double>("trot_lin_accel", trot.lin_accel);
trot.entry_ramp = auto_declare<double>("trot_entry_ramp", trot.entry_ramp);
        trot.yaw_accel = auto_declare<double>("trot_yaw_accel", trot.yaw_accel);
        trot.turn_fade_speed = auto_declare<double>("trot_turn_fade_speed", trot.turn_fade_speed);
        trot.turn_lean = auto_declare<double>("trot_turn_lean", trot.turn_lean);
        trot.gait_period = auto_declare<double>("trot_gait_period", trot.gait_period);
        trot.gait_duty = auto_declare<double>("trot_gait_duty", trot.gait_duty);
        trot.gait_height = auto_declare<double>("trot_gait_height", trot.gait_height);
        trot.yaw_step_ratio = auto_declare<double>("trot_yaw_step_ratio", trot.yaw_step_ratio);
        trot.yaw_error_max = auto_declare<double>("trot_yaw_error_max", trot.yaw_error_max);
trot.turn_guard_tilt = auto_declare<double>("trot_turn_guard_tilt", trot.turn_guard_tilt);
trot.turn_guard_release = auto_declare<double>("trot_turn_guard_release", trot.turn_guard_release);
trot.turn_budget = auto_declare<double>("trot_turn_budget", trot.turn_budget);
        trot.tilt_soft = auto_declare<double>("trot_tilt_soft", trot.tilt_soft);
        trot.tilt_hard = auto_declare<double>("trot_tilt_hard", trot.tilt_hard);

        RCLCPP_INFO(get_node()->get_logger(),
                    "Trot limits: vx=%.2f m/s vy=%.2f m/s yaw=%.2f rad/s accel=%.2f/%.2f "
                    "gait=%.2f s duty=%.2f height=%.2f m tilt=%.2f/%.2f rad yaw_band=%.2f rad",
                    trot.vx_max, trot.vy_max, trot.yaw_max, trot.lin_accel, trot.yaw_accel,
                    trot.gait_period, trot.gait_duty, trot.gait_height,
                    trot.tilt_soft, trot.tilt_hard, trot.yaw_error_max);
    }

    rcl_interfaces::msg::SetParametersResult UnitreeGuideController::applyTrotParameters(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        auto &trot = ctrl_interfaces_.trot_;
        for (const auto &parameter : parameters)
        {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                continue;
            }
            const std::string &name = parameter.get_name();
            const double value = parameter.as_double();
            double applied = value;
            bool known = true;

            if (name == "trot_vx_max") trot.vx_max = applied = std::clamp(value, 0.02, 1.20);
            else if (name == "trot_vy_max") trot.vy_max = applied = std::clamp(value, 0.01, 0.60);
            else if (name == "trot_yaw_max") trot.yaw_max = applied = std::clamp(value, 0.05, 2.00);
            else if (name == "trot_lin_accel") trot.lin_accel = applied = std::clamp(value, 0.05, 3.00);
            else if (name == "trot_entry_ramp") trot.entry_ramp = applied = std::clamp(value, 0.00, 3.00);
            else if (name == "trot_yaw_accel") trot.yaw_accel = applied = std::clamp(value, 0.50, 30.0);
            else if (name == "trot_turn_fade_speed")
                trot.turn_fade_speed = applied = std::clamp(value, 0.05, 1.00);
            else if (name == "trot_turn_lean")
                trot.turn_lean = applied = std::clamp(value, -0.20, 0.20);
            else if (name == "trot_gait_period") trot.gait_period = applied = std::clamp(value, 0.30, 1.20);
            else if (name == "trot_gait_duty") trot.gait_duty = applied = std::clamp(value, 0.45, 0.90);
            else if (name == "trot_gait_height") trot.gait_height = applied = std::clamp(value, 0.02, 0.15);
            else if (name == "trot_yaw_step_ratio") trot.yaw_step_ratio = applied = std::clamp(value, 0.02, 0.90);
            else if (name == "trot_yaw_error_max") trot.yaw_error_max = applied = std::clamp(value, 0.05, 1.00);
            else if (name == "trot_turn_guard_tilt") trot.turn_guard_tilt = applied = std::clamp(value, 0.10, 0.50);
            else if (name == "trot_turn_guard_release") trot.turn_guard_release = applied = std::clamp(value, 0.02, 0.40);
            else if (name == "trot_turn_budget") trot.turn_budget = applied = std::clamp(value, 0.10, 6.30);
            else if (name == "trot_tilt_soft") trot.tilt_soft = applied = std::clamp(value, 0.05, 0.45);
            else if (name == "trot_tilt_hard") trot.tilt_hard = applied = std::clamp(value, 0.20, 0.80);
            else known = false;

            if (!known)
            {
                continue;
            }
            if (applied != value)
            {
                RCLCPP_WARN(get_node()->get_logger(),
                            "Parameter %s=%.3f out of range, clamped to %.3f",
                            name.c_str(), value, applied);
            }
            else
            {
                RCLCPP_INFO(get_node()->get_logger(), "Parameter %s set to %.3f",
                            name.c_str(), applied);
            }
        }

        // Keep the attitude limits ordered; a hard limit at or below the soft one
        // would make the derating term meaningless.
        if (trot.tilt_hard <= trot.tilt_soft)
        {
            trot.tilt_hard = trot.tilt_soft + 0.05;
        }

        return result;
    }

    std::shared_ptr<FSMState> UnitreeGuideController::getNextState(const FSMStateName stateName) const
    {
        switch (stateName)
        {
        case FSMStateName::INVALID:
            return state_list_.invalid;
        case FSMStateName::PASSIVE:
            return state_list_.passive;
        case FSMStateName::FIXEDDOWN:
            return state_list_.fixedDown;
        case FSMStateName::FIXEDSTAND:
            return state_list_.fixedStand;
        case FSMStateName::FREESTAND:
            return state_list_.freeStand;
        case FSMStateName::TROTTING:
            return state_list_.trotting;
        case FSMStateName::SWINGTEST:
            return state_list_.swingTest;
        case FSMStateName::BALANCETEST:
            return state_list_.balanceTest;
        default:
            return state_list_.invalid;
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(unitree_guide_controller::UnitreeGuideController, controller_interface::ControllerInterface);
