// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "openarm_hardware/openarm_simple_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <sstream>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware {

namespace {

// Parse a whitespace-separated list of doubles, e.g. "0.1 -0.2 0.3".
bool parse_double_list(const std::string& text, const std::string& name,
                       size_t expected, std::vector<double>& out) {
  out.clear();
  std::istringstream stream(text);
  double value;
  while (stream >> value) {
    out.push_back(value);
  }
  if (!stream.eof()) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                 "Parameter %s contains a non-numeric token: '%s'",
                 name.c_str(), text.c_str());
    return false;
  }
  if (out.size() != expected) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                 "Parameter %s has %zu values, expected %zu", name.c_str(),
                 out.size(), expected);
    return false;
  }
  return true;
}

}  // namespace

OpenArmHW::OpenArmHW() = default;

bool OpenArmHW::parse_config(const hardware_interface::HardwareInfo& info) {
  // Parse CAN interface (default: can0)
  auto it = info.hardware_parameters.find("can_interface");
  can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";

  // Parse arm prefix (default: empty for single arm, "left_" or "right_" for
  // bimanual)
  it = info.hardware_parameters.find("arm_prefix");
  arm_prefix_ = (it != info.hardware_parameters.end()) ? it->second : "";

  // Parse gripper enable (default: true for V10)
  it = info.hardware_parameters.find("hand");
  if (it == info.hardware_parameters.end()) {
    hand_ = true;  // Default to true for V10
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    hand_ = (value == "true");
  }

  // Parse CAN-FD enable (default: true for V10)
  it = info.hardware_parameters.find("can_fd");
  if (it == info.hardware_parameters.end()) {
    can_fd_ = true;  // Default to true for V10
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    can_fd_ = (value == "true");
  }

  // Parse control gains
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    it = info.hardware_parameters.find("kp" + std::to_string(i));
    if (it != info.hardware_parameters.end()) {
      kp_[i - 1] = std::stod(it->second);
    }
    it = info.hardware_parameters.find("kd" + std::to_string(i));
    if (it != info.hardware_parameters.end()) {
      kd_[i - 1] = std::stod(it->second);
    }
  }
  // Parse ee_type (default: parallel_link for v10)
  it = info.hardware_parameters.find("ee_type");
  ee_type_ =
      (it != info.hardware_parameters.end()) ? it->second : "parallel_link";
  if (hand_) {
    it = info.hardware_parameters.find("kp_hand");
    if (it != info.hardware_parameters.end()) {
      gripper_kp_ = std::stod(it->second);
    }
    it = info.hardware_parameters.find("kd_hand");
    if (it != info.hardware_parameters.end()) {
      gripper_kd_ = std::stod(it->second);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
              "Configuration: CAN=%s, arm_prefix=%s, hand=%s, can_fd=%s",
              can_interface_.c_str(), arm_prefix_.c_str(),
              hand_ ? "enabled" : "disabled", can_fd_ ? "enabled" : "disabled");
  return parse_encoder_calibration(info);
}

bool OpenArmHW::parse_encoder_calibration(
    const hardware_interface::HardwareInfo& info) {
  auto lb_it = info.hardware_parameters.find("lb_encoder");
  auto ub_it = info.hardware_parameters.find("ub_encoder");
  const bool has_lb = lb_it != info.hardware_parameters.end();
  const bool has_ub = ub_it != info.hardware_parameters.end();

  if (!has_lb && !has_ub) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArmHW"),
                "No lb_encoder/ub_encoder calibration parameters; assuming "
                "firmware-zeroed motors (zero offsets, default gripper map)");
    return true;
  }
  if (has_lb != has_ub) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                 "lb_encoder and ub_encoder must both be set; only %s found",
                 has_lb ? "lb_encoder" : "ub_encoder");
    return false;
  }

  const size_t n_motors = ARM_DOF + 1;  // gripper always included
  if (!parse_double_list(lb_it->second, "lb_encoder", n_motors, lb_encoder_) ||
      !parse_double_list(ub_it->second, "ub_encoder", n_motors, ub_encoder_)) {
    return false;
  }
  for (size_t i = 0; i < n_motors; ++i) {
    if (lb_encoder_[i] > ub_encoder_[i]) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                   "lb_encoder[%zu]=%.6f exceeds ub_encoder[%zu]=%.6f", i,
                   lb_encoder_[i], i, ub_encoder_[i]);
      return false;
    }
  }

  if (hand_) {
    if (ee_type_ == "pinch_gripper") {
      RCLCPP_WARN(rclcpp::get_logger("OpenArmHW"),
                  "Gripper encoder calibration is ignored for ee_type "
                  "pinch_gripper");
    } else {
      if (ub_encoder_[ARM_DOF] - lb_encoder_[ARM_DOF] < 1e-6) {
        RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                     "Gripper encoder span [%.6f, %.6f] is degenerate",
                     lb_encoder_[ARM_DOF], ub_encoder_[ARM_DOF]);
        return false;
      }
      // Motor angle decreases as the gripper opens (default open map is
      // -1.0472), so the swept minimum is open and the maximum is closed.
      gripper_motor_open_rad_ = lb_encoder_[ARM_DOF];
      gripper_motor_closed_rad_ = ub_encoder_[ARM_DOF];
    }
  }

  calibrated_ = true;
  return true;
}

bool OpenArmHW::compute_offsets(const hardware_interface::HardwareInfo& info) {
  if (!calibrated_) {
    return true;
  }
  for (size_t i = 0; i < ARM_DOF; ++i) {
    auto it = info.limits.find(joint_names_[i]);
    if (it == info.limits.end() || !it->second.has_position_limits) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                   "Joint %s has no URDF position limits; cannot compute "
                   "calibration offset",
                   joint_names_[i].c_str());
      return false;
    }
    pos_offsets_[i] = it->second.min_position - lb_encoder_[i];
    RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
                "%s: lb_encoder=%.4f urdf_lb=%.4f offset=%.4f",
                joint_names_[i].c_str(), lb_encoder_[i],
                it->second.min_position, pos_offsets_[i]);
  }
  if (hand_ && ee_type_ != "pinch_gripper") {
    RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
                "Gripper motor map: open(lb)=%.4f closed(ub)=%.4f",
                gripper_motor_open_rad_, gripper_motor_closed_rad_);
  }
  return true;
}

void OpenArmHW::generate_joint_names() {
  joint_names_.clear();
  // TODO: read from urdf properly and sort in the future.
  // Currently, the joint names are hardcoded for order consistency to align
  // with hardware. Generate arm joint names: openarm_{arm_prefix}joint{N}
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    std::string joint_name =
        "openarm_" + arm_prefix_ + "joint" + std::to_string(i);
    joint_names_.push_back(joint_name);
  }

  // Generate gripper joint name if enabled
  if (hand_) {
    std::string gripper_joint_name = "openarm_" + arm_prefix_ + "finger_joint1";
    joint_names_.push_back(gripper_joint_name);
    RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Added gripper joint: %s",
                gripper_joint_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
                "Gripper joint NOT added because hand_=false");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
              "Generated %zu joint names for arm prefix '%s'",
              joint_names_.size(), arm_prefix_.c_str());
}

hardware_interface::CallbackReturn OpenArmHW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // Parse configuration
  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }

  // Generate joint names based on arm prefix
  generate_joint_names();

  // Validate joint count (7 arm joints + optional gripper)
  size_t expected_joints = ARM_DOF + (hand_ ? 1 : 0);
  if (joint_names_.size() != expected_joints) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                 "Generated %zu joint names, expected %zu", joint_names_.size(),
                 expected_joints);
    return CallbackReturn::ERROR;
  }

  // Compute calibration offsets (needs joint names to look up URDF limits)
  if (!compute_offsets(info)) {
    return CallbackReturn::ERROR;
  }

  // Initialize OpenArm with configurable CAN-FD setting
  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
              "Initializing OpenArm on %s with CAN-FD %s...",
              can_interface_.c_str(), can_fd_ ? "enabled" : "disabled");
  openarm_ =
      std::make_unique<openarm::can::socket::OpenArm>(can_interface_, can_fd_);

  // Initialize arm motors with V10 defaults
  openarm_->init_arm_motors(DEFAULT_MOTOR_TYPES, DEFAULT_SEND_CAN_IDS,
                            DEFAULT_RECV_CAN_IDS);

  // Initialize gripper if enabled
  if (hand_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Initializing gripper...");
    openarm_->init_gripper_motor(DEFAULT_GRIPPER_MOTOR_TYPE,
                                 DEFAULT_GRIPPER_SEND_CAN_ID,
                                 DEFAULT_GRIPPER_RECV_CAN_ID);
  }

  // Initialize state and command vectors based on generated joint count
  const size_t total_joints = joint_names_.size();
  pos_commands_.resize(total_joints, 0.0);
  vel_commands_.resize(total_joints, 0.0);
  tau_commands_.resize(total_joints, 0.0);
  pos_states_.resize(total_joints, 0.0);
  vel_states_.resize(total_joints, 0.0);
  tau_states_.resize(total_joints, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
              "OpenArm V10 Simple HW initialized successfully");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmHW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Set callback mode to ignore during configuration
  openarm_->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArmHW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenArmHW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // TODO: consider exposing only needed interfaces to avoid undefined behavior.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn OpenArmHW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Activating OpenArm V10...");
  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
  openarm_->enable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  // Seed commands with current position to prevent unintended motion on
  // activation
  const auto& arm_motors = openarm_->get_arm().get_motors();
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    pos_states_[i] = arm_motors[i].get_position() + pos_offsets_[i];
    pos_commands_[i] = pos_states_[i];
  }
  if (hand_) {
    const auto& gripper_motors = openarm_->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      pos_states_[ARM_DOF] =
          motor_radians_to_joint(gripper_motors[0].get_position());
      pos_commands_[ARM_DOF] = pos_states_[ARM_DOF];
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "OpenArm V10 activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmHW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Deactivating OpenArm V10...");

  // Disable all motors (like full_arm.cpp exit)
  for (int i = 0; i < 3; ++i) {
    openarm_->disable_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    openarm_->recv_all();
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "OpenArm V10 deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArmHW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Receive all motor states
  openarm_->refresh_all();
  openarm_->recv_all();

  // Read arm joint states
  const auto& arm_motors = openarm_->get_arm().get_motors();
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    pos_states_[i] = arm_motors[i].get_position() + pos_offsets_[i];
    vel_states_[i] = arm_motors[i].get_velocity();
    tau_states_[i] = arm_motors[i].get_torque();
  }

  // Read gripper state if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    const auto& gripper_motors = openarm_->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      // TODO the mappings are approximates
      // Convert motor position (radians) to joint value (0-0.044m)
      double motor_pos = gripper_motors[0].get_position();
      pos_states_[ARM_DOF] = motor_radians_to_joint(motor_pos);

      // Unimplemented: Velocity and torque mapping
      vel_states_[ARM_DOF] = 0;  // gripper_motors[0].get_velocity();
      tau_states_[ARM_DOF] = 0;  // gripper_motors[0].get_torque();
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArmHW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Control arm motors with MIT control
  std::vector<openarm::damiao_motor::MITParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    arm_params.push_back({kp_[i], kd_[i], pos_commands_[i] - pos_offsets_[i],
                          vel_commands_[i], tau_commands_[i]});
  }
  openarm_->get_arm().mit_control_all(arm_params);
  // Control gripper if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    // TODO the true mappings are unimplemented.
    double motor_command = joint_to_motor_radians(pos_commands_[ARM_DOF]);
    openarm_->get_gripper().mit_control_all(
        {{gripper_kp_, gripper_kd_, motor_command, 0.0, 0.0}});
  }
  openarm_->recv_all(100);
  return hardware_interface::return_type::OK;
}

void OpenArmHW::return_to_zero() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Returning to zero position...");

  openarm_->refresh_all();
  // Return arm to zero with MIT control
  std::vector<openarm::damiao_motor::MITParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    arm_params.push_back({kp_[i], kd_[i], 0.0, 0.0, 0.0});
  }
  openarm_->get_arm().mit_control_all(arm_params);

  // Return gripper to zero if enabled
  if (hand_) {
    openarm_->get_gripper().mit_control_all(
        {{gripper_kp_, gripper_kd_, GRIPPER_JOINT_0_POSITION, 0.0, 0.0}});
  }
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  openarm_->recv_all();
  const auto& arm_motors = openarm_->get_arm().get_motors();

  std::vector<double> start_pos(ARM_DOF, 0.0);
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    start_pos[i] = arm_motors[i].get_position();
  }

  const int steps = 200;
  const int step_ms = 10;

  for (int step = 0; step <= steps; ++step) {
    double t = static_cast<double>(step) / steps;  // 0.0 → 1.0

    std::vector<openarm::damiao_motor::MITParam> arm_params;
    for (size_t i = 0; i < ARM_DOF; ++i) {
      double target = start_pos[i] + t * (ZERO_POSITION[i] - start_pos[i]);
      arm_params.push_back({kp_[i], kd_[i], target, 0.0, 0.0});
    }
    openarm_->get_arm().mit_control_all(arm_params);

    if (hand_) {
      openarm_->get_gripper().mit_control_all(
          {{GRIPPER_KP, GRIPPER_KD, GRIPPER_JOINT_0_POSITION, 0.0, 0.0}});
    }

    openarm_->recv_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Reached zero position");
}

// void OpenArmHW::return_to_zero() {
//   RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Returning to zero
//   position...");

//   // Return arm to zero with MIT control
//   std::vector<openarm::damiao_motor::MITParam> arm_params;
//   for (size_t i = 0; i < ARM_DOF; ++i) {
//     arm_params.push_back({kp_[i], kd_[i], 0.0, 0.0, 0.0});
//   }
//   openarm_->get_arm().mit_control_all(arm_params);

//   // Return gripper to zero if enabled
//   if (hand_) {
//     openarm_->get_gripper().mit_control_all(
//         {{GRIPPER_KP, GRIPPER_KD, GRIPPER_JOINT_0_POSITION, 0.0, 0.0}});
//   }
//   std::this_thread::sleep_for(std::chrono::microseconds(1000));
//   openarm_->recv_all();
// }

double OpenArmHW::joint_to_motor_radians(double joint_value) {
  if (ee_type_ == "pinch_gripper") {
    // revolute: joint 0-1.5708 rad -> motor 0-1.5708
    return joint_value;
  } else {
    // parallel_link (prismatic): joint 0 m (closed) -> closed motor angle,
    // 0.044 m (open) -> open motor angle
    return gripper_motor_closed_rad_ +
           (joint_value / GRIPPER_JOINT_0_POSITION) *
               (gripper_motor_open_rad_ - gripper_motor_closed_rad_);
  }
}

double OpenArmHW::motor_radians_to_joint(double motor_radians) {
  if (ee_type_ == "pinch_gripper") {
    // revolute:
    return motor_radians;
  } else {
    // parallel_link (prismatic)
    return GRIPPER_JOINT_0_POSITION *
           (motor_radians - gripper_motor_closed_rad_) /
           (gripper_motor_open_rad_ - gripper_motor_closed_rad_);
  }
}

// // Gripper mapping helper functions
// double OpenArmHW::joint_to_motor_radians(double joint_value) {
//   // Joint 0=closed -> motor 0 rad, Joint 0.044=open -> motor -1.0472 rad
//   return (joint_value / GRIPPER_JOINT_0_POSITION) *
//          GRIPPER_MOTOR_1_RADIANS;  // Scale from 0-0.044 to 0 to -1.0472
// }

// double OpenArmHW::motor_radians_to_joint(double motor_radians) {
//   // Motor 0 rad=closed -> joint 0, Motor -1.0472 rad=open -> joint 0.044
//   return GRIPPER_JOINT_0_POSITION *
//          (motor_radians /
//           GRIPPER_MOTOR_1_RADIANS);  // Scale from 0 to -1.0472 to 0-0.044
// }

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArmHW,
                       hardware_interface::SystemInterface)
