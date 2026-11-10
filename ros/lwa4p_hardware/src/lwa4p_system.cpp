// Copyright (c) 2026 Eduard Zelenay. MIT License, see LICENSE.

#include "lwa4p_hardware/lwa4p_system.hpp"

#include <linux/can.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lwa4p_hardware/can_socket.hpp"
#include "lwa4p_hardware/cia402.hpp"
#include "lwa4p_hardware/sdo_client.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace lwa4p_hardware
{

namespace
{

using controller_manager_msgs::srv::SwitchController;
using hardware_interface::return_type;
using std_srvs::srv::Trigger;
using Clock = std::chrono::steady_clock;

constexpr double DEG = 0.017453292519943295;
constexpr int32_t COMMUTATION_STEP_MDEG = 500;
constexpr uint32_t COMMUTATION_VELOCITY_MDEG_S = 2000;
constexpr int32_t MOVE_TOLERANCE_MDEG = 100;
constexpr auto FEEDBACK_TIMEOUT = std::chrono::milliseconds(100);

rclcpp::Logger Log() { return rclcpp::get_logger("Lwa4pSystem"); }

double Param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  double fallback)
{
  const auto it = params.find(key);
  return it == params.end() ? fallback : std::stod(it->second);
}

void Sleep(double seconds) { std::this_thread::sleep_for(std::chrono::duration<double>(seconds)); }

std::string Hex16(uint16_t value)
{
  char buffer[8];
  std::snprintf(buffer, sizeof(buffer), "0x%04X", value);
  return buffer;
}

}  // namespace

Lwa4pSystem::~Lwa4pSystem()
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  if (sdo_) {
    for (const auto & joint : joints_) {
      try {
        sdo_->Download(joint.node, OD_CONTROLWORD, 0, CW_SHUTDOWN, 2);
      } catch (const std::exception &) {
      }  // best effort: the modules also stop on their own once SYNC stops
    }
  }
}

Lwa4pSystem::CallbackReturn Lwa4pSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const auto & hw = info_.hardware_parameters;
  interface_ = hw.count("can_interface") ? hw.at("can_interface") : "can0";
  controller_ = hw.count("controller") ? hw.at("controller") : "arm_controller";
  max_velocity_ = Param(hw, "max_velocity_deg_s", 30.0) * DEG;
  margin_ = Param(hw, "limit_margin_deg", 3.0) * DEG;
  accel_mdeg_ = Param(hw, "acceleration_deg_s2", 40.0) * 1e3;
  cycle_ms_ = static_cast<int>(Param(hw, "cycle_ms", 8.0));
  commutation_check_ = !hw.count("commutation_check") || hw.at("commutation_check") != "false";
  for (const auto & info_joint : info_.joints) {
    if (!info_joint.parameters.count("node_id")) {
      RCLCPP_FATAL(Log(), "joint %s has no node_id parameter", info_joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    Joint joint;
    joint.name = info_joint.name;
    joint.node = static_cast<uint8_t>(std::stoi(info_joint.parameters.at("node_id")));
    joint.sign = Param(info_joint.parameters, "sign", 1.0);
    joint.offset = Param(info_joint.parameters, "offset_deg", 0.0) * DEG;
    joints_.push_back(joint);
  }
  return CallbackReturn::SUCCESS;
}

Lwa4pSystem::CallbackReturn Lwa4pSystem::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    // TPDOs (0x180-0x1FF) and emergencies (0x080-0x0FF) for the control loop
    cyclic_ = std::make_unique<CanSocket>(
      interface_, std::vector<can_filter>{{COB_TPDO1, 0x780}, {COB_EMCY, 0x780}});
    sdo_ = std::make_unique<SdoClient>(interface_);
    for (auto & joint : joints_) {
      const auto lo = static_cast<int32_t>(sdo_->Upload(joint.node, OD_SW_LIMITS, 1));
      const auto hi = static_cast<int32_t>(sdo_->Upload(joint.node, OD_SW_LIMITS, 2));
      const double a = ToRad(lo, joint.sign, joint.offset);
      const double b = ToRad(hi, joint.sign, joint.offset);
      joint.lower = std::min(a, b) + margin_;
      joint.upper = std::max(a, b) - margin_;
      joint.position_mdeg = static_cast<int32_t>(sdo_->Upload(joint.node, OD_POSITION, 0));
      joint.position = ToRad(joint.position_mdeg, joint.sign, joint.offset);
      joint.command = joint.position;
      joint.target_mdeg = joint.position_mdeg;
      joint.last_feedback = Clock::now();
      // the drives interpolate over one control cycle: 0x60C2 = cycle_ms * 10^-3 s
      sdo_->Download(joint.node, OD_IP_PERIOD, 1, static_cast<uint32_t>(cycle_ms_), 1);
      sdo_->Download(joint.node, OD_IP_PERIOD, 2, static_cast<uint8_t>(-3), 1);
    }
    // synchronous PDOs only flow in NMT operational
    const uint8_t start_all[2] = {NMT_START, 0};
    cyclic_->Send(COB_NMT, start_all, 2);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(Log(), "configure failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    Log(), "LWA4P on %s: %zu joints, interpolated position, %d ms cycle", interface_.c_str(),
    joints_.size(), cycle_ms_);
  return CallbackReturn::SUCCESS;
}

Lwa4pSystem::CallbackReturn Lwa4pSystem::on_activate(const rclcpp_lifecycle::State &)
{
  node_ = rclcpp::Node::make_shared("lwa4p_driver");
  client_node_ = rclcpp::Node::make_shared("lwa4p_driver_controller_switch");
  switch_client_ =
    client_node_->create_client<SwitchController>("/controller_manager/switch_controller");
  services_.push_back(node_->create_service<Trigger>(
    "~/enable", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
      res->message = EnableAll();
      res->success = enabled_;
    }));
  services_.push_back(node_->create_service<Trigger>(
    "~/disable", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
      DisableAll();
      res->success = true;
      res->message = "all joints disabled (brakes closed)";
    }));
  services_.push_back(node_->create_service<Trigger>(
    "~/reset", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
      res->message = ResetAll();
      res->success = res->message.find("fault") == std::string::npos;
    }));
  trip_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), [this] { HandleTrip(); });
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this] { executor_->spin(); });
  RCLCPP_INFO(
    Log(), "drives DISABLED - call 'ros2 service call /lwa4p_driver/enable std_srvs/srv/Trigger'");
  return CallbackReturn::SUCCESS;
}

Lwa4pSystem::CallbackReturn Lwa4pSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  DisableAll();
  executor_->cancel();
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  services_.clear();
  trip_timer_.reset();
  node_.reset();
  executor_.reset();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Lwa4pSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (auto & joint : joints_) {
    out.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.position);
    out.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.velocity);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> Lwa4pSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (auto & joint : joints_) {
    out.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.command);
  }
  return out;
}

return_type Lwa4pSystem::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = Clock::now();
  can_frame frame{};
  while (cyclic_->Receive(frame, 0)) {
    for (auto & joint : joints_) {
      if (frame.can_id == COB_TPDO1 + joint.node && frame.can_dlc == 8) {
        const Tpdo1 tpdo = ParseTpdo1(frame.data);
        joint.statusword = tpdo.statusword;
        joint.position_mdeg = tpdo.position_mdeg;
        joint.last_feedback = now;
      } else if (frame.can_id == COB_EMCY + joint.node && frame.can_dlc >= 2) {
        const auto code = static_cast<uint16_t>(frame.data[0] | (frame.data[1] << 8));
        if (code != 0 && joint.ip_active) {
          Trip(joint.name + " emergency " + ErrorText(code));
        } else if (code != 0) {
          RCLCPP_WARN(Log(), "%s emergency %s", joint.name.c_str(), ErrorText(code).c_str());
        }
      }
    }
  }
  const double dt = period.seconds();
  for (auto & joint : joints_) {
    const double position = ToRad(joint.position_mdeg, joint.sign, joint.offset);
    if (dt > 0.0) {
      joint.velocity = 0.5 * joint.velocity + 0.5 * (position - joint.position) / dt;
    }
    joint.position = position;
    if (!joint.ip_active) {
      continue;
    }
    if (now - joint.last_feedback > FEEDBACK_TIMEOUT) {
      Trip(joint.name + ": no feedback for 100 ms");
    } else if (IsFault(joint.statusword) || !IsEnabled(joint.statusword)) {
      Trip(
        joint.name + " statusword " + Hex16(joint.statusword) + " (" + StateName(joint.statusword) +
        ")");
    }
  }
  return return_type::OK;
}

return_type Lwa4pSystem::write(const rclcpp::Time &, const rclcpp::Duration & period)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // safety net behind the trajectory controller: never step faster than the drives may move
  const auto step = static_cast<int32_t>(
    std::max(1.0, max_velocity_ * std::max(period.seconds(), 1e-3) / DEG * 1e3));
  for (auto & joint : joints_) {
    if (!joint.ip_active) {
      continue;
    }
    if (tripped_) {
      const auto pdo = PackRpdo1(CW_SHUTDOWN, joint.position_mdeg);
      cyclic_->Send(COB_RPDO1 + joint.node, pdo.data(), 8);
      joint.ip_active = false;
      continue;
    }
    int32_t target = joint.target_mdeg;
    if (std::isfinite(joint.command)) {
      const double command = std::clamp(joint.command, joint.lower, joint.upper);
      target = std::clamp(
        ToMdeg(command, joint.sign, joint.offset), joint.target_mdeg - step,
        joint.target_mdeg + step);
    }
    joint.target_mdeg = target;
    const auto pdo = PackRpdo1(CW_ENABLE_IP, target);
    cyclic_->Send(COB_RPDO1 + joint.node, pdo.data(), 8);
  }
  // every module applies its new setpoint on this SYNC and answers with its TPDO
  cyclic_->Send(COB_SYNC, nullptr, 0);
  return return_type::OK;
}

std::string Lwa4pSystem::EnableAll()
{
  if (enabled_) {
    return "already enabled";
  }
  HandleTrip();
  SwitchArmController(false);
  try {
    for (size_t i = 0; i < joints_.size(); ++i) {
      EnableJoint(i);
    }
  } catch (const std::exception & e) {
    DisableAll();
    RCLCPP_ERROR(Log(), "enable failed: %s", e.what());
    return std::string("enable failed: ") + e.what();
  }
  enabled_ = true;
  if (!SwitchArmController(true)) {
    DisableAll();
    return "drives enabled, but " + controller_ + " did not activate: disabled again";
  }
  RCLCPP_INFO(Log(), "all joints enabled (interpolated position), holding position");
  return "all joints enabled in interpolated position mode, holding position";
}

void Lwa4pSystem::EnableJoint(size_t index)
{
  const std::string & name = joints_[index].name;
  const uint8_t node = joints_[index].node;
  if (IsFault(Statusword(index))) {
    sdo_->Download(node, OD_CONTROLWORD, 0, CW_FAULT_RESET, 2);
    Sleep(0.1);
    sdo_->Download(node, OD_CONTROLWORD, 0, 0, 2);
    Sleep(0.1);
  }
  // profile position first: enable as the Python driver does, then the commutation check
  sdo_->Download(node, OD_MODE, 0, static_cast<uint8_t>(MODE_PROFILE_POSITION), 1);
  sdo_->Download(node, OD_PROFILE_ACCEL, 0, static_cast<uint32_t>(accel_mdeg_), 4);
  const auto actual = sdo_->Upload(node, OD_POSITION, 0);
  sdo_->Download(node, OD_TARGET, 0, actual, 4);
  sdo_->Download(node, OD_CONTROLWORD, 0, CW_SHUTDOWN, 2);
  WaitState(index, 0x006F, 0x0021, 3.0);
  sdo_->Download(node, OD_CONTROLWORD, 0, CW_SWITCH_ON, 2);
  WaitState(index, 0x006F, 0x0023, 3.0);
  sdo_->Download(node, OD_CONTROLWORD, 0, CW_ENABLE, 2);
  WaitState(index, 0x006F, 0x0027, 10.0);  // first enable after power-up may commutate
  Sleep(0.3);
  if (commutation_check_ && !Commutate(index)) {
    throw std::runtime_error(name + " did not move during the commutation check (+-0.5 deg)");
  }
  // interpolated position from the current position: a setpoint != actual faults the module
  sdo_->Download(node, OD_MODE, 0, static_cast<uint8_t>(MODE_INTERPOLATED_POSITION), 1);
  const auto deadline = Clock::now() + std::chrono::milliseconds(500);
  while (static_cast<int8_t>(sdo_->Upload(node, OD_MODE_DISPLAY, 0)) !=
         MODE_INTERPOLATED_POSITION) {
    if (Clock::now() > deadline) {
      throw std::runtime_error(name + " did not switch to interpolated position mode");
    }
    Sleep(0.02);
  }
  const auto here = static_cast<int32_t>(sdo_->Upload(node, OD_POSITION, 0));
  sdo_->Download(node, OD_IP_DATA, 1, static_cast<uint32_t>(here), 4);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Joint & joint = joints_[index];
    joint.position_mdeg = here;
    joint.target_mdeg = here;
    joint.command = ToRad(here, joint.sign, joint.offset);
    joint.last_feedback = Clock::now();
    joint.ip_active = true;  // the control loop now streams controlword 0x1F and this setpoint
  }
  const auto ip_deadline = Clock::now() + std::chrono::seconds(1);
  while (true) {
    uint16_t statusword;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      statusword = joints_[index].statusword;
    }
    if (IsIpActive(statusword) && IsEnabled(statusword)) {
      break;
    }
    if (Clock::now() > ip_deadline || tripped_) {
      throw std::runtime_error(
        name + " not following setpoints (statusword " + Hex16(statusword) + ")");
    }
    Sleep(0.02);
  }
  RCLCPP_INFO(Log(), "%s enabled", name.c_str());
}

void Lwa4pSystem::DisableAll()
{
  SwitchArmController(false);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & joint : joints_) {
      joint.ip_active = false;
    }
  }
  for (const auto & joint : joints_) {
    try {
      sdo_->Download(joint.node, OD_CONTROLWORD, 0, CW_SWITCH_ON, 2);
      Sleep(0.05);
      sdo_->Download(joint.node, OD_CONTROLWORD, 0, CW_SHUTDOWN, 2);
    } catch (const std::exception & e) {
      RCLCPP_WARN(Log(), "%s disable: %s", joint.name.c_str(), e.what());
    }
  }
  enabled_ = false;
}

std::string Lwa4pSystem::ResetAll()
{
  const uint8_t start_all[2] = {NMT_START, 0};
  cyclic_->Send(COB_NMT, start_all, 2);
  std::string states;
  for (size_t i = 0; i < joints_.size(); ++i) {
    const Joint & joint = joints_[i];
    try {
      sdo_->Download(joint.node, OD_CONTROLWORD, 0, CW_FAULT_RESET, 2);
      Sleep(0.1);
      sdo_->Download(joint.node, OD_CONTROLWORD, 0, 0, 2);
      Sleep(0.1);
      const uint16_t statusword = Statusword(i);
      states += joint.name + ": " + Hex16(statusword) + " " + StateName(statusword);
      if (IsFault(statusword)) {
        states += ", last error " + ErrorText(LastError(joint.node));
      }
    } catch (const std::exception & e) {
      states += joint.name + ": " + e.what();
    }
    states += i + 1 < joints_.size() ? "; " : "";
  }
  return states;
}

void Lwa4pSystem::Trip(const std::string & why)
{
  if (tripped_) {
    return;
  }
  trip_reason_ = why;
  tripped_ = true;
  RCLCPP_ERROR(Log(), "%s -> disabling all drives", why.c_str());
}

void Lwa4pSystem::HandleTrip()
{
  if (!tripped_) {
    return;
  }
  SwitchArmController(false);
  std::string why;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    why = trip_reason_;
    for (auto & joint : joints_) {
      joint.ip_active = false;
    }
  }
  for (const auto & joint : joints_) {
    try {
      sdo_->Download(joint.node, OD_CONTROLWORD, 0, CW_SHUTDOWN, 2);
      const uint16_t code = LastError(joint.node);
      if (code != 0) {
        RCLCPP_ERROR(Log(), "%s last error %s", joint.name.c_str(), ErrorText(code).c_str());
      }
    } catch (const std::exception &) {
    }
  }
  enabled_ = false;
  tripped_ = false;
  RCLCPP_ERROR(
    Log(), "%s: drives disabled (brakes closed); fix the cause, then ~/reset and ~/enable",
    why.c_str());
}

bool Lwa4pSystem::SwitchArmController(bool activate)
{
  if (!switch_client_ || !switch_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(Log(), "controller_manager switch_controller service not available");
    return false;
  }
  auto request = std::make_shared<SwitchController::Request>();
  (activate ? request->activate_controllers : request->deactivate_controllers)
    .push_back(controller_);
  request->strictness = SwitchController::Request::BEST_EFFORT;
  request->activate_asap = true;
  request->timeout = rclcpp::Duration(std::chrono::seconds(2));
  auto future = switch_client_->async_send_request(request);
  if (
    rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(3)) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_WARN(Log(), "switching %s timed out", controller_.c_str());
    return false;
  }
  return future.get()->ok;
}

uint16_t Lwa4pSystem::Statusword(size_t index)
{
  return static_cast<uint16_t>(sdo_->Upload(joints_[index].node, OD_STATUSWORD, 0));
}

void Lwa4pSystem::WaitState(size_t index, uint16_t mask, uint16_t value, double timeout_s)
{
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_s);
  uint16_t statusword = Statusword(index);
  while ((statusword & mask) != value) {
    if (Clock::now() > deadline) {
      throw std::runtime_error(
        joints_[index].name + ": statusword " + Hex16(statusword) + " (" + StateName(statusword) +
        "), last error " + ErrorText(LastError(joints_[index].node)));
    }
    Sleep(0.02);
    statusword = Statusword(index);
  }
}

bool Lwa4pSystem::MoveTo(size_t index, int32_t target_mdeg)
{
  const uint8_t node = joints_[index].node;
  sdo_->Download(node, OD_PROFILE_VELOCITY, 0, COMMUTATION_VELOCITY_MDEG_S, 4);
  sdo_->Download(node, OD_TARGET, 0, static_cast<uint32_t>(target_mdeg), 4);
  sdo_->Download(node, OD_CONTROLWORD, 0, CW_NEW_SETPOINT, 2);
  Sleep(0.05);
  sdo_->Download(node, OD_CONTROLWORD, 0, CW_ENABLE, 2);
  const auto deadline = Clock::now() + std::chrono::seconds(6);
  while (Clock::now() < deadline) {
    const auto position = static_cast<int32_t>(sdo_->Upload(node, OD_POSITION, 0));
    if (std::abs(position - target_mdeg) < MOVE_TOLERANCE_MDEG) {
      return true;
    }
    if (IsFault(Statusword(index))) {
      return false;
    }
    Sleep(0.05);
  }
  return false;
}

bool Lwa4pSystem::Commutate(size_t index)
{
  // after a motor-power drop the modules commutate on their first motion; a joint resting
  // against something may only move one way, so try both
  const auto start = static_cast<int32_t>(sdo_->Upload(joints_[index].node, OD_POSITION, 0));
  for (const int32_t step : {COMMUTATION_STEP_MDEG, -COMMUTATION_STEP_MDEG}) {
    const bool out = MoveTo(index, start + step);
    const bool back = MoveTo(index, start);
    if (out && back) {
      return true;
    }
  }
  return false;
}

uint16_t Lwa4pSystem::LastError(uint8_t node)
{
  try {
    if (sdo_->Upload(node, OD_ERROR_HISTORY, 0) != 0) {
      return static_cast<uint16_t>(sdo_->Upload(node, OD_ERROR_HISTORY, 1) & 0xFFFF);
    }
  } catch (const std::exception &) {
  }
  return 0;
}

}  // namespace lwa4p_hardware

PLUGINLIB_EXPORT_CLASS(lwa4p_hardware::Lwa4pSystem, hardware_interface::SystemInterface)
