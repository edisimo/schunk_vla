// Copyright (c) 2026 Eduard Zelenay. MIT License, see LICENSE.

#ifndef LWA4P_HARDWARE__LWA4P_SYSTEM_HPP_
#define LWA4P_HARDWARE__LWA4P_SYSTEM_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "lwa4p_hardware/can_socket.hpp"
#include "lwa4p_hardware/sdo_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace lwa4p_hardware
{

/**
 * @brief LWA 4P arm (six Schunk ERB modules) in CiA 402 interpolated position mode.
 *
 * Every cycle sends one RPDO setpoint per enabled joint and a SYNC, on which all modules step
 * together; TPDO feedback follows each SYNC. The drives start disabled: the services
 * ~/enable, ~/disable and ~/reset (node "lwa4p_driver") switch them, and enabling activates the
 * trajectory controller only once every joint holds its current position in IP mode.
 */
class Lwa4pSystem : public hardware_interface::SystemInterface
{
public:
  ~Lwa4pSystem() override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Joint
  {
    std::string name;
    uint8_t node{0};
    double sign{1.0};
    double offset{0.0};
    double lower{0.0};  // rad, drive limits minus the margin
    double upper{0.0};
    double position{0.0};  // state interfaces
    double velocity{0.0};
    double command{0.0};  // command interface
    int32_t position_mdeg{0};
    uint16_t statusword{0};
    std::chrono::steady_clock::time_point last_feedback{};
    bool ip_active{false};   // setpoints are streamed for this joint
    int32_t target_mdeg{0};  // last setpoint sent
  };

  std::string EnableAll();
  void DisableAll();
  std::string ResetAll();
  void EnableJoint(size_t index);
  void Trip(const std::string & why);
  void HandleTrip();
  bool SwitchArmController(bool activate);
  uint16_t Statusword(size_t index);
  void WaitState(size_t index, uint16_t mask, uint16_t value, double timeout_s);
  bool MoveTo(size_t index, int32_t target_mdeg);
  bool Commutate(size_t index);
  uint16_t LastError(uint8_t node);
  /** @brief Publish the drive reason used to stop teleop without losing the fault code. */
  void PublishFault(const std::string & reason);

  std::vector<Joint> joints_;
  std::string interface_;
  std::string controller_;
  double max_velocity_{0.52};
  double margin_{0.052};
  double accel_mdeg_{40000.0};
  int cycle_ms_{8};
  bool commutation_check_{true};

  std::unique_ptr<CanSocket> cyclic_;
  std::unique_ptr<SdoClient> sdo_;
  std::mutex mutex_;  // joints_ between the control loop and the service thread
  std::atomic<bool> enabled_{false};
  std::atomic<bool> tripped_{false};
  std::string trip_reason_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  rclcpp::TimerBase::SharedPtr trip_timer_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr limits_publisher_;
};

}  // namespace lwa4p_hardware

#endif  // LWA4P_HARDWARE__LWA4P_SYSTEM_HPP_
