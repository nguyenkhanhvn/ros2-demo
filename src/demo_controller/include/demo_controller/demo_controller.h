#pragma once

#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "demo_interfaces/msg/demo_controller_command.hpp"
#include "controller_interface/controller_interface.hpp"
#include "demo_parameters.hpp"

namespace demo_controller
{
class DemoController : public controller_interface::ControllerInterface
{
public:
  DemoController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  // Parameters from ROS for DemoController
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::shared_ptr<rclcpp::Subscription<demo_interfaces::msg::DemoControllerCommand>> demo_controller_command_publisher_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<demo_interfaces::msg::DemoControllerCommand>> command_msg_external_point_ptr_;
  bool new_msg_ = false;
};
}  // namespace demo_controller