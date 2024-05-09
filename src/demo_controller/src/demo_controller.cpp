#include "demo_controller.h"

#define DEMO_COMMAND_TOPIC "demo_controller_command"

namespace demo_controller
{

#define logger get_node()->get_logger()

DemoController::DemoController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn DemoController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(logger, "Exception thrown during init stage with message: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration DemoController::command_interface_configuration()
  const
{
  std::vector<std::string> conf_names;
  for (const auto & command_interface : params_.command_interfaces)
  {
    conf_names.push_back(params_.left_wheel_name + "/" + command_interface);
    conf_names.push_back(params_.right_wheel_name + "/" + command_interface);
  }
  RCLCPP_INFO(logger, "Check Config name");
  for (const auto & name : conf_names)
  {
    RCLCPP_INFO(logger, "Config name: %s", name.c_str());
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration DemoController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto & state_interface : params_.state_interfaces)
  {
    conf_names.push_back(params_.left_wheel_name + "/" + state_interface);
    conf_names.push_back(params_.right_wheel_name + "/" + state_interface);
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::CallbackReturn DemoController::on_configure(const rclcpp_lifecycle::State &)
{
  if(!demo_controller_command_publisher_) {
    auto callback =
      [this](const std::shared_ptr<demo_interfaces::msg::DemoControllerCommand> msg) -> void
    {
      command_msg_external_point_ptr_.writeFromNonRT(msg);
      new_msg_ = true;
    };

    demo_controller_command_publisher_ =
      get_node()->create_subscription<demo_interfaces::msg::DemoControllerCommand>(
        DEMO_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(), callback);
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DemoController::on_activate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type DemoController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::shared_ptr<demo_interfaces::msg::DemoControllerCommand> command = nullptr;
  if (new_msg_)
  {
    command = *command_msg_external_point_ptr_.readFromRT();
    new_msg_ = false;
  }
  if(command) {
    for (auto & interface : command_interfaces_)
    {
      if(interface.get_prefix_name().compare(params_.left_wheel_name) == 0) interface.set_value(command->left_velocity);
      if(interface.get_prefix_name().compare(params_.right_wheel_name) == 0) interface.set_value(command->right_velocity);
    }
  }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn DemoController::on_deactivate(const rclcpp_lifecycle::State &)
{
  release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DemoController::on_cleanup(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DemoController::on_error(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DemoController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

}  // namespace demo_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(demo_controller::DemoController, controller_interface::ControllerInterface)