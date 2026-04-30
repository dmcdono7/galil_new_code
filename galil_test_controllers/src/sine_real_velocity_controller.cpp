#include "galil_test_controllers/sine_real_velocity_controller.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace galil_test_controllers
{

namespace
{
constexpr double PI = 3.14159265358979;
}

controller_interface::InterfaceConfiguration
SineRealVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_)
  {
    config.names.push_back(joint_name + "/" + interface_name_);
  }

  return config;
}

controller_interface::InterfaceConfiguration
SineRealVelocityController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::CallbackReturn SineRealVelocityController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<std::string>("interface_name", "velocity");
  auto_declare<int>("joint_index", 0);
  auto_declare<double>("amplitude", 1000.0);
  auto_declare<double>("frequency", 0.05);
  auto_declare<double>("offset", 0.0);
  auto_declare<double>("phase", 0.0);
  auto_declare<double>("duration", 0.0);

  joint_index_ = 0;
  amplitude_ = 0.0;
  frequency_ = 0.0;
  offset_ = 0.0;
  phase_ = 0.0;
  duration_ = 0.0;
  elapsed_time_seconds_ = 0.0;
  finished_ = false;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SineRealVelocityController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  interface_name_ = get_node()->get_parameter("interface_name").as_string();

  const int joint_index = get_node()->get_parameter("joint_index").as_int();
  amplitude_ = get_node()->get_parameter("amplitude").as_double();
  frequency_ = get_node()->get_parameter("frequency").as_double();
  offset_ = get_node()->get_parameter("offset").as_double();
  phase_ = get_node()->get_parameter("phase").as_double();
  duration_ = get_node()->get_parameter("duration").as_double();

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter must not be empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (interface_name_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_name' parameter must not be empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (joint_index < 0 || static_cast<std::size_t>(joint_index) >= joint_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'joint_index'=%d is out of range for %zu configured joints.",
      joint_index,
      joint_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (frequency_ < 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'frequency' must be nonnegative.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (duration_ < 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'duration' must be nonnegative.");
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_index_ = static_cast<std::size_t>(joint_index);
  elapsed_time_seconds_ = 0.0;
  finished_ = false;

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured sine_real_velocity_controller for joint '%s' on interface '%s' with amplitude=%g, "
    "frequency=%g Hz, offset=%g, phase=%g rad, duration=%g s, controller update_rate=%u Hz.",
    joint_names_[joint_index_].c_str(),
    interface_name_.c_str(),
    amplitude_,
    frequency_,
    offset_,
    phase_,
    duration_,
    get_update_rate());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SineRealVelocityController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (command_interfaces_.size() != joint_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu command interfaces but received %zu.",
      joint_names_.size(),
      command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  elapsed_time_seconds_ = 0.0;
  finished_ = false;
  set_all_commands(0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SineRealVelocityController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  set_all_commands(0.0);
  elapsed_time_seconds_ = 0.0;
  finished_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SineRealVelocityController::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  if (finished_)
  {
    set_all_commands(0.0);
    return controller_interface::return_type::OK;
  }

  if (duration_ > 0.0 && elapsed_time_seconds_ >= duration_)
  {
    set_all_commands(0.0);
    finished_ = true;
    return controller_interface::return_type::OK;
  }

  const double command = offset_ +
    amplitude_ * std::sin(phase_ + (2.0 * PI * frequency_ * elapsed_time_seconds_));

  for (std::size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(i == joint_index_ ? command : 0.0);
  }

  elapsed_time_seconds_ += period.seconds();
  return controller_interface::return_type::OK;
}

void SineRealVelocityController::set_all_commands(double value)
{
  for (auto & command_interface : command_interfaces_)
  {
    command_interface.set_value(value);
  }
}

}  // namespace galil_test_controllers

PLUGINLIB_EXPORT_CLASS(
  galil_test_controllers::SineRealVelocityController,
  controller_interface::ControllerInterface)
