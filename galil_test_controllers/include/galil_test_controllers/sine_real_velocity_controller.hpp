#ifndef GALIL_TEST_CONTROLLERS__SINE_REAL_VELOCITY_CONTROLLER_HPP_
#define GALIL_TEST_CONTROLLERS__SINE_REAL_VELOCITY_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace galil_test_controllers
{

class SineRealVelocityController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  void set_all_commands(double value);

  std::vector<std::string> joint_names_;
  std::string interface_name_;
  std::size_t joint_index_;
  double amplitude_;
  double frequency_;
  double offset_;
  double phase_;
  double duration_;
  double elapsed_time_seconds_;
  bool finished_;
};

}  // namespace galil_test_controllers

#endif
