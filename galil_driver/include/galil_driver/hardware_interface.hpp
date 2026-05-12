
#ifndef GALIL_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_
#define GALIL_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

// ROS
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cstddef>
#include <gclib.h>
#include <gclib_record.h>
#include <string>
#include <vector>

namespace galil_driver{

  class GalilSystemHardwareInterface : public hardware_interface::SystemInterface {
  public:

    RCLCPP_SHARED_PTR_DEFINITIONS(GalilSystemHardwareInterface)
    virtual ~GalilSystemHardwareInterface();
    
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
								const std::vector<std::string>& stop_interfaces) override;

    hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
								const std::vector<std::string>& stop_interfaces) override;
    
  private:
    struct RealVelocityInitSettings {
      double kp;
      double ki;
      double kd;
      double it;
      double ac;
      double dc;
      bool valid;
    };

    char axis_letter(std::size_t index) const;
    bool send_galil_command(const std::string& command);
    int real_velocity_command_counts(std::size_t index) const;
    bool stop_real_velocity_axis(std::size_t index);
    bool stop_all_real_velocity_axes();
    bool parse_real_velocity_init_settings(std::size_t index, RealVelocityInitSettings& settings);
    bool send_galil_vector_command(
      const std::string& name,
      const std::vector<double>& values);
    bool initialize_real_velocity_axes(const std::vector<std::size_t>& axes);
    bool servo_here_real_velocity_axes(const std::vector<std::size_t>& axes);
    bool initialize_position_axes(const std::vector<std::size_t>& axes);
    hardware_interface::return_type write_real_velocity();

    hardware_interface::HardwareInfo info_;
    std::vector<double> hw_commands_position_;
    std::vector<double> hw_commands_velocity_;
    std::vector<double> hw_commands_real_velocity_;
    std::vector<double> hw_states_position_;
    std::vector<double> hw_states_velocity_;
    std::vector<double> hw_states_effort_;
    std::vector<int> last_real_velocity_counts_;
    std::vector<bool> real_velocity_jog_active_;
    std::vector<RealVelocityInitSettings> real_velocity_init_settings_;
    std::vector<double> position_max_velocity_;
    int cmd_mode_;

    //std::vector< char > channels;
    std::vector<double> gears_m_2_cnt;
    std::vector<double> torque_constants_;
    double check_timeout_;
    bool estop_triggered_;
    //change
    std::string ip_address_;
    GCon connection;
  };
}

#endif
