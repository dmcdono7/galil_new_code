#include "controller_interface/controller_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "rclcpp_lifecycle/state.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "mp_needle_trajectory_controller/mpc_helpers.hpp"

class MpNeedleTrajectoryController : public controller_interface::ControllerInterface
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

protected:

  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_cmd_handles_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_state_handles_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr needle_tip_sub_;
  void needleTipCallback(const geometry_msgs::msg::PoseStamped::SharedPtr needle_tip);
  
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr target);
  
  bool initialized_ = {false};
  bool configured_ = {false};
  bool active_ = {false};
  bool target_set_ = {false};
  bool tip_set_ = {false};
  
  std::vector<std::string> joint_names_; 
  std::string cmd_interface_type_;
  
  // values from subscriptions
  std::vector<double> needle_tip_pose = {0, 0, 0, 0, 0};
  std::vector<double> target_point = {0, 0, 0};
  
  // values from hw interface
  std::vector<double> stages;
  
  // hard-coded values for MPC
  double insertion_step = 5.0;
  double insertion_length;
  double ns;
  double horizon = 5.0;
  double depth = 0;
  double limit = 5.0;
  
  MpcHelpers mpc = MpcHelpers(limit, insertion_step);
  
  // helper functions
  void writeJointControlCmds(const std::vector<double> & cmd);

};
