#include <mp_needle_trajectory_controller/mp_needle_traj.hpp>

#include <controller_interface/helpers.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/parameter.hpp>

#include <algorithm>
#include <cmath>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

controller_interface::CallbackReturn MpNeedleTrajectoryController::on_init(){
  std::cout << "MpNeedleTrajectoryController::on_init" << std::endl;
  if(!initialized_){
    auto_declare<std::string>("interface_name", "");
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<double>("max_velocity", 0.005);
    initialized_ = true;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MpNeedleTrajectoryController::on_configure (const rclcpp_lifecycle::State&){
  
  std::cout << "MpNeedleTrajectoryController::on_configure" << std::endl;
  
  if(configured_)
    { return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; }
  
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if(joint_names_.empty()){
    RCLCPP_ERROR(get_node()->get_logger(), "joints array is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  
  cmd_interface_type_ = get_node()->get_parameter("interface_name").as_string();
  std::cout << cmd_interface_type_ << std::endl;
  if(cmd_interface_type_.empty()){
    RCLCPP_ERROR(get_node()->get_logger(), "No command_interfaces specified");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  max_velocity_ = get_node()->get_parameter("max_velocity").as_double();
  if(!std::isfinite(max_velocity_) || max_velocity_ <= 0.0){
    RCLCPP_ERROR(get_node()->get_logger(), "max_velocity must be a positive finite value.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  
  for(const auto & joint_name : joint_names_){
    std::cout << joint_name << std::endl;
  }
  
  needle_tip_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>("sensor/tip", 3,
								      std::bind(&MpNeedleTrajectoryController::needleTipCallback,
										this, std::placeholders::_1));
  
  target_sub_ = get_node()->create_subscription<geometry_msgs::msg::PointStamped>("subject/state/target", 3,
								      std::bind(&MpNeedleTrajectoryController::targetCallback,
										this, std::placeholders::_1));
										
									
																			
  configured_ = true;
  stages.resize(4,0);
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MpNeedleTrajectoryController::on_activate(const rclcpp_lifecycle::State&){
  
  if(active_) { return controller_interface::CallbackReturn::SUCCESS; }

  if(!controller_interface::get_ordered_interfaces( command_interfaces_, // unordered command interfaces
						    joint_names_,        // ordered vector of names
						    cmd_interface_type_, // the interface type
						    joint_cmd_handles_ )){ // the ordered interfaces 
    RCLCPP_ERROR(get_node()->get_logger(),
		 "Expected %zu '%s' command interfaces, got %zu.",
		 joint_names_.size(), cmd_interface_type_.c_str(),
		 joint_cmd_handles_.size());
    return CallbackReturn::ERROR;
  }
  
  if(!controller_interface::get_ordered_interfaces(state_interfaces_,
						   joint_names_,
						   hardware_interface::HW_IF_POSITION,
						   joint_state_handles_ ) ) {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.",
		 joint_names_.size(), hardware_interface::HW_IF_POSITION,
		 joint_state_handles_.size());
    return CallbackReturn::ERROR;
  }

  last_command_positions_.resize(joint_names_.size(), 0.0);
  for(std::size_t i=0; i<joint_state_handles_.size(); ++i){
    last_command_positions_[i] = joint_state_handles_[i].get().get_value();
  }
  command_positions_initialized_ = true;

  active_ = true;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MpNeedleTrajectoryController::on_deactivate (const rclcpp_lifecycle::State&){
  std::cout << "MpNeedleTrajectoryController::on_deactivate" << std::endl;
  if(active_){ // flush everything
    joint_cmd_handles_.clear();
    joint_state_handles_.clear();
    this->release_interfaces();
  }
  active_ = false;
  command_positions_initialized_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration MpNeedleTrajectoryController::command_interface_configuration() const{
  std::cout << "MpNeedleTrajectoryController::command_interface_configuration" << std::endl;
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size() * cmd_interface_type_.size());
  for (const auto & joint_name : joint_names_){
    conf.names.push_back(joint_name + std::string("/").append(cmd_interface_type_));
    std::cout << joint_name + std::string("/").append(cmd_interface_type_) << std::endl;
  }
  return conf;
  
}

controller_interface::InterfaceConfiguration MpNeedleTrajectoryController::state_interface_configuration() const{
  std::cout << "MpNeedleTrajectoryController::state_interface_configuration" << std::endl;
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size());  // Only position
  for(const auto & joint_name : joint_names_){
    conf.names.push_back(joint_name + "/position");
    std::cout << joint_name + "/position" << std::endl;
  }
  
  return conf;
  
}

void MpNeedleTrajectoryController::needleTipCallback(const geometry_msgs::msg::PoseStamped::SharedPtr needle_tip){
  if(active_){
    // position
    needle_tip_pose[0] = needle_tip->pose.position.x;
    needle_tip_pose[1] = needle_tip->pose.position.y;
    needle_tip_pose[2] = needle_tip->pose.position.z;
    
    // get x and z angles
    tf2::Quaternion tip;
    tf2::fromMsg(needle_tip->pose.orientation, tip);
    
    double x_rot, y_rot, z_rot;
    tf2::Matrix3x3(tip).getRPY(x_rot, y_rot, z_rot);
    
    needle_tip_pose[3] = y_rot;
    needle_tip_pose[4] = z_rot;
    
    if (!tip_set_) {
      tip_set_ = true;
    }
  }
}

void MpNeedleTrajectoryController::targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr target){
  if(active_ && !target_set_){
    
    // set the target point
    target_point[0] = target->point.x;
    target_point[1] = target->point.y;
    target_point[2] = target->point.z;
    
    insertion_length = target->point.x;
    ns = std::floor(insertion_length / insertion_step);
        
    mpc.set_target_pose(target_point);
    
    target_set_ = true;
  }
}

controller_interface::return_type MpNeedleTrajectoryController::update(const rclcpp::Time&, const rclcpp::Duration& period){

  for(int i=0; i<static_cast<int>(joint_state_handles_.size()); ++i){
    stages[i] = joint_state_handles_[i].get().get_value();
    std::cout << "joint: " << i << "pos: " << stages[i] << std::endl;
  }

  depth = stages[3]*1e3;
  float step = std::floor((depth +0.1) / insertion_step);
  
  float H = std::fmin(horizon, ns - step);
  //std::cout << H << std::endl;
  if (H < 0 ) {
  
    target_set_ = false;
    tip_set_ = false;
    
  }

  float step_depth = insertion_step*step;
  
  if (target_set_ && tip_set_) {
    
    tip_set_ = false;
    std::vector<double> cmd = mpc.get_mpc_command(H, step_depth, step, needle_tip_pose, stages, 0);
    
    for (std::size_t i = 0; i < 4; i++) {
      
      std::cout << "joint: " << i << "cmd: " << cmd[i] << std::endl;
      
    }
    
    writeJointControlCmds(cmd, period);

  }

  return controller_interface::return_type::OK;
  
}

// void MpNeedleTrajectoryController::writeJointControlCmds(std::vector<double> cmd){
  
//   for (auto & command_interface : command_interfaces_) {
    
//     for (std::size_t i=0; i<cmd.size(); ++i) {
    
//       command_interface.set_value(cmd[i]);
      
//     }
//   }
// }

void MpNeedleTrajectoryController::writeJointControlCmds(
  const std::vector<double> & cmd,
  const rclcpp::Duration & period)
{

  // for debugging
  if (cmd.size() != joint_cmd_handles_.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu joint commands, got %zu.",
      joint_cmd_handles_.size(),
      cmd.size());
    return;
  }

  if(!command_positions_initialized_ || last_command_positions_.size() != cmd.size()){
    last_command_positions_.resize(cmd.size(), 0.0);
    for(std::size_t i=0; i<cmd.size(); ++i){
      last_command_positions_[i] = joint_state_handles_[i].get().get_value();
    }
    command_positions_initialized_ = true;
  }

  const double max_delta = std::max(0.0, max_velocity_ * period.seconds());
  for (std::size_t i = 0; i < cmd.size(); ++i) {
    const double desired_position = cmd[i] * 1e-3;
    const double limited_position = std::clamp(
      desired_position,
      last_command_positions_[i] - max_delta,
      last_command_positions_[i] + max_delta);
    joint_cmd_handles_[i].get().set_value(limited_position);
    last_command_positions_[i] = limited_position;
  }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(MpNeedleTrajectoryController, controller_interface::ControllerInterface)
