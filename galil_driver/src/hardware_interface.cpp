#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "galil_driver/hardware_interface.hpp"

namespace galil_driver {

  GalilSystemHardwareInterface::~GalilSystemHardwareInterface(){
    on_deactivate(rclcpp_lifecycle::State());
  }
  
  hardware_interface::CallbackReturn
  GalilSystemHardwareInterface::on_init(const hardware_interface::HardwareInfo& info){
    cmd_mode_ = 0;

    if( hardware_interface::SystemInterface::on_init(info) !=
	hardware_interface::CallbackReturn::SUCCESS ){
      return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo & joint : info_.joints){
      for ( std::size_t i=0; i<joint.command_interfaces.size(); i++ ){
	if( joint.command_interfaces[i].name == hardware_interface::HW_IF_POSITION )
	  RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), " %s has position command interface.", joint.name.c_str() );
	if( joint.command_interfaces[i].name == hardware_interface::HW_IF_VELOCITY )
	  RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), " %s has velocity command interface.", joint.name.c_str() );
      }
      for ( std::size_t i=0; i<joint.state_interfaces.size(); i++ ){
	if( joint.state_interfaces[i].name == hardware_interface::HW_IF_POSITION )
	  RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), " %s has position state interface.", joint.name.c_str() );
	if( joint.state_interfaces[i].name == hardware_interface::HW_IF_VELOCITY )
	  RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), " %s has velocity state interface.", joint.name.c_str() );
	if( joint.state_interfaces[i].name == hardware_interface::HW_IF_EFFORT )
	  RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), " has effort state interface.");
      }
    }
    
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
  GalilSystemHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){
    RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), "Configuring ...please wait...");

    GSize BUFFER_LENGTH=32;
    GSize bytes_returned;
    char buffer[32];
    std::string address("169.254.0.51");
    if( GOpen( address.c_str(), &connection ) == G_NO_ERROR ){
      RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), "Connection to Galil successful.");
    }
    else{
      RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "Failed to open connection with Galil.");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // The BWH order:
    // This should be in on_init and search for joint.name instead
    // Likewise, gears should use URDF transmission
    gears_m_2_cnt.push_back(1.0);
    gears_m_2_cnt.push_back(1.0);
    gears_m_2_cnt.push_back(1.0);
    gears_m_2_cnt.push_back(1.0);

    RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), "Successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  
  std::vector<hardware_interface::StateInterface> GalilSystemHardwareInterface::export_state_interfaces(){
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i=0; i<info_.joints.size(); i++){
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
								       hardware_interface::HW_IF_POSITION,
								       &hw_states_position_[i]) );
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
								       hardware_interface::HW_IF_VELOCITY,
								       &hw_states_velocity_[i]) );
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
								       hardware_interface::HW_IF_EFFORT,
								       &hw_states_effort_[i]) );
    }
    return state_interfaces;
  }
  
  std::vector<hardware_interface::CommandInterface> GalilSystemHardwareInterface::export_command_interfaces(){
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i=0; i<info_.joints.size(); i++){
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
									   hardware_interface::HW_IF_POSITION,
									   &hw_commands_position_[i]) );
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
									   hardware_interface::HW_IF_VELOCITY,
									   &hw_commands_velocity_[i]) );
    }
    return command_interfaces;
  }
  
  hardware_interface::CallbackReturn
  GalilSystemHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/){
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  
  hardware_interface::CallbackReturn
  GalilSystemHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/){
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  
  hardware_interface::return_type
  GalilSystemHardwareInterface::prepare_command_mode_switch
  (const std::vector<std::string>& start_interfaces,
   const std::vector<std::string>& stop_interfaces){
    
    std::cout << "GalilSystemHardwareInterface::prepare_command_mode_switch" << std::endl;
    hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

    // Process the interfaces to stop.
    for (const auto& key : stop_interfaces){
      std::cout << "stop: " << key << std::endl;
      
      for(auto i = 0u; i < info_.joints.size(); i++){
	if(key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION){
	  hw_commands_velocity_[i] = 0.0;

	  GSize BUFFER_LENGTH=1024;
	  GSize bytes_returned;
	  char channels[] = "ABCD";
	  char command[1024]="";
	  char buffer[1024];
	  sprintf( command, "ST %c", channels[i]);
	  std::cout << command << std::endl;
	  int error = GCommand(connection, command, buffer, BUFFER_LENGTH, &bytes_returned );
	  if( error != G_NO_ERROR ){
	    RCLCPP_ERROR( rclcpp::get_logger("GalilSystemHardwareInterface"),
			  "Failed to send command: %s, error code %d", command, error);
	  }
	}

	// for velocity we must send stop command
	if(key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY){
	  hw_commands_velocity_[i] = 0.0;

	  GSize BUFFER_LENGTH=1024;
	  GSize bytes_returned;
	  char channels[] = "ABCD";
	  char command[1024]="";
	  char buffer[1024];
	  sprintf( command, "PT 0,0,0,0" ); // turn off position tracking
	  std::cout << command << std::endl;
	  int error = GCommand(connection, command, buffer, BUFFER_LENGTH, &bytes_returned );
	  if( error != G_NO_ERROR ){
	    RCLCPP_ERROR( rclcpp::get_logger("GalilSystemHardwareInterface"),
			  "Failed to send command: %s, error code %d", command, error);
	  }
	}
      }
    }

    for (const auto& key : start_interfaces){
      std::cout << "start: " << key << std::endl;
      for (auto i = 0u; i < info_.joints.size(); i++) {
	if(key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
	  cmd_mode_ = 1;
	  hw_commands_velocity_[i] = 0.0;
	}
	if(key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
	  cmd_mode_ = 2;
	  hw_commands_velocity_[i] = 0.0;
	}
      }
    }

    return ret_val;
  }
  
  hardware_interface::return_type
  GalilSystemHardwareInterface::perform_command_mode_switch
  (const std::vector<std::string>& start_interfaces,
   const std::vector<std::string>& stop_interfaces){

    std::cout << "GalilSystemHardwareInterface::perform_command_mode_switch" << std::endl;
    hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

    for (const auto& key : stop_interfaces){
      std::cout << "stop: " << key << std::endl;
      for(auto i = 0u; i < info_.joints.size(); i++) {
	if(key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION){
	  hw_commands_velocity_[i] = 0.0;
	}
	if(key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY){
	  hw_commands_velocity_[i] = 0.0;
	}
      }
    }
    
    for (const auto& key : start_interfaces){
      std::cout << "start: " << key << std::endl;
      for (auto i = 0u; i < info_.joints.size(); i++) {
	if(key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION){
	  // This is to prevent sending old position when switching
	  std::cout << "switch: " << hw_states_position_[i] << std::endl;
	  hw_commands_position_[i] = hw_states_position_[i];
	  hw_commands_velocity_[i] = 0.0;
	}
	if(key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY){
    hw_commands_position_[i] = hw_states_position_[i];
	  hw_commands_velocity_[i] = 0.0;
	  GSize BUFFER_LENGTH=1024;
	  GSize bytes_returned;
	  char channels[] = "ABCD";
	  char command[1024]="";
	  char buffer[1024];
	  sprintf( command, "PT 1,1,1,1" );
	  std::cout << command << std::endl;
	  int error = GCommand(connection, command, buffer, BUFFER_LENGTH, &bytes_returned );
	  if( error != G_NO_ERROR ){
	    RCLCPP_ERROR( rclcpp::get_logger("GalilSystemHardwareInterface"),
			  "Failed to send command: %s, error code %d", command, error);
	  }
	}
      }
    }
    
    return ret_val;
  }

  
  hardware_interface::return_type GalilSystemHardwareInterface::read( const rclcpp::Time& /*time*/,
								      const rclcpp::Duration& /*period*/){
    //RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), "Reading...");
    GSize BUFFER_LENGTH=1024;
    GSize bytes_returned;
    char buffer[1024];
    if( GCommand(connection, "TP", buffer, BUFFER_LENGTH, &bytes_returned ) == G_NO_ERROR ){
      char * pch;
      pch = strtok(buffer,",");
      for( std::size_t i=0; i<info_.joints.size(); i++ ){
	if(pch != NULL){
	  hw_states_position_[i] = atof( pch )/gears_m_2_cnt[i];
	  pch = strtok (NULL, ",");
	}
      }
    }
    else{
      RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "Failed to send TP command to Galil.");
      hardware_interface::return_type::ERROR;
    }
    if( GCommand(connection, "TV", buffer, BUFFER_LENGTH, &bytes_returned ) == G_NO_ERROR ){
      char * pch;
      pch = strtok(buffer,",");
      for( std::size_t i=0; i<info_.joints.size(); i++ ){
	if(pch != NULL){
	  hw_states_velocity_[i] = atof( pch )/gears_m_2_cnt[i];
	  pch = strtok (NULL, ",");
	}
      }
    }
    else{
      RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "Failed to send TV command to Galil.");
      hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }
  
  hardware_interface::return_type GalilSystemHardwareInterface::write(const rclcpp::Time& /*time*/,
								      const rclcpp::Duration& period){

    char command[1024]="";
    GSize BUFFER_LENGTH=1024;
    GSize bytes_returned;
    char buffer[1024];

    std::cout << period.seconds() << std::endl;
    if( cmd_mode_ == 1 )
      { sprintf( command, "PA " ); }
    if( cmd_mode_ == 2 )
      { sprintf( command, "PA " ); } // for position tracking mode

    for( std::size_t i=0; i<info_.joints.size(); i++ ){

      char separator=',';
      if( i == info_.joints.size()-1 )
	separator=' ';
      
      // The following blocks assume that i follows ABCD
      if( cmd_mode_ == 1 ){
	if( !isnan(hw_commands_position_[i]) && 0<strlen(command) )
	  // Position Absolute command
	  { sprintf( command, "%s%d%c", command, ((int)(hw_commands_position_[i]*gears_m_2_cnt[i])), separator ); }
      }
      if( cmd_mode_ == 2 ){
	if( !isnan(hw_commands_velocity_[i]) && 0<strlen(command) ){
	  // JoG command stinks so we use PT with incremental position
	  hw_commands_position_[i] += period.seconds() * hw_commands_velocity_[i]*gears_m_2_cnt[i];
	  sprintf( command, "%s%d%c", command, ((int)(hw_commands_position_[i]*gears_m_2_cnt[i])), separator );
	}
      }
      
    }

    // if 
    if( 0<strlen(command) ){
      std::cout << command << std::endl;
      int error = GCommand(connection, command, buffer, BUFFER_LENGTH, &bytes_returned );
      if( error == G_NO_ERROR ){
	if( cmd_mode_ == 1 ){
	  error = GCommand(connection, "BG", buffer, BUFFER_LENGTH, &bytes_returned );
	  if( error == G_NO_ERROR ){}
	  else{
	    RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "Failed to send command: %s error code %d",
			 command, error);
	    error = GCommand(connection, "TC1", buffer, BUFFER_LENGTH, &bytes_returned );
	    RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "TC1: %s.", buffer);
	    hardware_interface::return_type::ERROR;
	  }
	}
      }
      else{
	RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "Failed to send command: %s error code %d",
		     command, error);
	error = GCommand(connection, "TC1", buffer, BUFFER_LENGTH, &bytes_returned );
	RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "TC1: %s.", buffer);
	hardware_interface::return_type::ERROR;
      }
    }
    else // empty command
      {/*RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "NaN command. Not sending.");*/}
    
    return hardware_interface::return_type::OK;
  }
  
}
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(galil_driver::GalilSystemHardwareInterface, hardware_interface::SystemInterface)
