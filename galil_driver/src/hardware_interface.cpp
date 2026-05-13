#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "galil_driver/hardware_interface.hpp"

namespace galil_driver {

  namespace {
    // Mode IDs
    constexpr int COMMAND_MODE_IDLE = 0;
    constexpr int COMMAND_MODE_POSITION = 1;
    constexpr int COMMAND_MODE_LEGACY_VELOCITY = 2;
    constexpr int COMMAND_MODE_REAL_VELOCITY = 3;
    constexpr int GALIL_MIN_NONZERO_JOG_SPEED = 2;
    constexpr const char* HW_IF_REAL_VELOCITY = "real_velocity";
    constexpr const char* REAL_VELOCITY_INIT_PLACEHOLDER = "SET_ME";
    constexpr const char* REAL_VELOCITY_KP_PARAM = "real_velocity_kp";
    constexpr const char* REAL_VELOCITY_KI_PARAM = "real_velocity_ki";
    constexpr const char* REAL_VELOCITY_KD_PARAM = "real_velocity_kd";
    constexpr const char* REAL_VELOCITY_IT_PARAM = "real_velocity_it";
    constexpr const char* REAL_VELOCITY_AC_PARAM = "real_velocity_ac";
    constexpr const char* REAL_VELOCITY_DC_PARAM = "real_velocity_dc";
    constexpr const char* POSITION_MAX_VELOCITY_PARAM = "position_max_velocity";

    // String cleanup
    std::string trim_copy(const std::string& value){
      const auto begin = std::find_if_not(value.begin(), value.end(), [](unsigned char character) {
	return std::isspace(character) != 0;
      });
      const auto end = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char character) {
	return std::isspace(character) != 0;
      }).base();

      if( begin >= end ){
	return "";
      }

      return std::string(begin, end);
    }

    // Galil formatting
    std::string format_galil_number(double value){
      if( std::abs(value - std::llround(value)) < 1e-9 ){
	return std::to_string(std::llround(value));
      }

      std::ostringstream stream;
      stream << std::setprecision(15) << value;
      return stream.str();
    }
  }

  GalilSystemHardwareInterface::~GalilSystemHardwareInterface(){
    on_deactivate(rclcpp_lifecycle::State());
  }

  // Axis mapping
  char GalilSystemHardwareInterface::axis_letter(std::size_t index) const{
    static constexpr char AXES[] = "ABCDEFGH";
    return index < (sizeof(AXES) - 1) ? AXES[index] : '?';
  }

  // Command sender
  bool GalilSystemHardwareInterface::send_galil_command(const std::string& command){
    GSize BUFFER_LENGTH=1024;
    GSize bytes_returned;
    char buffer[1024]="";

    std::cout << command << std::endl;
    int error = GCommand(connection, command.c_str(), buffer, BUFFER_LENGTH, &bytes_returned);
    if( error == G_NO_ERROR ){
      return true;
    }

    RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"),
		 "Failed to send command: %s error code %d", command.c_str(), error);

    char error_buffer[1024]="";
    if( GCommand(connection, "TC1", error_buffer, BUFFER_LENGTH, &bytes_returned) == G_NO_ERROR ){
      RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"),
		   "TC1: %s.", error_buffer);
    }
    else{
      RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"),
		   "Failed to query Galil TC1 after command failure.");
    }

    return false;
  }

  // Velocity scaling
  int GalilSystemHardwareInterface::real_velocity_command_counts(std::size_t index) const{
    if( index >= hw_commands_real_velocity_.size() ){
      return 0;
    }

    const double command = hw_commands_real_velocity_[index];
    if( std::isnan(command) ){
      return 0;
    }

    const double scale = index < gears_m_2_cnt.size() ? gears_m_2_cnt[index] : 1.0;
    const int counts = static_cast<int>(std::lround(command * scale));
    if( std::abs(counts) < GALIL_MIN_NONZERO_JOG_SPEED ){
      return 0;
    }

    return counts;
  }

  // Jog stop
  bool GalilSystemHardwareInterface::stop_real_velocity_axis(std::size_t index){
    if( index >= real_velocity_jog_active_.size() ){
      return true;
    }

    if( !real_velocity_jog_active_[index] ){
      last_real_velocity_counts_[index] = 0;
      return true;
    }

    char command[32]="";
    std::snprintf(command, sizeof(command), "ST %c", axis_letter(index));
    if( !send_galil_command(command) ){
      return false;
    }

    real_velocity_jog_active_[index] = false;
    last_real_velocity_counts_[index] = 0;
    return true;
  }

  // Stop all jogs
  bool GalilSystemHardwareInterface::stop_all_real_velocity_axes(){
    bool ok = true;
    for( std::size_t i=0; i<real_velocity_jog_active_.size(); i++ ){
      ok = stop_real_velocity_axis(i) && ok;
    }
    return ok;
  }

  // PID parsing
  bool GalilSystemHardwareInterface::parse_real_velocity_init_settings(
    std::size_t index,
    RealVelocityInitSettings& settings)
  {
    if( index >= info_.joints.size() ){
      RCLCPP_ERROR(
	rclcpp::get_logger("GalilSystemHardwareInterface"),
	"real_velocity init settings requested for invalid joint index %zu",
	index);
      return false;
    }

    const auto& joint = info_.joints[index];
    const auto parse_param =
      [&](const char* name,
	  double& value,
	  double min_value,
	  bool min_inclusive,
	  bool require_max,
	  double max_value,
	  bool max_inclusive) -> bool
    {
      const auto parameter = joint.parameters.find(name);
      if( parameter == joint.parameters.end() ){
	RCLCPP_ERROR(
	  rclcpp::get_logger("GalilSystemHardwareInterface"),
	  "Joint %s is missing required real_velocity parameter %s",
	  joint.name.c_str(),
	  name);
	return false;
      }

      const std::string raw_value = trim_copy(parameter->second);
      if( raw_value.empty() || raw_value == REAL_VELOCITY_INIT_PLACEHOLDER ){
	RCLCPP_ERROR(
	  rclcpp::get_logger("GalilSystemHardwareInterface"),
	  "Joint %s parameter %s must be replaced with a numeric value before activating real_velocity_controller",
	  joint.name.c_str(),
	  name);
	return false;
      }

      try{
	std::size_t parsed_characters = 0;
	value = std::stod(raw_value, &parsed_characters);
	if( parsed_characters != raw_value.size() ){
	  throw std::invalid_argument("trailing characters");
	}
      }
      catch(const std::exception&){
	RCLCPP_ERROR(
	  rclcpp::get_logger("GalilSystemHardwareInterface"),
	  "Joint %s parameter %s must be numeric, got '%s'",
	  joint.name.c_str(),
	  name,
	  parameter->second.c_str());
	return false;
      }

      if( !std::isfinite(value) ){
	RCLCPP_ERROR(
	  rclcpp::get_logger("GalilSystemHardwareInterface"),
	  "Joint %s parameter %s must be a finite numeric value, got '%s'",
	  joint.name.c_str(),
	  name,
	  parameter->second.c_str());
	return false;
      }

      const bool min_invalid = min_inclusive ? (value < min_value) : (value <= min_value);
      const bool max_invalid = require_max ? (max_inclusive ? (value > max_value) : (value >= max_value)) : false;
      if( min_invalid || max_invalid ){
	if( require_max ){
	  RCLCPP_ERROR(
	    rclcpp::get_logger("GalilSystemHardwareInterface"),
	    "Joint %s parameter %s=%s is out of range %c%g, %g%c",
	    joint.name.c_str(),
	    name,
	    raw_value.c_str(),
	    min_inclusive ? '[' : '(',
	    min_value,
	    max_value,
	    max_inclusive ? ']' : ')');
	}
	else{
	  RCLCPP_ERROR(
	    rclcpp::get_logger("GalilSystemHardwareInterface"),
	    "Joint %s parameter %s=%s must be %s %g",
	    joint.name.c_str(),
	    name,
	    raw_value.c_str(),
	    min_inclusive ? ">=" : ">",
	    min_value);
	}
	return false;
      }

      return true;
    };

    if( !parse_param(REAL_VELOCITY_KP_PARAM, settings.kp, 0.0, true, false, 0.0, true) ||
	!parse_param(REAL_VELOCITY_KI_PARAM, settings.ki, 0.0, true, false, 0.0, true) ||
	!parse_param(REAL_VELOCITY_KD_PARAM, settings.kd, 0.0, true, false, 0.0, true) ||
	!parse_param(REAL_VELOCITY_IT_PARAM, settings.it, 0.004, true, true, 1.0, true) ||
	!parse_param(REAL_VELOCITY_AC_PARAM, settings.ac, 0.0, false, false, 0.0, true) ||
	!parse_param(REAL_VELOCITY_DC_PARAM, settings.dc, 0.0, false, false, 0.0, true) ){
      return false;
    }

    settings.valid = true;
    return true;
  }

  // Vector command
  bool GalilSystemHardwareInterface::send_galil_vector_command(
    const std::string& name,
    const std::vector<double>& values)
  {
    if( values.size() != info_.joints.size() ){
      RCLCPP_ERROR(
	rclcpp::get_logger("GalilSystemHardwareInterface"),
	"Refusing to send %s because value count %zu does not match joint count %zu",
	name.c_str(),
	values.size(),
	info_.joints.size());
      return false;
    }

    std::ostringstream command;
    command << name << " ";
    for( std::size_t i=0; i<values.size(); i++ ){
      if( i != 0 ){
	command << ",";
      }
      command << format_galil_number(values[i]);
    }

    return send_galil_command(command.str());
  }

  // Real-velocity setup
  bool GalilSystemHardwareInterface::initialize_real_velocity_axes(const std::vector<std::size_t>& axes){
    if( axes.empty() ){
      return true;
    }

    std::vector<double> kp_values;
    std::vector<double> ki_values;
    std::vector<double> kd_values;
    std::vector<double> it_values;
    std::vector<double> ac_values;
    std::vector<double> dc_values;

    kp_values.reserve(info_.joints.size());
    ki_values.reserve(info_.joints.size());
    kd_values.reserve(info_.joints.size());
    it_values.reserve(info_.joints.size());
    ac_values.reserve(info_.joints.size());
    dc_values.reserve(info_.joints.size());

    for( std::size_t i=0; i<info_.joints.size(); i++ ){
      RealVelocityInitSettings settings{};
      if( !parse_real_velocity_init_settings(i, settings) ){
	return false;
      }

      real_velocity_init_settings_[i] = settings;
      kp_values.push_back(settings.kp);
      ki_values.push_back(settings.ki);
      kd_values.push_back(settings.kd);
      it_values.push_back(settings.it);
      ac_values.push_back(settings.ac);
      dc_values.push_back(settings.dc);
    }

    return send_galil_vector_command("KP", kp_values) &&
	   send_galil_vector_command("KI", ki_values) &&
	   send_galil_vector_command("KD", kd_values) &&
	   send_galil_vector_command("IT", it_values) &&
	   send_galil_vector_command("AC", ac_values) &&
	   send_galil_vector_command("DC", dc_values);
  }

  // Position speed setup
  bool GalilSystemHardwareInterface::initialize_position_axes(const std::vector<std::size_t>& axes){
    if( axes.empty() ){
      return true;
    }

    if( position_max_velocity_.size() != info_.joints.size() ){
      RCLCPP_ERROR(
	rclcpp::get_logger("GalilSystemHardwareInterface"),
	"Refusing to initialize position control because position_max_velocity count %zu does not match joint count %zu",
	position_max_velocity_.size(),
	info_.joints.size());
      return false;
    }

    std::vector<double> speed_values;
    speed_values.reserve(info_.joints.size());
    for( std::size_t i=0; i<info_.joints.size(); i++ ){
      const double speed_counts = position_max_velocity_[i] * gears_m_2_cnt[i];
      speed_values.push_back(speed_counts);
    }

    return send_galil_vector_command("SP", speed_values);
  }

  // Servo here
  bool GalilSystemHardwareInterface::servo_here_real_velocity_axes(const std::vector<std::size_t>& axes){
    std::string axis_mask;
    for( const auto index : axes ){
      if( index < info_.joints.size() ){
	axis_mask.push_back(axis_letter(index));
      }
    }

    if( axis_mask.empty() ){
      return true;
    }

    for( const auto index : axes ){
      if( index < hw_commands_real_velocity_.size() ){
	hw_commands_real_velocity_[index] = 0.0;
	last_real_velocity_counts_[index] = 0;
	real_velocity_jog_active_[index] = false;
      }
    }

    return send_galil_command("SH " + axis_mask);
  }
  
  // Hardware setup
  hardware_interface::CallbackReturn
  GalilSystemHardwareInterface::on_init(const hardware_interface::HardwareInfo& info){
    cmd_mode_ = COMMAND_MODE_IDLE;

    if( hardware_interface::SystemInterface::on_init(info) !=
	hardware_interface::CallbackReturn::SUCCESS ){
      return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;
    
    auto ip_read = info_.hardware_parameters.find("ip_address");
    if(ip_read != info_.hardware_parameters.end()) { //if it exists
        ip_address_ = ip_read->second; //save as the value
    } else {
        ip_address_ = "169.254.0.51";
        RCLCPP_WARN(rclcpp::get_logger("GalilSystemHardwareInterface"), "ip_address was not found in the URDF. Defaulting to %s", ip_address_.c_str());
    }

    hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_real_velocity_.resize(info_.joints.size(), 0.0);
    last_real_velocity_counts_.resize(info_.joints.size(), 0);
    real_velocity_jog_active_.resize(info_.joints.size(), false);
    real_velocity_init_settings_.resize(
      info_.joints.size(),
      RealVelocityInitSettings{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false});
    position_max_velocity_.clear();

    gears_m_2_cnt.clear();
    torque_constants_.clear();
    estop_triggered_ = false;
    check_timeout_ = 0.1;

    auto time_read = info_.hardware_parameters.find("check_timeout");
    if(time_read != info_.hardware_parameters.end()) { //if it exists
        try{
            check_timeout_ = std::stod(time_read->second); //save as the value
        } catch (const std::exception& e){
            RCLCPP_WARN(rclcpp::get_logger("GalilSystemHardwareInterface"), "Defaulting to 0.1s for timeout");
        } 
    }

    for (const hardware_interface::ComponentInfo & joint : info_.joints){
      double gear_ratio = 1.0;
      auto gear_read = joint.parameters.find("gear_ratio");
      if(gear_read != joint.parameters.end()) {
          try{
              gear_ratio = std::stod(gear_read->second); //string to double
          } catch (const std::exception& e) {
              RCLCPP_WARN(rclcpp::get_logger("GalilSystemHardwareInterface"),"Defaulting to 1.0 for gear ratio");
          }
      }
      gears_m_2_cnt.push_back(gear_ratio);

      double position_max_velocity = 0.0;
      const auto position_max_velocity_read = joint.parameters.find(POSITION_MAX_VELOCITY_PARAM);
      if(position_max_velocity_read == joint.parameters.end()) {
	RCLCPP_ERROR(
	  rclcpp::get_logger("GalilSystemHardwareInterface"),
	  "Joint %s is missing required position control parameter %s",
	  joint.name.c_str(),
	  POSITION_MAX_VELOCITY_PARAM);
	return hardware_interface::CallbackReturn::ERROR;
      }

      const std::string raw_position_max_velocity = trim_copy(position_max_velocity_read->second);
      try{
	std::size_t parsed_characters = 0;
	position_max_velocity = std::stod(raw_position_max_velocity, &parsed_characters);
	if( parsed_characters != raw_position_max_velocity.size() ){
	  throw std::invalid_argument("trailing characters");
	}
      }
      catch (const std::exception&){
	RCLCPP_ERROR(
	  rclcpp::get_logger("GalilSystemHardwareInterface"),
	  "Joint %s parameter %s must be numeric, got '%s'",
	  joint.name.c_str(),
	  POSITION_MAX_VELOCITY_PARAM,
	  position_max_velocity_read->second.c_str());
	return hardware_interface::CallbackReturn::ERROR;
      }
      if( !std::isfinite(position_max_velocity) || position_max_velocity <= 0.0 ){
	RCLCPP_ERROR(
	  rclcpp::get_logger("GalilSystemHardwareInterface"),
	  "Joint %s parameter %s must be a positive finite value, got %s",
	  joint.name.c_str(),
	  POSITION_MAX_VELOCITY_PARAM,
	  position_max_velocity_read->second.c_str());
	return hardware_interface::CallbackReturn::ERROR;
      }
      position_max_velocity_.push_back(position_max_velocity);

      double torque_const = 1.0;
      auto torque_read = joint.parameters.find("torque_constant");
      if(torque_read != joint.parameters.end()) {
          try{
              torque_const = std::stod(torque_read->second); //string to double
          } catch (const std::exception& e) {
              RCLCPP_WARN(rclcpp::get_logger("GalilSystemHardwareInterface"),"Defaulting to 1.0 for torque ratio");
          }
      }
      torque_constants_.push_back(torque_const);


      for ( std::size_t i=0; i<joint.command_interfaces.size(); i++ ){
	if( joint.command_interfaces[i].name == hardware_interface::HW_IF_POSITION )
	  RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), " %s has position command interface.", joint.name.c_str() );
	if( joint.command_interfaces[i].name == hardware_interface::HW_IF_VELOCITY )
	  RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), " %s has velocity command interface.", joint.name.c_str() );
	if( joint.command_interfaces[i].name == HW_IF_REAL_VELOCITY )
	  RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), " %s has real_velocity command interface.", joint.name.c_str() );
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

  // Galil connection
  hardware_interface::CallbackReturn
  GalilSystemHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){
    RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), "Configuring ...please wait...");

    
    if( GOpen( ip_address_.c_str(), &connection ) == G_NO_ERROR ){
      RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), "Connection to Galil successful.");
    }
    else{
      RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "Failed to open connection with Galil.");
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), "Successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  
  // State exports
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
  
  // Command exports
  std::vector<hardware_interface::CommandInterface> GalilSystemHardwareInterface::export_command_interfaces(){
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i=0; i<info_.joints.size(); i++){
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
									   hardware_interface::HW_IF_POSITION,
									   &hw_commands_position_[i]) );
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
									   hardware_interface::HW_IF_VELOCITY,
									   &hw_commands_velocity_[i]) );
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
									   HW_IF_REAL_VELOCITY,
									   &hw_commands_real_velocity_[i]) );
    }
    return command_interfaces;
  }
  
  // Activate hardware
  hardware_interface::CallbackReturn
  GalilSystemHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/){
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  
  // Stop hardware
  hardware_interface::CallbackReturn
  GalilSystemHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/){
    if( !stop_all_real_velocity_axes() ){
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  
  // Prepare switch
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

	if(key == info_.joints[i].name + "/" + HW_IF_REAL_VELOCITY){
	  hw_commands_real_velocity_[i] = 0.0;
	  if( !stop_real_velocity_axis(i) ){
	    ret_val = hardware_interface::return_type::ERROR;
	  }
	  if( cmd_mode_ == COMMAND_MODE_REAL_VELOCITY ){
	    cmd_mode_ = COMMAND_MODE_IDLE;
	  }
	}
      }
    }

    for (const auto& key : start_interfaces){
      std::cout << "start: " << key << std::endl;
      for (auto i = 0u; i < info_.joints.size(); i++) {
	if(key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
	  cmd_mode_ = COMMAND_MODE_POSITION;
	  hw_commands_velocity_[i] = 0.0;
	}
	if(key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
	  cmd_mode_ = COMMAND_MODE_LEGACY_VELOCITY;
	  hw_commands_velocity_[i] = 0.0;
	}
	if(key == info_.joints[i].name + "/" + HW_IF_REAL_VELOCITY) {
	  cmd_mode_ = COMMAND_MODE_REAL_VELOCITY;
	  hw_commands_real_velocity_[i] = 0.0;
	  last_real_velocity_counts_[i] = 0;
	  real_velocity_jog_active_[i] = false;
	}
      }
    }

    return ret_val;
  }
  
  // Perform switch
  hardware_interface::return_type
  GalilSystemHardwareInterface::perform_command_mode_switch
  (const std::vector<std::string>& start_interfaces,
   const std::vector<std::string>& stop_interfaces){

    std::cout << "GalilSystemHardwareInterface::perform_command_mode_switch" << std::endl;
    hardware_interface::return_type ret_val = hardware_interface::return_type::OK;
    std::vector<std::size_t> real_velocity_start_axes;
    std::vector<std::size_t> position_start_axes;

    for (const auto& key : stop_interfaces){
      std::cout << "stop: " << key << std::endl;
      for(auto i = 0u; i < info_.joints.size(); i++) {
	if(key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION){
	  hw_commands_velocity_[i] = 0.0;
	}
	if(key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY){
	  hw_commands_velocity_[i] = 0.0;
	}
	if(key == info_.joints[i].name + "/" + HW_IF_REAL_VELOCITY){
	  hw_commands_real_velocity_[i] = 0.0;
	  if( !stop_real_velocity_axis(i) ){
	    ret_val = hardware_interface::return_type::ERROR;
	  }
	  if( cmd_mode_ == COMMAND_MODE_REAL_VELOCITY ){
	    cmd_mode_ = COMMAND_MODE_IDLE;
	  }
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
	      position_start_axes.push_back(i);
	    }
	if(key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY){
    hw_commands_position_[i] = hw_states_position_[i];
	  hw_commands_velocity_[i] = 0.0;
	  GSize BUFFER_LENGTH=1024;
	  GSize bytes_returned;

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
	if(key == info_.joints[i].name + "/" + HW_IF_REAL_VELOCITY){
	  cmd_mode_ = COMMAND_MODE_REAL_VELOCITY;
	  hw_commands_real_velocity_[i] = 0.0;
	  last_real_velocity_counts_[i] = 0;
	  real_velocity_jog_active_[i] = false;
	  real_velocity_start_axes.push_back(i);
	}
      }
    }

    if( !position_start_axes.empty() ){
      if( !initialize_position_axes(position_start_axes) ){
	ret_val = hardware_interface::return_type::ERROR;
      }
    }

    if( !real_velocity_start_axes.empty() ){
      if( !initialize_real_velocity_axes(real_velocity_start_axes) ||
	  !servo_here_real_velocity_axes(real_velocity_start_axes) ){
	ret_val = hardware_interface::return_type::ERROR;
	cmd_mode_ = COMMAND_MODE_IDLE;
	for( const auto index : real_velocity_start_axes ){
	  if( index < hw_commands_real_velocity_.size() ){
	    hw_commands_real_velocity_[index] = 0.0;
	    last_real_velocity_counts_[index] = 0;
	    real_velocity_jog_active_[index] = false;
	  }
	}
      }
    }
    
    return ret_val;
  }

  
  // State reads
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
      return hardware_interface::return_type::ERROR;
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
      return hardware_interface::return_type::ERROR;
    }

    if( GCommand(connection, "TT", buffer, BUFFER_LENGTH, &bytes_returned ) == G_NO_ERROR ){
      char * pch;
      pch = strtok(buffer,",");
      for( std::size_t i=0; i<info_.joints.size(); i++ ){
        if(pch != NULL){
          hw_states_effort_[i] = atof( pch ) * torque_constants_[i];
          pch = strtok (NULL, ",");
        }
      }
    }
    else{
      RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "Failed to send TT command to Galil.");
      return hardware_interface::return_type::ERROR;
    }

    

    return hardware_interface::return_type::OK;
  }

  // Real-velocity writes
  hardware_interface::return_type GalilSystemHardwareInterface::write_real_velocity(){
    for( std::size_t i=0; i<info_.joints.size(); i++ ){
      const int command_counts = real_velocity_command_counts(i);

      if( command_counts == last_real_velocity_counts_[i] &&
	  (command_counts != 0 || !real_velocity_jog_active_[i]) ){
	continue;
      }

      if( command_counts == 0 ){
	if( !stop_real_velocity_axis(i) ){
	  return hardware_interface::return_type::ERROR;
	}
	continue;
      }

      char jog_command[64]="";
      std::snprintf(jog_command, sizeof(jog_command), "JG%c=%d", axis_letter(i), command_counts);
      if( !send_galil_command(jog_command) ){
	if( real_velocity_jog_active_[i] ){
	  stop_real_velocity_axis(i);
	}
	return hardware_interface::return_type::ERROR;
      }

      if( !real_velocity_jog_active_[i] ){
	char begin_command[32]="";
	std::snprintf(begin_command, sizeof(begin_command), "BG %c", axis_letter(i));
	if( !send_galil_command(begin_command) ){
	  char stop_command[32]="";
	  std::snprintf(stop_command, sizeof(stop_command), "ST %c", axis_letter(i));
	  send_galil_command(stop_command);
	  return hardware_interface::return_type::ERROR;
	}
	real_velocity_jog_active_[i] = true;
      }

      last_real_velocity_counts_[i] = command_counts;
    }

    return hardware_interface::return_type::OK;
  }
  
  // Command writes
  hardware_interface::return_type GalilSystemHardwareInterface::write(const rclcpp::Time& /*time*/,
								      const rclcpp::Duration& period){


    if(period.seconds() > check_timeout_) {
        if(!estop_triggered_) {
            RCLCPP_FATAL(rclcpp::get_logger("GalilSystemHardwareInterface"),"Timeout, the period has triggered an estop.");
            estop_triggered_ = true; 
        }
    }

    if(estop_triggered_) {
        GSize BUFFER_LENGTH=32;
        GSize bytes_returned;
        char buffer[32];
        GCommand(connection, "ST", buffer, BUFFER_LENGTH,&bytes_returned);

        for(std::size_t i=0; i<info_.joints.size(); i++){
            hw_commands_velocity_[i] = 0.0;
            hw_commands_real_velocity_[i] = 0.0;
        }

        return hardware_interface::return_type::OK;
    }


    if( cmd_mode_ == COMMAND_MODE_REAL_VELOCITY ){
      return write_real_velocity();
    }

    char command[1024]="";
    GSize BUFFER_LENGTH=1024;
    GSize bytes_returned;
    char buffer[1024];

    std::cout << period.seconds() << std::endl;
    if( cmd_mode_ == COMMAND_MODE_POSITION )
      { sprintf( command, "PA " ); }
    if( cmd_mode_ == COMMAND_MODE_LEGACY_VELOCITY )
      { sprintf( command, "PA " ); } // for position tracking mode

    for( std::size_t i=0; i<info_.joints.size(); i++ ){

      char separator=',';
      if( i == info_.joints.size()-1 )
	separator=' ';
      
      // The following blocks assume that i follows ABCD
      if( cmd_mode_ == COMMAND_MODE_POSITION ){
	if( !isnan(hw_commands_position_[i]) && 0<strlen(command) )
	  // Position Absolute command
	  { sprintf(command + strlen(command), "%d%c", ((int)(hw_commands_position_[i]*gears_m_2_cnt[i])), separator);}
	  
	  
	  //sprintf( command, "%s%d%c", command, ((int)(hw_commands_position_[i]*gears_m_2_cnt[i])), separator ); }
      }
      if( cmd_mode_ == COMMAND_MODE_LEGACY_VELOCITY ){
	if( !isnan(hw_commands_velocity_[i]) && 0<strlen(command) ){
	  // JoG command stinks so we use PT with incremental position
	  hw_commands_position_[i] += period.seconds() * hw_commands_velocity_[i];
	  sprintf(command + strlen(command), "%d%c", ((int)(hw_commands_position_[i]*gears_m_2_cnt[i])), separator);
	}
      }
      
    }

    // if 
    if( 0<strlen(command) ){
      std::cout << command << std::endl;
      int error = GCommand(connection, command, buffer, BUFFER_LENGTH, &bytes_returned );
      if( error == G_NO_ERROR ){
	if( cmd_mode_ == COMMAND_MODE_POSITION ){
	  error = GCommand(connection, "BG", buffer, BUFFER_LENGTH, &bytes_returned );
	  if( error == G_NO_ERROR ){}
	  else{
	    RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "Failed to send command: %s error code %d",
			 command, error);
	    error = GCommand(connection, "TC1", buffer, BUFFER_LENGTH, &bytes_returned );
	    RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "TC1: %s.", buffer);
	    return hardware_interface::return_type::ERROR;
	  }
	}
      }
      else{
	RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "Failed to send command: %s error code %d",
		     command, error);
	error = GCommand(connection, "TC1", buffer, BUFFER_LENGTH, &bytes_returned );
	RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "TC1: %s.", buffer);
	return hardware_interface::return_type::ERROR;
      }
    }
    else // empty command
      {/*RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "NaN command. Not sending.");*/}
    
    return hardware_interface::return_type::OK;
  }
  
}
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(galil_driver::GalilSystemHardwareInterface, hardware_interface::SystemInterface)
