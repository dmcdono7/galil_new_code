#include <needle_tip_sim/needle_tip_sim.hpp>

NeedleTipSim::NeedleTipSim( const std::string& name ) : Node( name ) {
  sim_tip_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("sensor/tip", 10);
  declare_parameter( "sim_tip_vals_file", rclcpp::ParameterType::PARAMETER_STRING );
  declare_parameter( "period", rclcpp::ParameterType::PARAMETER_INTEGER );
  
  std::string sim_tip_vals_file;
  std::string def = "../data/insertion_points.csv";
  get_parameter_or("sim_tip_vals_file", sim_tip_vals_file, def);
  file.open(sim_tip_vals_file);
}

void NeedleTipSim::publish( ){
  std::string line;
  std::getline(file, line);

  std::stringstream ss(line);
  std::string field;
  std::vector<double> row;

  while (std::getline(ss, field, ',')) {
    row.push_back(std::stod(field));
  }
  
  geometry_msgs::msg::PoseStamped p;
  p.header.stamp = this->now();
  p.header.frame_id = "needle_drive_link";
  p.pose.position.x = row[0];
  p.pose.position.y = row[1];
  p.pose.position.z = row[2];
  
  tf2::Quaternion q;
  q.setRPY(row[3], row[4], row[5]);
  
  p.pose.orientation = tf2::toMsg(q);
  sim_tip_publisher->publish( p );
  
  int period;
  get_parameter_or("period", period, 200);
  rclcpp::sleep_for(std::chrono::milliseconds(period));
}
