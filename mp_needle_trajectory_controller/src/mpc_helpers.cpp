#include "mp_needle_trajectory_controller/mpc_helpers.hpp"

/* This class is following the MPC from:
     https://github.com/SmartNeedle/trajcontrol/blob/main/README.md
   It has been significantly modified to use ros2_control, compatability with 
   galil_driver hardware interface, and translation to CPP. 
*/

MpcHelpers::MpcHelpers(float limit, double ins_step) {

  safe_limit = limit;
  insertion_step = ins_step;
  J = Eigen::MatrixXd(5, 3);
  
  J.row(0) << -1.511229017846247302e-01, 6.799925781032750693e-02, 1.839164108832668509e-01;
  J.row(1) << 3.297693406960715279e-02,	9.980663523749191413e-01, 2.872869774199128534e-03;
  J.row(2) << -2.756733735665418267e-02, -5.977784667351204034e-02, -4.326361460576071405e-01;
  J.row(3) << -5.299719365454021690e-04, -5.610543239948689506e-05, 4.145943199272274053e-04;
  J.row(4) << 5.198104733130729785e-04,	-1.750181516897319936e-04, -1.069084685929871589e-03;

}

void MpcHelpers::set_target_pose(std::vector<double> target_vec) {

  target = Eigen::MatrixXd(5, 1);
  
  for (std::size_t i = 0; i < target_vec.size(); i++) {
    target(i, 0) = target_vec[i];
  }

}

void MpcHelpers::update_jacobian() {

  Eigen::MatrixXd delta_base(3,1);
  Eigen::MatrixXd delta_base_T = delta_base.transpose();
  delta_base << (stages[3] - prev_stages[3])*1e3, (stages[1] - prev_stages[1])*1e3, (stages[2] - prev_stages[2])*1e3;
  Eigen::MatrixXd delta_tip = tip - prev_tip;
  
  Eigen::MatrixXd J_delta_base = J * delta_base;
  Eigen::MatrixXd diff = delta_tip - J_delta_base;
  Eigen::MatrixXd denom_mat = delta_base_T * delta_base;
  double denom = denom_mat.norm() + 1e-9;
  
  Eigen::MatrixXd num = diff / denom;

  Eigen::MatrixXd outer = num * delta_base_T;
  
  J = outer;

}

Eigen::MatrixXd MpcHelpers::process_model(Eigen::MatrixXd y0, std::vector<double> u0, std::vector<double> u, Eigen::MatrixXd Jc){

  Eigen::MatrixXd delta_u(3, 1);
  delta_u(0, 0) = insertion_step;
  delta_u(1, 0) = u[0]-u0[0];
  delta_u(2, 0) = u[1]-u0[1];
  
  Eigen::MatrixXd upd = Jc * delta_u;
  Eigen::MatrixXd y = y0 + upd;
  
  return y;

}

double MpcHelpers::objective(const std::vector<double>& uhat, std::vector<double>& /*grad*/, void */*data*/){

  float H = uhat.size() / 2;
  
  std::vector<Eigen::MatrixXd> yhat;
  
  Eigen::MatrixXd yhat0 = tip;
  std::vector<double> uhat0 = {stages[1]*1e3, stages[2]*1e3}; // stage ind 1 is y axis, stage ind 2 is z axis
  
  Eigen::MatrixXd curr_u(1, 2);
  
  for (int i = 0; i < static_cast<int>(H); i++) {
    std::vector<double> curr_u = {uhat[i*2], uhat[i*2+1]};
    Eigen::MatrixXd yp = process_model(yhat0, uhat0, curr_u, J);

    yhat.push_back(yp);

    yhat0 = yhat[i];
    uhat0 = curr_u;
    
  }
  
  Eigen::MatrixXd tg_xz(static_cast<int>(H), 4);
  Eigen::MatrixXd yhat_xz(static_cast<int>(H),4);

  for (int i = 0; i < static_cast<int>(H); i++) {
    
    tg_xz.row(i) << target(1, 0), target(2, 0), target(3, 0), target(4, 0);
    yhat_xz.row(i) << yhat[i](1, 0), yhat[i](2,0), yhat[i](3,0), yhat[i](4,0);
      
  }
  
  Eigen::MatrixXd yhat_xz_T = yhat_xz.transpose();
  
  Eigen::MatrixXd diff = yhat_xz - tg_xz;
  
  
  Eigen::MatrixXd sq = diff.array().square();
  Eigen::MatrixXd weights(4,1);
  weights << 1.0, 1.0, 3.5, 3.5;
  
  Eigen::MatrixXd err2 = sq * weights;

  Eigen::MatrixXd ones = Eigen::MatrixXd::Constant(1,static_cast<int>(H),1);
  
  Eigen::MatrixXd obj1_mat = ones * err2;
  
  double obj1 = obj1_mat.norm();
  
  return obj1;
  
} 

double MpcHelpers::obj_wrapper(const std::vector<double> &x, std::vector<double> &grad, void* data) {
  MpcHelpers* obj = static_cast<MpcHelpers*>(data);
  return obj->objective(x, grad, data);
}

std::vector<double> MpcHelpers::get_mpc_command(float H, double step_depth, int step, std::vector<double> tip_vec, std::vector<double> stages_vec, int test) {
  
  tip = Eigen::MatrixXd(5, 1);
  
  for (std::size_t i = 0; i < tip_vec.size(); i++) {
    tip(i, 0) = tip_vec[i];
  }
  
  stages = stages_vec;
  
  std::vector<double> cmd(4);
  
  std::cout << "tip: " << tip << std::endl;
  
  if (step == 0) {
  
    cmd[0] = 0;
    cmd[1] = 0;
    cmd[2] = 0;
    cmd[3] = 5.0; // x axis which controls insertion
    
  } else if (H > 0) {
    MpcHelpers::update_jacobian();
    std::vector<double> uhat;
    for (int i = 0; i < static_cast<int>(H); i++) {
    
      uhat.push_back(prev_cmd[1]); // y axis
      uhat.push_back(prev_cmd[2]); // z axis
      
    }

    int vars = static_cast<int>(H*2);
    
    nlopt::opt opt(nlopt::LN_COBYLA, vars);
    
    std::vector<double> lb(vars); 
    std::vector<double> ub(vars);
    
    for (int i = 0; i < static_cast<int>(H); i++) {
    
      lb[i*2] = -safe_limit; // y axis lb
      lb[i*2+1] = -safe_limit; // z axis lb
      
      ub[i*2] = safe_limit; // y axis lb
      ub[i*2+1] = safe_limit; // z axis lb
      
    }

    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
    opt.set_xtol_rel(1e-4);
    
    opt.set_min_objective(obj_wrapper, this);
    
    double minf;
    try {
      nlopt::result result = opt.optimize(uhat, minf);
      std::cout << "optimization result: " << result << std::endl;
    } 
    catch(std::exception &e) {
      std::cout << "exception, err code: " << e.what() << std::endl;
    }
    
    if (test == 1) {
      cmd[0] = 0;
      cmd[1] = 0;
      cmd[2] = 0;
      cmd[3] = std::fmin(target(0), step_depth);
      
    } else {
      cmd[0] = 0;
      cmd[1] = uhat[0];
      cmd[2] = uhat[1];
      cmd[3] = std::fmin(target(0), prev_cmd[3] + insertion_step);
    }
    
  } else {
  
    if (test == 1) {
      cmd[0] = 0;
      cmd[1] = 0;
      cmd[2] = 0;
      cmd[3] = std::fmin(target(0), step_depth);
    } else {
      std::cout << "here" << std::endl;
      cmd[0] = 0;
      cmd[1] = stages[1]*1e3;
      cmd[2] = stages[2]*1e3;
      cmd[3] = std::fmin(target(0), prev_cmd[3] + insertion_step);
    }
            
  }
  
  prev_cmd = cmd;
  prev_tip = tip;
  prev_stages = stages;
  
  return cmd;
  
}

