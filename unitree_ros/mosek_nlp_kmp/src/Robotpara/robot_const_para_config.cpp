//
// Created by chunyuchen on 18-8-23.
//
#include "robot_const_para_config.h"
#include "math.h"

namespace gait {
  
    const double  dt_mpc_slow = 0.025;    /////first layer MPC-time interval
    const double  dt_mpc_fast = 0.01;  /////second layer MPC-time interval

    const double  J_ini = 12 * 0.1*0.1;  /////inertial tensor

    const double  RobotPara_Z_C = 0.309458;
    const double  RobotPara_G = 9.8; 
    const double  RobotPara_FOOT_LENGTH = 0.01; 
    const double  RobotPara_FOOT_WIDTH = 0.01;
    const double  RobotPara_HIP_TO_ANKLE_X_OFFSET = 0.0; 
    const double  RobotParaClass_HALF_HIP_WIDTH = 0.12675;  
    
    /// gait parameters
    const double t_period = 0.7; //normal walking
    //const double t_period = 1; //stair climbing    
    
    const double footstepsnumber = 27;
    //const double footstepsnumber = 15; // ball hit
    
    const double height_offset_time = 1;
    const double height_offsetx = 0.000; /// walking on the flat ground
    //const double height_offsetx = 0.01; //climbing and resting on the stone
    //const double height_offsetx = 0.015; //climbing multi stairs
    

}


