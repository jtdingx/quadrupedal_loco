/*************************************************************
 * Copyright: Walker Team, UBTech
 * Auther: chunyu.chen
 * Build Date: 31/07/2018
 * Modify Date:
 *************************************************************/

#ifndef ROBOT_CONST_PARA_CONFIG_H
#define ROBOT_CONST_PARA_CONFIG_H

#include <ros/ros.h>

/**
 * @namespace gait
 */
namespace gait{
    
  
    //// MPC gait 
    extern const double  dt_mpc_slow;   /////first layer MPC-time interval
    extern const double  dt_mpc_fast;  /////second layer MPC-time interval
    extern const double  J_ini;  /////robot mass
    
    extern const double height_offset_time;
    extern const double height_offsetx; 

    extern const double  RobotPara_Z_C;
    extern const double  RobotPara_G; 
    extern const double  RobotPara_FOOT_LENGTH; 
    extern const double  RobotPara_FOOT_WIDTH;
    extern const double  RobotPara_HIP_TO_ANKLE_X_OFFSET; 
    extern const double  RobotParaClass_HALF_HIP_WIDTH;  
    
    extern const double t_period;
    extern const double footstepsnumber;  //// step number
    
/*    extern const int nh;                    /// =PreviewT/_dt: number of sampling time for predictive window: <= 2*_nT;
    extern const double tstep;              ///step period
    extern const int nT;                    /// _tstep/_dt)  the number of one step cycle
    extern const int nsum;  */                /// number of whole control loop       
    
    extern const double height_offset_time;
    extern const double height_offsetx; 

    
}//namespace gait


#endif //ROBOT_CONST_PARA_CONFIG_H
