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

    extern const double  RobotPara_Z_C;
    extern const double  RobotPara_G; 
    extern const double  RobotPara_FOOT_LENGTH; 
    extern const double  RobotPara_FOOT_WIDTH;
    extern const double  RobotPara_HIP_TO_ANKLE_X_OFFSET; 
    extern const double  RobotParaClass_HALF_HIP_WIDTH;  
  
    /// gait parameters
    extern const double t_period;
    extern const double time_set;    ///squat down time
  
    // robot parameters
    extern const double mass;   //total mass    
    extern const double force_z_limt; // for support leg judgement
    extern const double Z_c; // normal height
    
    
    /// controller parameters
    extern const double com_pos_max;
    extern const double com_pos_min;
    extern const double com_rpy_max;
    extern const double com_rpy_min;    
    
    ///math constant
    extern const double PI;  //pi
    extern const double pi;
    extern const double Rad2Deg;
    extern const double Deg2Rad;

    //physical constants
    extern const double g;   //gravity accelerate
    extern const double _g;

    //ros system constant
    extern const double t_program_cyclic;  //program running time period,

    //Robot model
    extern const int LegMotorNumber;

    extern const double LIP_height;  //LIP model's height

    static const struct sWalkerLegParas {
        const double leg_length[6];        
    }WalkerLegParas = {0.2196/2, 0.098, 0.29, 0.275, 0.141, 0.1563};
    // = {0.2196/2, 0.098, 0.29, 0.275, 0.141, 0.1563}

    //standing init pose
    extern double l[6];
    extern double r[6];
    extern double w[6];

    extern double stand_init_w[6];

    //current limits
    extern const double max_torque;
    extern const double min_torque;

    //motor_controller
    extern const double motor_kp[12];
    extern const double motor_kd[12];
    
    
    
    

}//namespace gait


#endif //ROBOT_CONST_PARA_CONFIG_H
