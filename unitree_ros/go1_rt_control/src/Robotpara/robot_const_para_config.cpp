//
// Created by chunyuchen on 18-8-23.
//
#include "robot_const_para_config.h"

namespace gait {
  
    const double  dt_mpc_slow = 0.025;   /////first layer MPC-time interval
    const double  dt_mpc_fast = 0.01; /////second layer MPC-time interval

    const double  J_ini = 12 * 0.1*0.1;  /////inertial tensor

    const double  RobotPara_Z_C = 0.309458;
    const double  RobotPara_G = 9.8; 
    const double  RobotPara_FOOT_LENGTH = 0.01; 
    const double  RobotPara_FOOT_WIDTH = 0.01;
    const double  RobotPara_HIP_TO_ANKLE_X_OFFSET = 0.0; 
    const double  RobotParaClass_HALF_HIP_WIDTH = 0.12675;     

  
    /// gait parameters
    const double t_period = 0.7;
    
    
    const double time_set = 2;
    
  
    // robot parameters
    const double mass = 12;   //total mass      
    const double force_z_limt = 50;
    const double Z_c = 0.309458;
    

    const double com_pos_max = 0.004;
    const double com_pos_min = -0.004;
    const double com_rpy_max = -0.0025;
    const double com_rpy_min = -0.0025;    
  
  
//math constant;
    const double PI = 3.141526;  //pi
    const double pi = 3.141526;
    const double Rad2Deg = 180 / pi;
    const double Deg2Rad = pi / 180;

//physical constant
    const double g = 9.8;   //gravity accelerate
    const double _g = 9.8;

//ros system constant
    const double t_program_cyclic = 0.005;  //program running time period

//Robot model
    const int LegMotorNumber = 12;

    const double LIP_height = 0.309458;  //LIP model's height

//standing init pose
    double l[6] = {0, 0.1098, 0, 0, 0, 0};
    double r[6] = {0, -0.1098, 0, 0, 0, 0};
    double w[6] = {0.0, 0.0, 0.565, 0, 0, 0};

    //20181115：0.58的质心高度，可以走0.28的步长: zc = 0.48
    //20181121:0.60的质心高度，可以走0.24: zc = 0.50;
    //20181121:0.56的质心高度，可以走0.30； zc = 0.46
    //double stand_init_w[6] ={0, 0.0, 0.46, 0, 0, 0};
    double stand_init_w[6] ={0.0, 0.0, Z_c, 0, 0, 0};//0.509 ===>0.609
    //dynamic 0.512
    //jumping 0.46
    //bend down 0.40
    //stand on one leg 0.48

//current limits
    const double max_torque = 24000;
    const double min_torque = -24000;

//motor_controller
    const double motor_kp[12] = {400000.0, 300000.0, 300000.0, 400000.0, 1000000.0, 400000.0,
                                 400000.0, 300000.0, 300000.0, 400000.0, 1000000.0, 400000.0};//700000.0

    const double motor_kd[12] = {10000.0,  5000.0,   5000.0,   15000.0,  23000.0,  15000.0,
                                 10000.0,  5000.0,   5000.0,   15000.0,  23000.0,  15000.0};//14000.0
}


