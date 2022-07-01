/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
Edited by Jiatao Ding, email: jtdingx@gmail.com
************************************************************************/

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include "body.h"
#include <Eigen/Dense>
#include "kinematics/Kinematics.h"
#include "Filter/butterworth_filter.h"
#include "Filter/butterworthLPF.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "whole_body_dynamics/dynmics_compute.h"
#include "utils/Utils.h"

#include "utils/filter.hpp"

using namespace std;
using namespace unitree_model;



int n_count;

double lastPos[12], percent, pos_cmd[12];
double stand_duration;
double time_programming;
int rt_frequency;

Kinematicclass Kine;

Dynamiccclass Dynam;


/////////////*******************8 robot state ******************************************///////////////////////
Eigen::Matrix<double,3,1> body_p_Homing, body_p_des, body_r_des;
Eigen::Matrix<double,3,1> FR_foot_des, FL_foot_des,RR_foot_des, RL_foot_des;
Eigen::Matrix<double,3,1> FR_foot_Homing, FL_foot_Homing,RR_foot_Homing, RL_foot_Homing;
Eigen::Matrix<double,3,1> body_p_est, body_r_est;


Eigen::Matrix<double,3,1> FR_angle_des, FL_angle_des,RR_angle_des, RL_angle_des;
Eigen::Matrix<double,3,1> FR_angle_mea, FL_angle_mea,RR_angle_mea, RL_angle_mea;
Eigen::Matrix<double,3,1> FR_dq_mea, FL_dq_mea,RR_dq_mea, RL_dq_mea;

Eigen::Matrix<double,3,1> FR_foot_relative_des, FL_foot_relative_des,RR_foot_relative_des, RL_foot_relative_des;
Eigen::Matrix<double,3,1> FR_foot_relative_des_old, FL_foot_relative_des_old,RR_foot_relative_des_old, RL_foot_relative_des_old;
Eigen::Matrix<double,3,1> FR_foot_relative_mea, FL_foot_relative_mea,RR_foot_relative_mea, RL_foot_relative_mea;
Eigen::Matrix<double,3,1> FR_foot_relative_mea_old, FL_foot_relative_mea_old,RR_foot_relative_mea_old, RL_foot_relative_mea_old;

Eigen::Matrix<double,3,3> FR_Jaco, FL_Jaco,RR_Jaco, RL_Jaco;
Eigen::Matrix<double,3,3> FR_Jaco_est, FL_Jaco_est,RR_Jaco_est, RL_Jaco_est;

Eigen::Matrix<double,3,1> FR_v_relative, FL_v_relative,RR_v_relative, RL_v_relative;
Eigen::Matrix<double,3,1> FR_v_est_relative, FL_v_est_relative,RR_v_est_relative, RL_v_est_relative;

////state variables
Eigen::Vector3d root_pos;
Eigen::Quaterniond root_quat;
Eigen::Vector3d root_euler;
Eigen::Matrix3d root_rot_mat;
Eigen::Matrix3d root_rot_mat_z;
Eigen::Vector3d root_lin_vel;
Eigen::Vector3d root_ang_vel;
Eigen::Vector3d root_acc;

/////////////////////////////////////////************* Force distribution **********************/////////////////////////////////////////
////// Force distribution 
Eigen::Vector3d F_sum;
Eigen::Matrix3d Momentum_sum;

double rleg_com, lleg_com;
Eigen::Matrix<double,6,1> F_lr_predict;
bool FR_swing,FL_swing,RR_swing,RL_swing;

int bjx1;
Eigen::Matrix<double, 3,1>  FR_torque,FL_torque,RR_torque,RL_torque;
Eigen::Matrix<double, 12,1> Legs_torque;



//// gait mode test:
///  bipedal walking: 101; troting: 102; gallop: 103; bounding: 104; run:105; jump:106;
int gait_mode;

double x_offset;
double y_offset;

////global state estimation:feet trajectory and COM trajectory//////////
int support_flag; /////left: 0; right: 1; double: 2
double fz_double; 
double fz_limit;
double omega_sensor;
double zmp_ref[3], dcm_ref[3], dcm_sensor[3];
Eigen::Matrix<double, 6,1> Force_L_R;

double support_pos_sensor[3]; ///left support by default
double com_sensor[3];
double com_sensor_hip[3];
double com_sensor_pre[3];
double com_des[3];
double comv_des[3];
double coma_des[3];
double com_des_pre[3];
double rfoot_des[3];
double lfoot_des[3];
double rfoot_theta_des[3];
double lfoot_theta_des[3];
double theta_des[3];
double theta_des_pre[3];	
double rfoot_pose_sensor[3];
double lfoot_pose_sensor[3];
double zmp_sensor[3];
bool using_ft_sensor;
Eigen::Matrix<double,23,1> comav_butterworth;
Eigen::Vector2d theta_default;
double comv_sensor[3];
double coma_sensor[3];

Eigen::Vector3d L_com;
Eigen::Vector3d com_estkine;
Eigen::Vector3d cop_estkine;
Eigen::Vector3d theta_estkine;
Eigen::Vector3d thetaa_estkine;
double Fr_estkine;
double Fl_estkine;
Eigen::Vector3d comv_estkine;
Eigen::Matrix<double, 15,1> dob_result;
double J_ini_xx_est,J_ini_yy_est;
Eigen::Vector3d thetav_estkine;



int count_in_mpc_max;

////===============================================================/////////////////
/////////////////// for fast_mpc: body inclination optimization //////////////

sensor_msgs::JointState joint2simulationx;
sensor_msgs::JointState state_to_MPC; /// 1 flag + 18state+3right_leg+3left_foot;

bool mpc_start = false;

Eigen::Matrix<double,25,1> state_feedback;
Eigen::Matrix<double,100,1> slow_mpc_gait;
int mpc_gait_flag,  mpc_gait_flag_old;
Eigen::Matrix<double,45,1> slow_mpc_gait_inte;

int fast_mpc_gait_flag,  fast_mpc_gait_flag_old;
Eigen::Matrix<double,51,1> fast_mpc_gait;	


Eigen::Matrix<double, 21,1> rpy_mpc_body, rfoot_inter, lfoot_inter, bodytheta_inter, rftheta_inter, lftheta_inter, zmp_inter, dcm_inter, comacc_inter;

Eigen::Vector3d COM_in1, COM_in2, COMxyz_ref, COMv_ref, COM_ref2;
Eigen::Vector3d FootL_in1, FootL_in2, FootL_ref, FootLv_ref, FootL_ref2;
Eigen::Vector3d FootR_in1, FootR_in2, FootR_ref, FootRv_ref, FootR_ref2;
Eigen::Vector3d body_in1, body_in2, body_ref, bodyv_ref, body_ref2;
Eigen::Vector3d rfootrpy_in1, rfootrpy_in2, rfootrpy_ref, rfootrpyv_ref, rfootrpy_ref2;
Eigen::Vector3d lfootrpy_in1, lfootrpy_in2, lfootrpy_ref, lfootrpyv_ref, lfootrpy_ref2;
Eigen::Vector3d COMacc_in1, COMacc_in2, COMacc_ref, COMaccv_ref, COMacc_ref2;

Eigen::Vector3d zmp_in1, zmp_in2, zmpxyz_ref, zmpv_ref, zmp_ref2;
Eigen::Vector3d dcm_in1, dcm_in2, dcmxyz_ref, dcmv_ref, dcm_ref2;

Eigen::Vector3d PelvisPos, body_thetax, LeftFootPosx,RightFootPosx;
Eigen::Vector3d F_L, F_R, M_L, M_R;
Eigen::Vector3d LeftFootRPY, RightFootRPY;

Eigen::Matrix<double, 2, 5 > zmp_mpc_ref, rfoot_mpc_ref, lfoot_mpc_ref, bodyangle_mpc_ref;
Eigen::Matrix<double, 3, 5 > comacc_mpc_ref;
Eigen::Matrix<double, 4, 1 > bodyangle_state;
Eigen::Matrix<double, 6, 1 > bodyangle_mpc;


int count_in_rt_loop;
int count_in_rt_ros;
int count_inteplotation;
int count_inteplotation_fast;
int t_int;
int n_t_int;
int n_t_int_fast;
double dtx;
double _mass;
double _j_ini;
double _Zsc;
double _ggg;
    
void nrt_gait_sub_operation(const sensor_msgs::JointState::ConstPtr &msg);
void rt_gait_sub_operation(const sensor_msgs::JointState::ConstPtr &msg);


ros::Subscriber nrt_mpc_gait_subscribe_;
ros::Subscriber gait_des_sub_;

ros::Publisher control_to_rtmpc_pub_;

//// lower-pass-filter///butterworthLPF1/////
butterworthLPF  butterworthLPF1,butterworthLPF2,butterworthLPF3,butterworthLPF4,butterworthLPF5,butterworthLPF6,
butterworthLPF7,butterworthLPF8,butterworthLPF9,butterworthLPF10,butterworthLPF11,butterworthLPF12,
butterworthLPF13,butterworthLPF14,butterworthLPF15,butterworthLPF16,butterworthLPF17,butterworthLPF18,
butterworthLPF19,butterworthLPF20,butterworthLPF21,butterworthLPF22,butterworthLPF23,
butterworthLPF24,butterworthLPF25,butterworthLPF26,butterworthLPF27,butterworthLPF28;
double f_sample_comx1;
double fcutoff_comx1;
double fcutoff_comx2;
double fcutoff_comx3;

double pitch_angle_W;
int n_period;