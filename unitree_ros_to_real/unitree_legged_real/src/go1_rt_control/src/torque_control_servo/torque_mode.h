/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/
#pragma once

#include <ros/ros.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "convert.h"
#include "sensor_msgs/JointState.h"
#include <Eigen/Dense>

using namespace UNITREE_LEGGED_SDK;


const int torque_err_row = 500;
////// initial parameters for joint-tracking
double torq_kp_calf, torq_kd_calf, torq_ki_calf;
double torq_kp_thigh, torq_kd_thigh, torq_ki_thigh;
double torq_kp_hip, torq_kd_hip, torq_ki_hip;
Eigen::Matrix<double, torque_err_row,12> torque_err;
Eigen::Matrix<double, 12,1>  torque_err_intergration;
Eigen::Matrix<double, 12,1> Torque_ff; 

/////// spring constant here
bool FF_enable;
double k_spring_calf,k_spring_thigh,k_spring_hip;
double k_p_rest_calf,k_p_rest_thigh,k_p_rest_hip;


//// torque control mode
Eigen::Matrix<double, 12,1>  torque;

