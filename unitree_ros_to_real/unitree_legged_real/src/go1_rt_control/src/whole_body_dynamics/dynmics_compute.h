/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
Edited by Jiatao Ding, email: jtdingx@gmail.com
************************************************************************/
#pragma once

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
// #include "body/body.h"
#include <Eigen/Dense>



using namespace std;




class Dynamiccclass{
    public: 
        Dynamiccclass();
        ~Dynamiccclass();
        
        double mass;
        double initial_matrix;
        Eigen::Matrix<double, 12,1> joint_torque;

        double stance_kp, stance_kd;
        double swing_kp, swing_kd;
        Eigen::Matrix<double, 3, 4> gravity_compensate;

        ////// torque_control;
        Eigen::Matrix<double, 3,1> compute_joint_torques(Eigen::Matrix<double, 3,3> Jaco,
                                                         bool support_flag, 
                                                         Eigen::Matrix<double, 3,1> p_des,
                                                         Eigen::Matrix<double, 3,1> p_est,
                                                         Eigen::Matrix<double, 3,1> pv_des,
                                                         Eigen::Matrix<double, 3,1> pv_est,
                                                         int leg_number);


        //// force distribution
        Eigen::Matrix<double, 3,4>  F_leg_ref;
        void force_distribution(Eigen::Matrix<double, 3,1> com_des,
                                Eigen::Matrix<double, 12,1> leg_des, 
                                Eigen::Matrix<double, 6,1> F_force_des, 
                                int mode, double y_coefficient);         
};

