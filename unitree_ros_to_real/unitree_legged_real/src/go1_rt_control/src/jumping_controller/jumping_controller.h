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
#include "kinematics/Kinematics.h"
#include "utils/Utils.h"


using namespace UNITREE_LEGGED_SDK;

double stand_duration;
double rate;
Eigen::Matrix<double,3,1> FR_angle_des, FL_angle_des,RR_angle_des, RL_angle_des;

