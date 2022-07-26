#ifndef JUMPING_CONTROLLER_H
#define JUMPING_CONTROLLER_H
#include <Eigen/Dense>
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

class Quadruped{
    public:

    Eigen::Matrix<double,3,1> FR_angle_mea, FL_angle_mea, RR_angle_mea, RL_angle_mea;
    Eigen::Matrix<double,3,1> FR_dq_mea, FL_dq_mea, RR_dq_mea, RL_dq_mea;
    // Body position and orientation:
    Eigen::Vector3d base_pos;
    Eigen::Vector3d base_rpy; // LATER CORRECT FOR DRIFT

    float Kp_joint[12];
    float Kd_joint[12];
    
    Eigen::MatrixXd;

    Quadruped();

    void upload_history(const std::string& history_file, int& n_cols);
    void zeroStates();
    void updateStates(unitree_legged_msgs::LowState RecvLowROS);
    void PIDgains(bool on_ground);
    unitree_legged_msgs::LowCmd publishCommand(unitree_legged_msgs::LowCmd SendLowROS,float qDes[12]);
    void stand_up(float* qDes, int rate_count, const float qInit[12], const float homing_pose_q[12]);
    void select_reference(float* q_des, int rc);
};


#endif