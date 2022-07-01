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
#include "dynmics_compute.h"


using namespace std;
using namespace Eigen;


Dynamiccclass::Dynamiccclass()
{
        mass = 12;
        initial_matrix = 0;
        joint_torque.setZero();

        stance_kp = 10; 
        stance_kd = 1;
        swing_kp = 1;
        swing_kd = 0.01;
        gravity_compensate<< -0.80,   0.80, -0.80, 0.80, 
                                 0,      0,     0,    0,
                                 0,      0,     0,    0; 

        F_leg_ref.setZero();  
}


Dynamiccclass::~Dynamiccclass()
{
}



Eigen::Matrix<double, 3,1> Dynamiccclass::compute_joint_torques(Eigen::Matrix<double, 3,3> Jaco, 
                                                                bool support_flag, 
                                                                Eigen::Matrix<double, 3,1> p_des,
                                                                Eigen::Matrix<double, 3,1> p_est,
                                                                Eigen::Matrix<double, 3,1> pv_des,
                                                                Eigen::Matrix<double, 3,1> pv_est,
                                                                int leg_number)
{   
    Eigen::Matrix<double, 3,1>  joint_torque_des;

    if (support_flag) //// swing foot
    {
       joint_torque_des = -Jaco.transpose() * (swing_kp*(p_des - p_est) + swing_kd*(pv_des - pv_est)) + gravity_compensate.col(leg_number); 
    }
    else
    {
       joint_torque_des = -Jaco.transpose() * F_leg_ref.col(leg_number) + gravity_compensate.col(leg_number);
    //    if (leg_number==0)
    //    {
    //       cout<<"stance F_leg_ref:" << F_leg_ref.col(leg_number).transpose()<<endl;
    //    } 
    }
    
    
    
    


    return joint_torque_des;
}


void Dynamiccclass::force_distribution(Eigen::Matrix<double, 3,1> com_des, 
                                       Eigen::Matrix<double, 12,1> leg_des, 
                                       Eigen::Matrix<double, 6,1> F_force_des, 
                                       int mode, double y_coefficient)
{
    double body_FR_dis = sqrt(pow(com_des(0)-leg_des(0),2) + pow(com_des(1)-leg_des(1),2) + pow(com_des(2)-leg_des(2),2));
    double body_FL_dis = sqrt(pow(com_des(0)-leg_des(3),2) + pow(com_des(1)-leg_des(4),2) + pow(com_des(2)-leg_des(5),2));
    double body_RR_dis = sqrt(pow(com_des(0)-leg_des(6),2) + pow(com_des(1)-leg_des(7),2) + pow(com_des(2)-leg_des(8),2));
    double body_RL_dis = sqrt(pow(com_des(0)-leg_des(9),2) + pow(com_des(1)-leg_des(10),2) + pow(com_des(2)-leg_des(11),2));

    double f_double;
    

    ///// F_leg_ref:  FR, FL, RR, RL ////////
    if (mode == 101) ///// bipedal 
    {
         /////// right two legs is the right leg, left two legs is the left leg
         f_double = F_force_des(0)* body_FL_dis/(body_FL_dis + body_RL_dis); /// RL X;
         F_leg_ref(0,3) = f_double;
         F_leg_ref(0,1) = F_force_des(0) - f_double;
         
         f_double = F_force_des(1)* body_FL_dis/(body_FL_dis + body_RL_dis) *y_coefficient; /// RL Y;
         F_leg_ref(1,3) = f_double;
         F_leg_ref(1,1) = F_force_des(1)*y_coefficient - f_double;        

         f_double = F_force_des(2)* body_FL_dis/(body_FL_dis + body_RL_dis); /// RL Z;
         F_leg_ref(2,3) = f_double;
         F_leg_ref(2,1) = F_force_des(2) - f_double;


         f_double = F_force_des(3)* body_FR_dis/(body_FR_dis + body_RR_dis); /// RR X;
         F_leg_ref(0,2) = f_double;
         F_leg_ref(0,0) = F_force_des(3) - f_double;          

         f_double = F_force_des(4)* body_FR_dis/(body_FR_dis + body_RR_dis) *y_coefficient; /// RR y;
         F_leg_ref(1,2) = f_double;
         F_leg_ref(1,0) = F_force_des(4)*y_coefficient- f_double;  

         f_double = F_force_des(5)* body_FR_dis/(body_FR_dis + body_RR_dis); /// RR Z;
         F_leg_ref(2,2) = f_double;
         F_leg_ref(2,0) = F_force_des(5) - f_double;           
    }
    else
    {
        if (mode == 102) ////troting
        {
            ///// FL,RR is the left leg, FR,RL is the left leg /////
            f_double = F_force_des(0)* body_FL_dis/(body_FL_dis + body_RR_dis); /// RR X;
            F_leg_ref(0,2) = f_double;
            F_leg_ref(0,1) = F_force_des(0) - f_double;
            
            f_double = F_force_des(1)* body_FL_dis/(body_FL_dis + body_RR_dis)*y_coefficient; /// RR Y;
            F_leg_ref(1,2) = f_double;
            F_leg_ref(1,1) = F_force_des(1)*y_coefficient - f_double;        

            f_double = F_force_des(2)* body_FL_dis/(body_FL_dis + body_RR_dis); /// RR Z;
            F_leg_ref(2,2) = f_double;
            F_leg_ref(2,1) = F_force_des(2) - f_double;


            f_double = F_force_des(3)* body_FR_dis/(body_FR_dis + body_RL_dis); /// RL x;
            F_leg_ref(0,3) = f_double;
            F_leg_ref(0,0) = F_force_des(3) - f_double;          

            f_double = F_force_des(4)* body_FR_dis/(body_FR_dis + body_RL_dis)*y_coefficient; /// RL y;
            F_leg_ref(1,3) = f_double;
            F_leg_ref(1,0) = F_force_des(4)*y_coefficient - f_double;  

            f_double = F_force_des(5)* body_FR_dis/(body_FR_dis + body_RL_dis); /// RL z;
            F_leg_ref(2,3) = f_double;
            F_leg_ref(2,0) = F_force_des(5) - f_double;           
        } 
        else  ////bounding
        {
           ////// not testing now ///////////////////
        } 


           
    }
    
 

}


