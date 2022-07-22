/*****************************************************************************
FBCtrlClass.h

Description:	Header file of FBCtrlClass

@Version:	1.0
@Author:	Jiatao Ding
@Release:	Tue 27 Jun 2017 09:31:28 AM CEST
@Update:	Tue 27 Jun 2017 09:31:24 AM CEST
*****************************************************************************/
#pragma once
#include "Robotpara/robot_const_para_config.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>

#include "ros/ros.h"
#include "Filter/butterworth_filter.h"
#include "geometry_msgs/WrenchStamped.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "Filter/butterworthLPF.h"





using namespace Eigen;
using namespace std;


// namespace gait{
class FBCtrlClass
{
    public:
        FBCtrlClass();
        ~FBCtrlClass();
    
        butterworthLPF  butterworthcomx_mod;
        butterworthLPF  butterworthcomy_mod;

        ////reference: <Biped Walking Stabilization Based on Linear Inverted Pendulum Tracking>
        //// low-level controller
        void body_pose_pd_tracking(const double time_program_running, const double *theta_des, const double *theta_des_pre,
                    const double *body_foot_des, const double *body_foot_pos_real, const double *body_foot_des_pre,  const double *body_foot_pos_pre,
                                    const double body_roll, const double *rpy_angle_now,  const double body_roll_pre,  const double *rpy_angle_pre, const double *rpy_angle_last,
                                    const double *com_des, const double *com_sensor,  const double *com_des_pre,  const double *com_sensor_pre, bool admi_ctrl_enable, bool dcm_ctr_enable);

        //// foot height modilation
        void foot_height_mod(const double time_program_running, const int time_in_dynamic, const int support_flag, 
                                const double *lfoot_pose_sensor, const double *rfoot_pose_sensor, const double *l_plan, const double *r_plan, bool admit_ctr_enable);

        ////admittance control/////
        double Damping_controller(double f_ref, double f_mea, double Dam_co, double delta);
        double rfoot_roll, rfoot_pitch, lfoot_roll, lfoot_pitch;
        double body_roll, body_roll_pre, z_ctrl, rfz_m, lfz_m;
        double l_r_fz_diff_ref, l_r_fz_diff_ref_pre, l_r_fz_diff_mea, l_r_fz_diff_mea_pre;

        void foot_com_admittance_control(const double lf_t_x_ref, const double lf_t_y_ref, const double rf_t_x_ref, const double rf_t_y_ref,double fz_det_ref,
                                            const double time_program_running, const geometry_msgs::Wrench left_ft_, const geometry_msgs::Wrench right_ft_,
                                            const double *zmp_refx, const double *zmp_sensorx);
    
        double fz_det_default, lf_tx_default, lf_ty_default, rf_tx_default, rf_ty_default;

        ////clamp function
        void clamp_function(double &vari, double max_val, double min_val);


        ///// reference  <Stair Climbing Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance Control>////////
        //////========== <Online Regeneration of Bipedal Walking Gait Optimizing Footstep Placement and Timing> ========== 
        void integral_exp();
        void zmp_mod_function(double omega_na);
        void zmp_mod_com(const double *dcm_refx, const double *dcm_sensorx, const double *zmp_refx, const double *zmp_sensorx, double omega_na);

        double zmp_mod[2];
        double zmp_mod_pre[2];

        double w_pos_m[3];
        double w_rpy_m[3];

        double dcmx_err_integral, dcmy_err_integral;
    
        double w_pos_m_dcm[3];	    
    
    private: 

        double tstep,T_constant;
        int n_tstep;
        double _dt; 
        double _omega_ref;

        Eigen::Matrix<double,1,1000> det_dcmx, det_dcmy, exp_coeff;
        double ki_comx, ki_comy, kd_comx, kd_comy;
        
        ////// butterworth filter
        double fsampling, fcutoff;
        
        double dcm_ref[3];
        double dcm_sensor[3];
        double zmp_ref[3];
        double zmp_sensor[3];	    
    
    
    
};
// }

