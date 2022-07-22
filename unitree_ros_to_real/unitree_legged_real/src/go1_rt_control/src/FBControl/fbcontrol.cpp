/*****************************************************************************
FBCtrlClass.cpp

Description:    source file of FBCtrlClass

@Version:   1.0
@Author:    Jiatao Ding
@Release:   Tue 27 Jun 2017 09:33:32 AM CEST
@Update:    Tue 27 Jun 2017 09:33:37 AM CEST
*****************************************************************************/
#include "fbcontrol.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>


using namespace Eigen;
using namespace std;
using namespace gait;

// namespace gait{

FBCtrlClass::FBCtrlClass()
{
    ////// ***********  end-effectors ********************////
    for(int i = 0; i < 3; i++){
        w_rpy_m[i] = 0;
        w_pos_m[i] = 0;
    dcm_ref[i] = 0;
    dcm_sensor[i] = 0;
    zmp_ref[i] = 0;
    zmp_sensor[i] = 0;
    }                      
    
    tstep = t_period;
    _dt = t_program_cyclic;
T_constant = 20;

_omega_ref = sqrt(_g/Z_c);


    n_tstep = int (round(tstep / _dt));
    if (n_tstep <=1)
    {
    n_tstep = 1;
    }
    else
    {
if (n_tstep >=999)
{ 
    n_tstep = 999;
}	
    }



    z_ctrl = 0;
    rfz_m = 0;
    lfz_m = 0;
    l_r_fz_diff_ref = 0;
    l_r_fz_diff_ref_pre = 0; 
    l_r_fz_diff_mea = 0; 
    l_r_fz_diff_mea_pre = 0;

    fz_det_default = 0; 
    lf_tx_default = 0; 
    lf_ty_default = 0; 
    rf_tx_default = 0; 
    rf_ty_default = 0; 
    body_roll = 0; 
    body_roll_pre = 0; 
    rfoot_roll = 0; 
    rfoot_pitch = 0; 
    lfoot_roll = 0; 
    lfoot_pitch = 0;      


    det_dcmx.setZero();
    det_dcmy.setZero(); 
    


    exp_coeff.setZero();
    for (int i = 1; i <= n_tstep; i++)
    {
        exp_coeff(0,i-1) = exp((i*_dt - tstep)/T_constant);
    }
    
    dcmx_err_integral = 0;
    dcmy_err_integral = 0;

    zmp_mod[0] = 0;
    zmp_mod[1] = 0;

    zmp_mod_pre[0] = 0;
    zmp_mod_pre[1] = 0;

w_pos_m_dcm[0] = 0;
w_pos_m_dcm[1] = 0;
w_pos_m_dcm[2] = 0;

    ki_comx = 0.001;
    ki_comy = 0.001;	
    kd_comx = 0.0004;
    kd_comy = 0.0004;

    fsampling = 1/_dt;
    fcutoff = 10;

    butterworthcomx_mod.init(fsampling,fcutoff);
    butterworthcomy_mod.init(fsampling,fcutoff);


}

FBCtrlClass::~FBCtrlClass()
{

}


//////////////////////=================== lower level- pelvis posture control: not use if the admittance controller is activated ///////////////////////////////////////////
void FBCtrlClass::body_pose_pd_tracking(const double time_program_running, const double *theta_des, const double *theta_des_pre,
                    const double *body_foot_des, const double *body_foot_pos_real, const double *body_foot_des_pre,  const double *body_foot_pos_pre,
                                        const double body_roll, const double *rpy_angle_now,  const double body_roll_pre,  const double *rpy_angle_pre, const double *rpy_angle_last,
                                        const double *com_des, const double *com_sensor,  const double *com_des_pre,  const double *com_sensor_pre, bool admi_ctrl_enable, bool dcm_ctr_enable)
{   
    
    if ((! admi_ctrl_enable)&&(! dcm_ctr_enable))
    {
    ///==================pelvis to foot postion control=================
    w_pos_m[0] = (0.05 * (body_foot_des[0]- body_foot_pos_real[0]) + 0.00005 * ((body_foot_des[0] - body_foot_pos_real[0] - (body_foot_des_pre[0] - body_foot_pos_pre[0])))/(time_program_running)); /// roll
    w_pos_m[1] = (0.075 * (body_foot_des[1]- body_foot_pos_real[1]) + 0.00005 * ((body_foot_des[1] - body_foot_pos_real[1] - (body_foot_des_pre[1] - body_foot_pos_pre[1])))/(time_program_running)); /// pitch

    // //==================global pelvis position control:::::::dangerous if status estimation is inaccurate:::: inactivate =================
    // //w_pos_m[0] = (0.05 * (com_des[0]- com_sensor[0]) + 0.00001 * ((com_des[0] - com_sensor[0] - (com_des_pre[0] - com_sensor_pre[0])))/(time_program_running)); /// roll
    // w_pos_m[1] = (0.05 * (com_des[1]- com_sensor[1]) + 0.00001 * ((com_des[1] - com_sensor[1] - (com_des_pre[1] - com_sensor_pre[1])))/(time_program_running)); /// pitch
    // //w_pos_m[2] = (0.05 * (com_des[2]- com_sensor[2]) + 0.00001 * ((com_des[2] - com_sensor[2] - (com_des_pre[2] - com_sensor_pre[2])))/(time_program_running)); /// yaw:
        
    if ((w_pos_m[0] >= com_pos_max))
    {
        w_pos_m[0] = com_pos_max;
    }
    else
    {
        if ((w_pos_m[0] <= com_pos_min))
        {
            w_pos_m[0] = com_pos_min;
        }
    }

    if ((w_pos_m[1] >= com_pos_max))
    {
        w_pos_m[1] = com_pos_max;
    }
    else
    {
        if ((w_pos_m[1] <= com_pos_min))
        {
            w_pos_m[1] = com_pos_min;
        }
    }

    }

    w_pos_m[2] = (0.01 * (body_foot_des[2]- body_foot_pos_real[2]) + 0.00001 * ((body_foot_des[2] - body_foot_pos_real[2] - (body_foot_des_pre[2] - body_foot_pos_pre[2])))/(time_program_running)); /// yaw:
    if ((w_pos_m[2] >= com_pos_max))
    {
        w_pos_m[2] = com_pos_max;
    }
    else
    {
        if ((w_pos_m[2] <= com_pos_min))
        {
            w_pos_m[2] = com_pos_min;
        }
    }        

    ////be careful that the yaw control is not using!!!!
    w_rpy_m[0] = (0.02 * (theta_des[0] + body_roll- (rpy_angle_now[0] - rpy_angle_last[0])) + 0.00001 * ((body_roll - body_roll_pre) + theta_des[0] - theta_des_pre[0] - (rpy_angle_now[0] - rpy_angle_pre[0]))/(time_program_running)); /// roll
    w_rpy_m[1] = (0.01 * (theta_des[1] - (rpy_angle_now[1] - rpy_angle_last[1])) + 0.00001 * (theta_des[1] - theta_des_pre[1] - (rpy_angle_now[1] - rpy_angle_pre[1]))/(time_program_running)); /// pitch
    //w_rpy_m[2] = (0.1 * (0- (rpy_angle_now[2] - rpy_angle_last[2])) + 0.00001 * (- (rpy_angle_now[2] - rpy_angle_pre[2]))/(time_program_running))*PI/180; /// yaw: 
    
    if ((w_rpy_m[0] >= com_rpy_max))
    {
        w_rpy_m[0] = com_rpy_max ;
    }
    else
    {
        if ((w_rpy_m[0] <= com_rpy_min))
        {
            w_rpy_m[0] = com_rpy_min;
        }
    }

    if ((w_rpy_m[1] >= com_rpy_max))
    {
        w_rpy_m[1] = com_rpy_max;
    }
    else
    {
        if ((w_rpy_m[1] <= com_rpy_min))
        {
            w_rpy_m[1] = com_rpy_min;
        }
    }

    if ((w_rpy_m[2] >= com_rpy_max))
    {
        w_rpy_m[2] = com_rpy_max;
    }
    else
    {
        if ((w_rpy_m[2] <= com_rpy_min))
        {
            w_rpy_m[2] = com_rpy_min;
        }
    }        


}

//// foot_height modification//////
void  FBCtrlClass::foot_height_mod(const double time_program_running, const int time_in_dynamic, const int support_flag, 
                                    const double *lfoot_pose_sensor, const double *rfoot_pose_sensor, const double *l_plan, const double *r_plan, bool admit_ctr_enable)
{
    if (! admit_ctr_enable)
    {
    l_r_fz_diff_ref = l_plan[2] - r_plan[2];
    l_r_fz_diff_mea = lfoot_pose_sensor[2] - rfoot_pose_sensor[2];
    
    /// PD control: for small length walking
    z_ctrl = 0.005 * (l_r_fz_diff_ref - l_r_fz_diff_mea) + 0.0005 * ((l_r_fz_diff_ref - l_r_fz_diff_mea) - (l_r_fz_diff_ref_pre - l_r_fz_diff_mea_pre));


    //// Damping control  
    // double Dam_co_lf_tx = 10000.0;
    // double delta_z_ctrl = z_ctrl;
    // z_ctrl = Damping_controller( l_r_fz_diff_ref,  l_r_fz_diff_mea, Dam_co_lf_tx, delta_z_ctrl);


    if ( time_in_dynamic * time_program_running <=2.5)  //left support
    {
        rfz_m = -0.5 * z_ctrl;
        lfz_m = 0.5 * z_ctrl;            
    }
    else
    {
        if ( support_flag == 0) ////left support
        {
            rfz_m = -z_ctrl;
        }
        else   ////right support
        {
            lfz_m = z_ctrl;
        }
    }           

    /////// clamp modification /////////                                     
    clamp_function(rfz_m, 0.001, -0.001); 
    clamp_function(lfz_m, 0.001, -0.001); 
    l_r_fz_diff_ref_pre = l_r_fz_diff_ref;
    l_r_fz_diff_mea_pre = l_r_fz_diff_mea;	
    }

}

////whole-body admittance control/////
double  FBCtrlClass::Damping_controller(double f_ref, double f_mea, double Dam_co, double delta)
{
    double delta_det =  1.0 / Dam_co *(f_ref - f_mea)  - (1/(2 * tstep)) * delta;
    double detla_mod = delta_det + delta;

    return detla_mod;        
} 

void  FBCtrlClass::foot_com_admittance_control(const double lf_t_x_ref, const double lf_t_y_ref, const double rf_t_x_ref, const double rf_t_y_ref,double fz_det_ref,
                                                const double time_program_running, const geometry_msgs::Wrench left_ft_, const geometry_msgs::Wrench right_ft_,
                        const double *zmp_refx, const double *zmp_sensorx)
{
    // ////// The block is for the nmpc test without phase transition
    //================use < /home/jiatao/airs_ubtech/MPC_demonstration_for_hardware/mpc_code_py_test/> =====/////
    // ////foot force control///////
    // double lf_t_x_mea = left_ft_.torque.x - lf_tx_default;    
    // double Dam_co_lf_tx = 15000.0;
    // double delta_lfoot_roll = lfoot_roll;
    // lfoot_roll = Damping_controller( lf_t_x_ref,  lf_t_x_mea, Dam_co_lf_tx, delta_lfoot_roll);
    
    // double lf_t_y_mea = left_ft_.torque.y - lf_ty_default;   
    // double Dam_co_lf_ty = 15000.0;
    // double delta_lfoot_pitch = lfoot_pitch;
    // lfoot_pitch = Damping_controller( lf_t_y_ref,  lf_t_y_mea, Dam_co_lf_ty, delta_lfoot_pitch);

    // double rf_t_x_mea = right_ft_.torque.x - rf_tx_default;       
    // double Dam_co_rf_tx = 15000.0;
    // double delta_rfoot_roll = rfoot_roll;
    // rfoot_roll = Damping_controller( rf_t_x_ref,  rf_t_x_mea, Dam_co_rf_tx, delta_rfoot_roll);
    
    // double rf_t_y_mea = right_ft_.torque.y - rf_ty_default;    
    // double Dam_co_rf_ty = 15000.0;
    // double delta_rfoot_pitch = rfoot_pitch;
    // rfoot_pitch = Damping_controller( rf_t_y_ref,  rf_t_y_mea, Dam_co_rf_ty, delta_rfoot_pitch);

    // ////// foot_force_difffence_control: reduce Dam_co_fz helps to enhance the stability but amplify the landing impact
    // double fz_det_mea = left_ft_.force.z - right_ft_.force.z - fz_det_default;      
    // double Dam_co_fz = 2500000.0;
    // double delta_z_ctrl = z_ctrl;
    // z_ctrl = Damping_controller( fz_det_ref,  fz_det_mea, Dam_co_fz, delta_z_ctrl);
    // body_roll = z_ctrl/(l[1] - r[1]); ////height difference transform to the body_roll angle;
    // rfz_m = 0.5 * z_ctrl;
    // lfz_m = -0.5 * z_ctrl;

    // ////// CoM_admittance_control     
    // double Dam_co_comx = 100;
    // double Dam_co_comy = 200; /// it turns out that Dam_co_comy maybe a minus: need further testing!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // w_pos_m[0] = 1/2.0 * Dam_co_comx * (zmp_sensor[0] - zmp_refx[0]) * pow(time_program_running,2); 
    // w_pos_m[1] = 1/2.0 * Dam_co_comy * (zmp_sensorx[1] - zmp_refx[1]) * pow(time_program_running,2);
    
    /////====================================================================================================================/////
    ///////////////////////////////////////////// The following code is used for testing swing leg phase transition /////////////
    //1111===============use < /home/jiatao/airs_ubtech/MPC_demonstration_for_hardware/mpc_phase_transition_test/> ==========/////
    // ////// The block is for the nmpc test with mild phase transition： polynomial intepolation test
    // ////foot force control///////
    // double lf_t_x_mea = left_ft_.torque.x - lf_tx_default;    
    // double Dam_co_lf_tx = 10000.0;
    // double delta_lfoot_roll = lfoot_roll;
    // lfoot_roll = Damping_controller( lf_t_x_ref,  lf_t_x_mea, Dam_co_lf_tx, delta_lfoot_roll);
    
    // double lf_t_y_mea = left_ft_.torque.y - lf_ty_default;   
    // double Dam_co_lf_ty = 10000.0;
    // double delta_lfoot_pitch = lfoot_pitch;
    // lfoot_pitch = Damping_controller( lf_t_y_ref,  lf_t_y_mea, Dam_co_lf_ty, delta_lfoot_pitch);

    // double rf_t_x_mea = right_ft_.torque.x - rf_tx_default;       
    // double Dam_co_rf_tx = 10000.0;
    // double delta_rfoot_roll = rfoot_roll;
    // rfoot_roll = Damping_controller( rf_t_x_ref,  rf_t_x_mea, Dam_co_rf_tx, delta_rfoot_roll);
    
    // double rf_t_y_mea = right_ft_.torque.y - rf_ty_default;    
    // double Dam_co_rf_ty = 10000.0;
    // double delta_rfoot_pitch = rfoot_pitch;
    // rfoot_pitch = Damping_controller( rf_t_y_ref,  rf_t_y_mea, Dam_co_rf_ty, delta_rfoot_pitch);

    // ////// foot_force_difffence_control: reduce Dam_co_fz helps to enhance the stability but amplify the landing impact
    // double fz_det_mea = left_ft_.force.z - right_ft_.force.z - fz_det_default;      
    // double Dam_co_fz = 2500000.0;
    // double delta_z_ctrl = z_ctrl;
    // z_ctrl = Damping_controller( fz_det_ref,  fz_det_mea, Dam_co_fz, delta_z_ctrl);
    // body_roll = z_ctrl/(l[1] - r[1]); ////height difference transform to the body_roll angle;
    // rfz_m = 0.5 * z_ctrl;
    // lfz_m = -0.5 * z_ctrl;

    // ////// CoM_admittance_control     
    // double Dam_co_comx = 100;
    // double Dam_co_comy = -3000; ///
    // w_pos_m[0] = 1/2.0 * Dam_co_comx * (zmp_sensorx[0] - zmp_refx[0]) * pow(time_program_running,2); 
    // w_pos_m[1] = 1/2.0 * Dam_co_comy * (zmp_sensorx[1] - zmp_refx[1]) * pow(time_program_running,2);

    //////22222======== The block is for the nmpc test with large phase transition(kmp_mod1): 
    //foot force control///////
    double lf_t_x_mea = left_ft_.torque.x - lf_tx_default;    
    double Dam_co_lf_tx = 500000.0;
    double delta_lfoot_roll = lfoot_roll;
    lfoot_roll = Damping_controller( lf_t_x_ref,  lf_t_x_mea, Dam_co_lf_tx, delta_lfoot_roll);
    
    double lf_t_y_mea = left_ft_.torque.y - lf_ty_default;   
    double Dam_co_lf_ty = 500000.0;
    double delta_lfoot_pitch = lfoot_pitch;
    lfoot_pitch = Damping_controller( lf_t_y_ref,  lf_t_y_mea, Dam_co_lf_ty, delta_lfoot_pitch);

    double rf_t_x_mea = right_ft_.torque.x - rf_tx_default;       
    double Dam_co_rf_tx = 500000.0;
    double delta_rfoot_roll = rfoot_roll;
    rfoot_roll = Damping_controller( rf_t_x_ref,  rf_t_x_mea, Dam_co_rf_tx, delta_rfoot_roll);
    
    double rf_t_y_mea = right_ft_.torque.y - rf_ty_default;    
    double Dam_co_rf_ty = 500000.0;
    double delta_rfoot_pitch = rfoot_pitch;
    rfoot_pitch = Damping_controller( rf_t_y_ref,  rf_t_y_mea, Dam_co_rf_ty, delta_rfoot_pitch);

    // ////// foot_force_difffence_control: reduce Dam_co_fz helps to enhance the stability but amplify the landing impact
    double fz_det_mea = left_ft_.force.z - right_ft_.force.z - fz_det_default;      
    double Dam_co_fz = 5000000.0;
    double delta_z_ctrl = z_ctrl;
    z_ctrl = Damping_controller( fz_det_ref,  fz_det_mea, Dam_co_fz, delta_z_ctrl);
    body_roll = z_ctrl/(l[1] - r[1]); ////height difference transform to the body_roll angle;
    rfz_m = 0.5 * z_ctrl;
    lfz_m = -0.5 * z_ctrl;

    ////// CoM_admittance_control : can be replaced by the pelvis position tracking controller    
    double Dam_co_comx = 100;
    double Dam_co_comy = 200; /// it turns out that Dam_co_comy maybe a minus: need further testing!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    w_pos_m[0] = 1/2.0 * Dam_co_comx * (zmp_sensorx[0] - zmp_refx[0]) * pow(time_program_running,2); 
    w_pos_m[1] = 1/2.0 * Dam_co_comy * (zmp_sensorx[1] - zmp_refx[1]) * pow(time_program_running,2);


    /////// clamp modification /////////
    clamp_function(lfoot_roll, 0.010, -0.010);        
    clamp_function(lfoot_pitch, 0.010, -0.010);
    clamp_function(rfoot_roll, 0.010, -0.010);        
    clamp_function(rfoot_pitch, 0.010, -0.010);
    clamp_function(body_roll, 0.015, -0.015);                                       
    clamp_function(rfz_m, 0.001, -0.001); 
    clamp_function(lfz_m, 0.001, -0.001); 

    if ((w_pos_m[0] >= com_pos_max))
    {
        w_pos_m[0] = com_pos_max;
    }
    else
    {
        if ((w_pos_m[0] <= com_pos_min))
        {
            w_pos_m[0] = com_pos_min;
        }
    }

    if ((w_pos_m[1] >= com_pos_max))
    {
        w_pos_m[1] = com_pos_max;
    }
    else
    {
        if ((w_pos_m[1] <= com_pos_min))
        {
            w_pos_m[1] = com_pos_min;
        }
    }  

    body_roll_pre = body_roll;      

}

void  FBCtrlClass::clamp_function(double &vari, double max_val, double min_val)
{
    if (vari >= max_val)
    {
        vari = max_val;
    }
    else
    {
        if (vari <= min_val)
        {
            vari = min_val;
        }
    }

}



/////// DCM-based CoM tracking controller
///// DCM error integration
void FBCtrlClass::integral_exp()
{

    det_dcmx(0,n_tstep) = dcm_ref[0] - dcm_sensor[0];
    det_dcmy(0,n_tstep) = dcm_ref[1] - dcm_sensor[1];    
    dcmx_err_integral = 0;
    dcmy_err_integral = 0;


    for (int j=1; j<=n_tstep; j++)
    {
    dcmx_err_integral += (exp_coeff(0,j-1) *  det_dcmx(0,j)); 
    dcmy_err_integral += (exp_coeff(0,j-1) *  det_dcmy(0,j)); 
    }
    
    dcmx_err_integral *= (1/T_constant); 
    dcmy_err_integral *= (1/T_constant); 

    

    for (int j=0; j<n_tstep; j++)
    {
    det_dcmx(0,j) = det_dcmx(0,j+1);
    det_dcmy(0,j) = det_dcmy(0,j+1);
    }  
    
}

void FBCtrlClass::zmp_mod_function(double omega_na)
{
    
    FBCtrlClass::integral_exp();
    double Kpx = 5;
    double Kpy = 5;
    double Kix = 20;
    double Kiy = 20;
    double Kzx = 10;
    double Kzy = 10;


    /// for test: assuming refence zmp coincides with the support center

if (abs(omega_na -0.00001) <=0.1)
{
    omega_na = _omega_ref;
}

    zmp_mod[0] = zmp_ref[0] - (1 + Kpx/omega_na) * (dcm_ref[0] - dcm_sensor[0]) 
                    - Kix/omega_na * dcmx_err_integral + Kzx/omega_na * (zmp_ref[0] - zmp_sensor[0]);


    zmp_mod[1] = zmp_ref[1] - (1 + Kpy/omega_na) * (dcm_ref[1] - dcm_sensor[1]) 
                    - Kiy/omega_na * dcmy_err_integral + Kzy/omega_na * (zmp_ref[1] - zmp_sensor[1]);



}


void FBCtrlClass::zmp_mod_com(const double *dcm_refx, const double *dcm_sensorx, const double *zmp_refx, const double *zmp_sensorx,double omega_na)
{
w_pos_m[0] = 0;
w_pos_m[1] = 0;
    dcm_ref[0] = dcm_refx[0];
dcm_ref[1] = dcm_refx[1];
    zmp_ref[0] = zmp_refx[0];
zmp_ref[1] = zmp_refx[1];

    dcm_sensor[0] = dcm_sensorx[0];
dcm_sensor[1] = dcm_sensorx[1];
    zmp_sensor[0] = zmp_sensorx[0];
zmp_sensor[1] = zmp_sensorx[1];	

    zmp_mod_function(omega_na);

    w_pos_m_dcm[0] =  (ki_comx * (zmp_mod[0] - zmp_ref[0]) + kd_comx * (zmp_mod[0] - zmp_mod_pre[0]));
    w_pos_m_dcm[1] =  (ki_comy * (zmp_mod[1] - zmp_ref[1]) + kd_comy * (zmp_mod[1] - zmp_mod_pre[1]));

//         w_pos_m[0] = butterworthcomx_mod.filter(w_pos_m_dcm[0]);
// 	w_pos_m[1] = butterworthcomy_mod.filter(w_pos_m_dcm[1]);
    w_pos_m[0] = w_pos_m_dcm[0];
w_pos_m[1] = w_pos_m_dcm[1];	


    if ((w_pos_m[0] >= com_pos_max))
    {
        w_pos_m[0] = com_pos_max;
    }
    else
    {
        if ((w_pos_m[0] <= com_pos_min))
        {
            w_pos_m[0] = com_pos_min;
        }
    }

    if ((w_pos_m[1] >= com_pos_max))
    {
        w_pos_m[1] = com_pos_max;
    }
    else
    {
        if ((w_pos_m[1] <= com_pos_min))
        {
            w_pos_m[1] = com_pos_min;
        }
    }
    
    zmp_mod_pre[0] = zmp_mod[0]; 
    zmp_mod_pre[1] = zmp_mod[1];        

}    


// }





