/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "convert.h"
#include "sensor_msgs/JointState.h"
#include <Eigen/Dense>
#include "torque_mode.h"
#include "Robotpara/robot_const_para_config.h"

using namespace UNITREE_LEGGED_SDK;

float qDes[3]={0};
float dqDes[3] = {0}; 
float dqDes_old[3] = {0}; 
int ctrl_estimation = 1000;


template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

double jointLinearInterpolation(double initPos, double targetPos, double rate, int j)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    dqDes[j] = 0.8*dqDes_old[j] + 0.2*(p - qDes[j])/(1.0/ctrl_estimation);
    dqDes_old[j] = dqDes[j];
    return p;
}


template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle n;
    ros::Rate loop_rate(ctrl_estimation);

    ros::Publisher gait_data_pub; // for data_analysis
    ros::Publisher gait_data_pubx;  


    long motiontime=0;
    torque.setZero();

    TCmd SendLowLCM = {0};
    TState RecvLowLCM = {0};
    unitree_legged_msgs::LowCmd SendLowROS;
    unitree_legged_msgs::LowState RecvLowROS;

    sensor_msgs::JointState joint2simulation, joint2simulationx;
    joint2simulation.position.resize(100);
    joint2simulationx.position.resize(100);    
     
    bool initiated_flag = false;  // initiate need time
    
    
    //////// for trajectory generation
    int count = 0;   

    int rate_count = 0;
    int sin_count = 0;
    float qInit[3]={0};

    ////first configure
    // float sin_mid_q[3] = {0.0, 1.2, -2.0};
    ////second configure
    float sin_mid_q[3] = {0.0, 0.8, -1.3};
    double sin_joint1, sin_joint2;


    /////// spring force forward compensation


    k_spring_calf = 3;
    k_p_rest_calf = -1.3;
    k_spring_thigh = 20; 
    k_p_rest_thigh = 0.75;
    k_spring_hip = 0;
    k_p_rest_hip = 0;

    torque_err.setZero();
    torque_err_intergration.setZero();
    Torque_ff.setZero();

    FF_enable = true; ///// Befor setting True, make sure the spring is engaged!!!!!!!!!!!!
    


    ///////// feedback PID joint controller
    torq_kp_calf = 0;
    torq_kd_calf = 0;
    torq_ki_calf = 0;

    torq_kp_thigh = 0; 
    torq_kd_thigh = 0; 
    torq_ki_thigh = 0;

    torq_kp_hip= 0;  
    torq_kd_hip= 0;  
    torq_ki_hip= 0; 

    ////500 Hz
    // if(FF_enable)
    // {
    //     //  feedback plus feedforward
    //     torq_kp_calf = 8;
    //     torq_kd_calf = 0.05;
    // }
    // else
    // {
    //     // // only feedback
    //     torq_kp_calf = 12;
    //     torq_kd_calf = 0.25;
    // }

    ////// 1K hz //////// used in this case
    if(FF_enable)
    {
        //  feedback plus feedforward
        torq_kp_calf = 7;
        torq_kd_calf = 0.3;
        torq_ki_calf = 0.05;
        // // only feedback
        torq_kp_thigh = 8;
        torq_kd_thigh = 0.3;
        torq_ki_thigh = 0.005;        
    }
    else
    {
        // // only feedback
        torq_kp_calf = 8;
        torq_kd_calf = 0.3;
        torq_ki_calf = 0.05;

        // // only feedback
        torq_kp_thigh = 8;
        torq_kd_thigh = 0.3;
        torq_ki_thigh = 0.005;

    }



    gait_mode = 102;
    x_offset = 0.01;

    if (gait_mode ==101) ///bipedal
    {
        y_offset = 0.75;
    }
    else
    {
        if (gait_mode ==102) ///bipedal
        {
            y_offset = 0.15;
        }
        else
        { 
            y_offset = 0.11;
        }        
    }


    //desired joint angles
    FR_angle_des.setZero(); FL_angle_des.setZero(); RR_angle_des.setZero(); RL_angle_des.setZero(); 

    angle_des.setZero();

    //measure angles
    FR_angle_mea.setZero(); FL_angle_mea.setZero(); RR_angle_mea.setZero(); RL_angle_mea.setZero();
    FR_dq_mea.setZero();    FL_dq_mea.setZero();    RR_dq_mea.setZero();    RL_dq_mea.setZero();

    /// Jacobian:
    FR_Jaco.setZero();
    FL_Jaco.setZero();
    RR_Jaco.setZero();
    RL_Jaco.setZero();

    /// Jacobian measured:
    FR_Jaco_est.setZero();
    FL_Jaco_est.setZero();
    RR_Jaco_est.setZero();
    RL_Jaco_est.setZero();  

    FR_v_relative.setZero(); 
    FL_v_relative.setZero();
    RR_v_relative.setZero(); 
    RL_v_relative.setZero();
    FR_v_est_relative.setZero(); 
    FL_v_est_relative.setZero();
    RR_v_est_relative.setZero(); 
    RL_v_est_relative.setZero();




    // desired foot location reatlive to body center;
    FR_foot_relative_des.setZero(); FL_foot_relative_des.setZero();
    RR_foot_relative_des.setZero(); RL_foot_relative_des.setZero();

    FR_foot_relative_des_old.setZero(); FL_foot_relative_des_old.setZero();
    RR_foot_relative_des_old.setZero(); RL_foot_relative_des_old.setZero();    

    // measured foot location reatlive to body center;
    FR_foot_relative_mea.setZero(); FL_foot_relative_mea.setZero();
    RR_foot_relative_mea.setZero(); RL_foot_relative_mea.setZero(); 
    FR_foot_relative_mea_old.setZero(); FL_foot_relative_mea_old.setZero();
    RR_foot_relative_mea_old.setZero(); RL_foot_relative_mea_old.setZero(); 


    // desired body posioin and rotation
    body_p_Homing.setZero();
    FR_foot_Homing.setZero();
    FL_foot_Homing.setZero();
    RR_foot_Homing.setZero();
    RL_foot_Homing.setZero();


    body_p_des.setZero();
    body_r_des.setZero();
    FR_foot_des.setZero(); 
    FL_foot_des.setZero(); 
    RR_foot_des.setZero(); 
    RL_foot_des.setZero();

    body_p_est.setZero();
    body_r_est.setZero();

    root_pos.setZero();
    root_quat.setIdentity();
    root_euler.setZero();
    root_rot_mat.setZero();
    root_rot_mat_z.setZero();
    root_lin_vel.setZero();
    root_ang_vel.setZero();
    root_acc.setZero();

    ////// Force distribution 
    F_sum.setZero();
    Momentum_sum << 2*0.0168352186, 2*0.0004636141, 2*0.0002367952,
                    2*0.0004636141, 2*0.0656071082, 2*3.6671e-05, 
                    2*0.0002367952, 2*3.6671e-05,   2*0.0742720659;


    rleg_com = 0; 
    lleg_com= 0;
    F_lr_predict.setZero();   
    Force_L_R.setZero();   
    bjx1 = 1;

    FR_torque.setZero(); 
    FL_torque.setZero(); 
    RR_torque.setZero(); 
    RL_torque.setZero();  
    Legs_torque.setZero();   

    FR_swing = false; 
    FL_swing = false;
    RR_swing = false;
    RL_swing = false;              

    ////////////////////////// state estimation///////////////////////
    support_flag = 0; /////left: 0; right: 1; double: 2

    fz_double = gait::mass * gait::_g /2; 
    fz_limit = gait::force_z_limt;
    omega_sensor = sqrt(gait::_g/gait::Z_c);

        
    support_pos_sensor[0] = 0; ///left support by default
    support_pos_sensor[1] = gait::RobotParaClass_HALF_HIP_WIDTH; ///left support by default
    support_pos_sensor[2] = 0; ///left support by default
    com_sensor[0] = 0; /////assuming com is 10cm above the pelvis
    com_sensor[1] = 0;
    com_sensor[2] = (gait::Z_c);
    com_sensor_hip[0] = 0; /////assuming com is 10cm above the pelvis
    com_sensor_hip[1] = 0;
    com_sensor_hip[2] = (gait::Z_c);	

    rfoot_pose_sensor[0] = 0;
    rfoot_pose_sensor[1] = -gait::RobotParaClass_HALF_HIP_WIDTH;
    rfoot_pose_sensor[2] = 0;
    lfoot_pose_sensor[0] = 0;
    lfoot_pose_sensor[1] = gait::RobotParaClass_HALF_HIP_WIDTH;
    lfoot_pose_sensor[2] = 0;


    using_ft_sensor = false;       

    for(int i = 0; i < 3; i++){

        com_sensor_pre[i] = 0;
        com_des[i] = 0;
        com_des_pre[i] = 0;
        comv_des[i] = 0;
        coma_des[i] = 0;
        rfoot_des[i] = 0;
        lfoot_des[i] = 0;
        theta_des[i] = 0;
        theta_des_pre[i] = 0;
        rfoot_theta_des[i] = 0;
        lfoot_theta_des[i] = 0;

        comv_sensor[i] = 0;
        coma_sensor[i] = 0;
        zmp_sensor[i] = 0;
        zmp_ref[i] = 0;
        dcm_ref[i]= 0;
        dcm_sensor[i] = 0;        

    }

    com_des[2] = gait::Z_c;

    rfoot_des[1] = -gait::RobotParaClass_HALF_HIP_WIDTH;
    lfoot_des[1] = gait::RobotParaClass_HALF_HIP_WIDTH;

    comav_butterworth.setZero();
    theta_default.setZero();


    L_com.setZero();
    com_estkine.setZero();
    cop_estkine.setZero();
    theta_estkine.setZero();
    thetaa_estkine.setZero();
    Fr_estkine = 0;
    Fl_estkine = 0;
    comv_estkine.setZero();
    dob_result.setZero();
    J_ini_xx_est = 0;
    J_ini_yy_est = 0;
    thetav_estkine.setZero();




    ////////////////////// real-time mpc loop /////////////////////////	
    count_in_mpc_max = 1000;
    count_in_rt_loop = 0;

	joint2simulationx.position.resize(100);
	state_to_MPC.position.resize(25);
	state_feedback.setZero();
	slow_mpc_gait.setZero();   
	mpc_gait_flag = 0;
	mpc_gait_flag_old = 0;
	slow_mpc_gait_inte.setZero(); 
	
	fast_mpc_gait_flag = 0;
	fast_mpc_gait_flag_old = 0;
	fast_mpc_gait.setZero();	
	
	
	COM_in1.setZero(); 
	COM_in2.setZero(); 
	COMxyz_ref.setZero(); 
	COM_ref2.setZero(); 
	COM_in1(2) = COM_in2(2) = COMxyz_ref(2) = COM_ref2(2) = gait::RobotPara_Z_C; 

	COMv_ref.setZero(); 

	COMacc_in1.setZero();  
	COMacc_in2.setZero();  
	COMacc_ref.setZero();  
	COMaccv_ref.setZero();  
	COMacc_ref2.setZero();  

	FootL_in1.setZero();  
	FootL_in2.setZero();  
	FootL_ref.setZero();  
	FootL_ref2.setZero();  
	FootL_in1(1) = FootL_in2(1)= FootL_ref(1) = FootL_ref2(1) = gait::RobotParaClass_HALF_HIP_WIDTH; 
	FootLv_ref.setZero(); 

	FootR_in1.setZero();  
	FootR_in2.setZero();  
	FootR_ref.setZero();  
	FootR_in1(1) = FootR_in2(1)= FootR_ref(1) = FootR_ref2(1) = -gait::RobotParaClass_HALF_HIP_WIDTH; 
	FootRv_ref.setZero(); 

	zmp_in1.setZero(); zmp_in2.setZero(); zmpxyz_ref.setZero(); zmpv_ref.setZero(); zmp_ref2.setZero();
	dcm_in1.setZero(); dcm_in2.setZero(); dcmxyz_ref.setZero(); dcmv_ref.setZero(); dcm_ref2.setZero();

	body_in1.setZero(); body_in2.setZero(); body_ref.setZero(); bodyv_ref.setZero(); body_ref2;
	rfootrpy_in1.setZero(); rfootrpy_in2.setZero(); rfootrpy_ref.setZero(); rfootrpyv_ref.setZero(); rfootrpy_ref2.setZero();
	lfootrpy_in1.setZero(); lfootrpy_in2.setZero(); lfootrpy_ref.setZero(); lfootrpyv_ref.setZero(); lfootrpy_ref2.setZero();



	PelvisPos.setZero();  
	body_thetax.setZero();  
	LeftFootPosx.setZero(); 
	RightFootPosx.setZero();  
	dcmxyz_ref.setZero();  
	F_L.setZero();  
	F_R.setZero(); 
	M_L.setZero();  
	M_R.setZero(); 
	LeftFootRPY.setZero();  
	RightFootRPY.setZero(); 
	
	rpy_mpc_body.setZero();
	rpy_mpc_body(2) = COM_ref2(2);

	rfoot_inter.setZero();
	rfoot_inter(1) = FootR_ref2(1);

	lfoot_inter.setZero();
	lfoot_inter(1) = FootL_ref2(1);

	bodytheta_inter.setZero();

	rftheta_inter.setZero(); 
	lftheta_inter.setZero();

	zmp_inter.setZero(); 
	dcm_inter.setZero();

	zmp_mpc_ref.setZero(); rfoot_mpc_ref.setZero(); lfoot_mpc_ref.setZero(); bodyangle_mpc_ref.setZero(); comacc_mpc_ref.setZero();
	bodyangle_mpc.setZero();
	bodyangle_state.setZero();

	count_in_rt_loop = 0;
	count_in_rt_ros = 0;

	count_inteplotation = 0;
	count_inteplotation_fast = 0;
	t_int = 0;
	dtx = gait::t_program_cyclic;   
	n_t_int = (int) round(gait::dt_mpc_slow /dtx); 
	n_t_int_fast = (int) round(gait::dt_mpc_fast /dtx); 


	_mass = gait::mass;
	_j_ini = gait::J_ini;

	_Zsc = 0;
	_ggg = gait::g;	
	
	//// gait filter
    f_sample_comx1 = 1/gait::t_program_cyclic;
	fcutoff_comx1 = 3;
	fcutoff_comx2 = 10;
	butterworthLPF1.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF2.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF3.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF4.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF5.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF6.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF7.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF8.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF9.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF10.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF11.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF12.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF13.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF14.init(f_sample_comx1,fcutoff_comx2);	

	
	fcutoff_comx3 = 20;
	butterworthLPF15.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF16.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF17.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF18.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF19.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF20.init(f_sample_comx1,fcutoff_comx3);
	butterworthLPF21.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF22.init(f_sample_comx1,fcutoff_comx3);
    butterworthLPF23.init(f_sample_comx1,fcutoff_comx3);    
	butterworthLPF24.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF25.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF26.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF27.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF28.init(f_sample_comx1,fcutoff_comx3);	
/*	butterworthLPF29.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF30.init(f_sample_comx1,fcutoff_comx1);*/	
    
    pitch_angle_W = 0;

    n_period = round(gait::t_period / gait::t_program_cyclic); 





    roslcm.SubscribeState();
    gait_data_pub = n.advertise<sensor_msgs::JointState>("go1_gait_data",10);
    gait_data_pubx = n.advertise<sensor_msgs::JointState>("go1_gait_datax",10);       

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    SendLowROS.levelFlag = LOWLEVEL;
    for(int i = 0; i<12; i++){
        SendLowROS.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
    }




    rt_frequency = ctrl_estimation; /// frequency of lower_level control
    time_programming = 1.0/rt_frequency;

    n_count = 0;
    stand_duration = 5; /// stand up: 2s
    
    



    while (ros::ok()){
        roslcm.Get(RecvLowLCM);
        RecvLowROS = ToRos(RecvLowLCM);
        printf("FR_1 position: %f\n",  RecvLowROS.motorState[FR_1].q);
        printf("FR force: %f\n",  RecvLowROS.footForceEst[FR_]);


        count_in_rt_ros += 1;

        ////////////////////////// kinematic-based state estimation ///////////////////////////////////////////////
        // Forward kinematics: for kinematic-based state estimation //////////////////////////////////////////////
        FR_angle_mea(0,0) = RecvLowROS.motorState[FR_0].q;
        FR_angle_mea(1,0) = RecvLowROS.motorState[FR_1].q;
        FR_angle_mea(2,0) = RecvLowROS.motorState[FR_2].q;
        FL_angle_mea(0,0) = RecvLowROS.motorState[FL_0].q;
        FL_angle_mea(1,0) = RecvLowROS.motorState[FL_1].q;
        FL_angle_mea(2,0) = RecvLowROS.motorState[FL_2].q;
        RR_angle_mea(0,0) = RecvLowROS.motorState[RR_0].q;
        RR_angle_mea(1,0) = RecvLowROS.motorState[RR_1].q;
        RR_angle_mea(2,0) = RecvLowROS.motorState[RR_2].q;
        RL_angle_mea(0,0) = RecvLowROS.motorState[FR_0].q;
        RL_angle_mea(1,0) = RecvLowROS.motorState[FR_1].q;
        RL_angle_mea(2,0) = RecvLowROS.motorState[FR_2].q;

        FR_dq_mea(0,0) = RecvLowROS.motorState[0].dq;
        FR_dq_mea(1,0) = RecvLowROS.motorState[1].dq;
        FR_dq_mea(2,0) = RecvLowROS.motorState[2].dq;
        FL_dq_mea(0,0) = RecvLowROS.motorState[3].dq;
        FL_dq_mea(1,0) = RecvLowROS.motorState[4].dq;
        FL_dq_mea(2,0) = RecvLowROS.motorState[5].dq;
        RR_dq_mea(0,0) = RecvLowROS.motorState[6].dq;
        RR_dq_mea(1,0) = RecvLowROS.motorState[7].dq;
        RR_dq_mea(2,0) = RecvLowROS.motorState[8].dq;
        RL_dq_mea(0,0) = RecvLowROS.motorState[9].dq;
        RL_dq_mea(1,0) = RecvLowROS.motorState[10].dq;
        RL_dq_mea(2,0) = RecvLowROS.motorState[11].dq;


        //// IMU quaternion
        root_quat = Eigen::Quaterniond(RecvLowROS.imu.quaternion[0],
                                       RecvLowROS.imu.quaternion[1],
                                       RecvLowROS.imu.quaternion[2],
                                       RecvLowROS.imu.quaternion[3]);         

        // euler angle: roll pitch yaw
        root_rot_mat = root_quat.toRotationMatrix();
        root_euler = Utils::quat_to_euler(root_quat);
        body_r_est(0,0) = root_euler(0);
        body_r_est(1,0) = root_euler(1);
        body_r_est(2,0) = root_euler(2);

        double yaw_angle = root_euler[2];
        root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

        /// relative to body center. global framework;
        FR_foot_relative_mea = Kine.Forward_kinematics_g(body_p_est, body_r_est,FR_angle_mea, 0);
        FR_Jaco_est = Kine.Jacobian_kin;
        FL_foot_relative_mea = Kine.Forward_kinematics_g(body_p_est, body_r_est,FL_angle_mea, 1);
        FL_Jaco_est = Kine.Jacobian_kin;
        RR_foot_relative_mea = Kine.Forward_kinematics_g(body_p_est, body_r_est,RR_angle_mea, 2);
        RR_Jaco_est = Kine.Jacobian_kin;
        RL_foot_relative_mea = Kine.Forward_kinematics_g(body_p_est, body_r_est,RL_angle_mea, 3);
        RL_Jaco_est = Kine.Jacobian_kin;
        
        //// relative velocity in global framework: another way is to use differential of measured retlative position;
        // FR_v_est_relative = FR_Jaco_est * FR_dq_mea;
        // FL_v_est_relative = FL_Jaco_est * FL_dq_mea; 
        // RR_v_est_relative = RR_Jaco_est * RR_dq_mea;
        // RL_v_est_relative = RL_Jaco_est * RL_dq_mea; 
        
        if (count_in_rt_ros > 1)
        {
            FR_v_est_relative = (FR_foot_relative_mea - FR_foot_relative_mea_old)/dtx;
            FL_v_est_relative = (FL_foot_relative_mea - FL_foot_relative_mea_old)/dtx;
            RR_v_est_relative = (RR_foot_relative_mea - RR_foot_relative_mea_old)/dtx;
            RL_v_est_relative = (RL_foot_relative_mea - RL_foot_relative_mea_old)/dtx;            
        }






        //////////////////// gait control loop/////////////////////////////////////////////////////
        /// *****************joint cmd generation*******************////
        double targetPos[12]  = {0.0, 0.8, -1.3, 0.0, 0.8, -1.3, 0.0, 0.8, -1.3, 0.0, 0.8, -1.3};  
        // reference angle generation: a simple test
        if (n_count*time_programming <= stand_duration) 
        {
            //****************Homing_pose*******************
            for(int j=0; j<12; j++) lastPos[j] = RecvLowROS.motorState[j].q;

            percent = pow((n_count*time_programming)/stand_duration,2);
            for(int j=0; j<12; j++){
                SendLowROS.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent; 
            }

            FR_angle_des(0,0) = SendLowROS.motorCmd[0].q;
            FR_angle_des(1,0) = SendLowROS.motorCmd[1].q;
            FR_angle_des(2,0) = SendLowROS.motorCmd[2].q;
            FL_angle_des(0,0) = SendLowROS.motorCmd[3].q;
            FL_angle_des(1,0) = SendLowROS.motorCmd[4].q;
            FL_angle_des(2,0) = SendLowROS.motorCmd[5].q;
            RR_angle_des(0,0) = SendLowROS.motorCmd[6].q;
            RR_angle_des(1,0) = SendLowROS.motorCmd[7].q;
            RR_angle_des(2,0) = SendLowROS.motorCmd[8].q;
            RL_angle_des(0,0) = SendLowROS.motorCmd[9].q;
            RL_angle_des(1,0) = SendLowROS.motorCmd[10].q;
            RL_angle_des(2,0) = SendLowROS.motorCmd[11].q;   

            FR_foot_des = FR_foot_relative_des = Kine.Forward_kinematics(FR_angle_des, 0);
            FL_foot_des = FL_foot_relative_des = Kine.Forward_kinematics(FL_angle_des, 1);
            RR_foot_des = RR_foot_relative_des = Kine.Forward_kinematics(RR_angle_des, 2);
            RL_foot_des = RL_foot_relative_des = Kine.Forward_kinematics(RL_angle_des, 3);  
            
            /// computing the homing body position:
            body_p_Homing(2,0) = body_p_des(2,0) = - (FR_foot_relative_des(2,0) + FL_foot_relative_des(2,0) + RR_foot_relative_des(2,0) + RL_foot_relative_des(2,0))/4;                   
            FR_foot_des(2) = 0;
            FL_foot_des(2) = 0;
            RR_foot_des(2) = 0;
            RL_foot_des(2) = 0;

            FR_foot_Homing = FR_foot_des;
            FL_foot_Homing = FL_foot_des;
            RR_foot_Homing = RR_foot_des;
            RL_foot_Homing = RL_foot_des;
            //cout<< "xxxx!!!!!!!!!!!!!!!!!!!!xxxxxxxx"<<endl;      
        }







        // gravity compensation
        SendLowROS.motorCmd[FR_0].tau = -0.65f;
        SendLowROS.motorCmd[FL_0].tau = +0.65f;
        SendLowROS.motorCmd[RR_0].tau = -0.65f;
        SendLowROS.motorCmd[RL_0].tau = +0.65f;
        
        if(initiated_flag == true){
            motiontime++;            
            if( motiontime >= 0){
                // first, get record initial position
                if( motiontime >= 0 && motiontime < 500){
                    qInit[0] = RecvLowROS.motorState[FR_0].q;
                    qInit[1] = RecvLowROS.motorState[FR_1].q;
                    qInit[2] = RecvLowROS.motorState[FR_2].q;

                    qDes[0] = qInit[0];
                    qDes[1] = qInit[1];
                    qDes[2] = qInit[2];

                }
                if( motiontime >= 500 && motiontime < 900){
                    // printf("%f %f %f\n", );
                    rate_count++;
                    double rate = rate_count/200.0;                       // needs count to 200

                    qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate, 0);
                    qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate, 1);
                    qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate, 2);
                }
                // last, do sine wave
                if( motiontime >= 900){
                    sin_count++;
                    sin_joint1 = 0.4 * sin(3*M_PI*sin_count/1000.0);
                    sin_joint2 = -0.3 * sin(1.8*M_PI*sin_count/1000.0);
                    qDes[0] = sin_mid_q[0];
                    qDes[1] = sin_mid_q[1] + sin_joint1;
                    //first configure
                    qDes[2] = sin_mid_q[2] + sin_joint2;
                    dqDes[2] = -0.3 * (1.8*M_PI/1000.0) * cos(1.8*M_PI*sin_count/1000.0);
                }

                if(qDes[2]<=go1_Calf_min)
                {
                    qDes[2]=go1_Calf_min;
                }
                else if (qDes[2]>=go1_Calf_max)
                {
                    qDes[2]=go1_Calf_max;
                }
 
                if(qDes[1]<=go1_Thigh_min)
                {
                    qDes[1]=go1_Thigh_min;
                }
                else if (qDes[1]>=go1_Thigh_max)
                {
                    qDes[1]=go1_Thigh_max;
                }               

                if(qDes[0]<=go1_Hip_min)
                {
                    qDes[0]=go1_Hip_min;
                }
                else if (qDes[0]>=go1_Hip_max)
                {
                    qDes[0]=go1_Hip_max;
                } 


                //// FR_0 joint tracking
                torque_err.block<torque_err_row-1,1>(0,FR_0) = torque_err.block<torque_err_row-1,1>(1,FR_0);

                torque_err(torque_err_row-1,FR_0) = qDes[0] - RecvLowROS.motorState[FR_0].q;

                torque_err_intergration.setZero();
                for(int ij=0; ij<torque_err_row; ij++)
                {
                   torque_err_intergration(FR_0,0) += torque_err(ij,FR_0);
                } 
                 
                torque(FR_0,0) = (qDes[0] - RecvLowROS.motorState[FR_0].q)*torq_kp_calf + (0 - RecvLowROS.motorState[FR_0].dq)*torq_kd_calf + torque_err_intergration(FR_0,0)*torq_ki_calf;
                
                if(qDes[0]<=k_p_rest_calf)
                {
                    Torque_ff(FR_0,0) = k_spring_calf * (qDes[0] - (k_p_rest_calf));
                }
                else
                {
                    Torque_ff(FR_0,0) = 0;
                }
 
                if(FF_enable)
                {
                    torque(FR_0,0) += Torque_ff(FR_0,0);
                }  

                //// FR_1 joint tracking
                torque_err.block<torque_err_row-1,1>(0,FR_1) = torque_err.block<torque_err_row-1,1>(1,FR_1);

                torque_err(torque_err_row-1,FR_1) = qDes[1] - RecvLowROS.motorState[FR_1].q;

                torque_err_intergration.setZero();
                for(int ij=0; ij<torque_err_row; ij++)
                {
                   torque_err_intergration(FR_1,0) += torque_err(ij,FR_1);
                } 
                 
                torque(FR_1,0) = (qDes[1] - RecvLowROS.motorState[FR_1].q)*torq_kp_thigh + (0 - RecvLowROS.motorState[FR_1].dq)*torq_kd_thigh + torque_err_intergration(FR_1,0)*torq_ki_thigh;
                
                // if(qDes[1]>=k_p_rest_thigh)
                // {
                //     Torque_ff(FR_1,0) = k_spring_thigh * (qDes[1] - (k_p_rest_thigh));
                // }
                // else
                // {
                //     Torque_ff(FR_1,0) = 0;
                // }
                Torque_ff(FR_1,0) = log(1+exp(k_spring_thigh*(qDes[1] - (k_p_rest_thigh))));
     
                if(FF_enable)
                {
                    torque(FR_1,0) += Torque_ff(FR_1,0);
                }               

                //// FR_2 joint tracking
                torque_err.block<torque_err_row-1,1>(0,FR_2) = torque_err.block<torque_err_row-1,1>(1,FR_2);

                torque_err(torque_err_row-1,FR_2) = qDes[2] - RecvLowROS.motorState[FR_2].q;

                torque_err_intergration.setZero();
                for(int ij=0; ij<torque_err_row; ij++)
                {
                   torque_err_intergration(FR_2,0) += torque_err(ij,FR_2);
                } 
                 
                torque(FR_2,0) = (qDes[2] - RecvLowROS.motorState[FR_2].q)*torq_kp_calf + (0 - RecvLowROS.motorState[FR_2].dq)*torq_kd_calf + torque_err_intergration(FR_2,0)*torq_ki_calf;
                
                if(qDes[2]<=k_p_rest_calf)
                {
                    Torque_ff(FR_2,0) = k_spring_calf * (qDes[2] - (k_p_rest_calf));
                }
                else
                {
                    Torque_ff(FR_2,0) = 0;
                }
                

                if(FF_enable)
                {
                    torque(FR_2,0) += Torque_ff(FR_2,0);
                }
                



                //// FL_2 joint
                torque(FL_2,0) = (qDes[2] - RecvLowROS.motorState[FL_2].q)*torq_kp_calf + (0 - RecvLowROS.motorState[FL_2].dq)*torq_kd_calf;
                
                if(qDes[2]<=k_p_rest_calf)
                {
                    Torque_ff(FL_2,0) = k_spring_calf * (qDes[2] - (k_p_rest_calf));
                }
                else
                {
                    Torque_ff(FL_2,0) = 0;
                }
                
                if(FF_enable)
                {
                    torque(FL_2,0) += Torque_ff(FL_2,0);
                }

                if(torque(FR_0,0) > 3.0f) torque(FR_0,0) = 3.0f;
                if(torque(FR_0,0) < -3.0f) torque(FR_0,0) = -3.0f;

                if(torque(FR_1,0) > 10.0f) torque(FR_1,0) = 10.0f;
                if(torque(FR_1,0) < -10.0f) torque(FR_1,0) = -10.0f;

                if(torque(FR_2,0) > 3.0f) torque(FR_2,0) = 3.0f;
                if(torque(FR_2,0) < -3.0f) torque(FR_2,0) = -3.0f;

                if(torque(FL_2,0) > 3.0f) torque(FL_2,0) = 3.0f;
                if(torque(FL_2,0) < -3.0f) torque(FL_2,0) = -3.0f;



                SendLowROS.motorCmd[FR_1].q = PosStopF;
                SendLowROS.motorCmd[FR_1].dq = VelStopF;
                SendLowROS.motorCmd[FR_1].Kp = 0;
                SendLowROS.motorCmd[FR_1].Kd = 0;
                SendLowROS.motorCmd[FR_1].tau = torque(FR_1,0);

                // SendLowROS.motorCmd[FR_2].q = PosStopF;
                // SendLowROS.motorCmd[FR_2].dq = VelStopF;
                // SendLowROS.motorCmd[FR_2].Kp = 0;
                // SendLowROS.motorCmd[FR_2].Kd = 0;
                // SendLowROS.motorCmd[FR_2].tau = torque(FR_2,0);

                // SendLowROS.motorCmd[FL_2].q = PosStopF;
                // SendLowROS.motorCmd[FL_2].dq = VelStopF;
                // SendLowROS.motorCmd[FL_2].Kp = 0;
                // SendLowROS.motorCmd[FL_2].Kd = 0;
                // SendLowROS.motorCmd[FL_2].tau = torque(FL_2,0);


            }

        }


        ///********************* data saving ************************************///////
        joint2simulation.header.stamp = ros::Time::now();
        joint2simulationx.header.stamp = ros::Time::now();
        ////
        for(int j=0; j<12; j++){
                joint2simulation.position[j] = SendLowROS.motorCmd[j].q; // desired joint angles; 
        }
        for(int j=0; j<12; j++)
        {
            joint2simulation.position[12+j] = RecvLowROS.motorState[j].q;   // measured joint angles;
        } 


        for(int j=0; j<3; j++)
        {
            joint2simulation.position[24+j] = FR_foot_des(j,0);   // desired FR Leg position;
        }         
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[27+j] = FL_foot_des(j,0);   // desired position;
        }
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[30+j] = RR_foot_des(j,0);   // desired position;
        }
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[33+j] = RL_foot_des(j,0);   // desired position;
        } 

        for(int j=0; j<3; j++)
        {
            joint2simulation.position[36+j] = FR_foot_relative_mea(j,0) + body_p_des(j,0);   // measured position;
        }         
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[39+j] = FL_foot_relative_mea(j,0) + body_p_des(j,0);   // measured position;
        }
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[42+j] = RR_foot_relative_mea(j,0) + body_p_des(j,0);   // measured position;
        }
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[45+j] = RL_foot_relative_mea(j,0) + body_p_des(j,0);   // measured position;
        } 
        
        /// body_pos_des
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[48+j] = body_p_des(j,0);   // desired body position;
        }
        /// body_R_des
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[51+j] = body_r_des(j,0);   // desired body ori;
        }

        /// body_R_des
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[51+j] = body_r_des(j,0);   // desired body ori;
        }  


        // joint torque desired 
        for(int j=0; j<12; j++)
        {
            joint2simulation.position[78+j] = torque(j,0);
        }

        //// torque measured
        for(int j=0; j<12; j++)
        {
            joint2simulationx.position[j] = RecvLowROS.motorState[j].tauEst;
        }
        
        /////// measured current
        for(int j=0; j<12; j++)
        {
            joint2simulationx.position[12+j] = Torque_ff(j,0);
        }        

        gait_data_pub.publish(joint2simulation);
        gait_data_pubx.publish(joint2simulationx);

        // /////sending command ////////////
        // SendLowLCM = ToLcm(SendLowROS, SendLowLCM);
        // roslcm.Send(SendLowLCM);

        ros::spinOnce();
        loop_rate.sleep();

        count++;
        if(count > 500){
            count = 500;
            initiated_flag = true;
        }
        n_count++;  

    }
    return 0;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "torque_ros_mode");

    UNITREE_LEGGED_SDK::LCM roslcm(LOWLEVEL);
    mainHelper<UNITREE_LEGGED_SDK::LowCmd, UNITREE_LEGGED_SDK::LowState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
}