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
#include "servo.h"
#include <Eigen/Dense>
#include "kinematics/Kinematics.h"
#include "sensor_msgs/JointState.h"
#include "Robotpara/robot_const_para_config.h"

using namespace std;
using namespace unitree_model;

bool start_up = true;

#define PI 3.1415926

sensor_msgs::JointState joint2simulation;



class multiThread
{
public:
    multiThread(string rname){
        robot_name = rname;
        imu_sub = nm.subscribe("/trunk_imu", 1, &multiThread::imuCallback, this);
        footForce_sub[0] = nm.subscribe("/visual/FR_foot_contact/the_force", 1, &multiThread::FRfootCallback, this);
        footForce_sub[1] = nm.subscribe("/visual/FL_foot_contact/the_force", 1, &multiThread::FLfootCallback, this);
        footForce_sub[2] = nm.subscribe("/visual/RR_foot_contact/the_force", 1, &multiThread::RRfootCallback, this);
        footForce_sub[3] = nm.subscribe("/visual/RL_foot_contact/the_force", 1, &multiThread::RLfootCallback, this);
        servo_sub[0] = nm.subscribe("/" + robot_name + "_gazebo/FR_hip_controller/state", 1, &multiThread::FRhipCallback, this);
        servo_sub[1] = nm.subscribe("/" + robot_name + "_gazebo/FR_thigh_controller/state", 1, &multiThread::FRthighCallback, this);
        servo_sub[2] = nm.subscribe("/" + robot_name + "_gazebo/FR_calf_controller/state", 1, &multiThread::FRcalfCallback, this);
        servo_sub[3] = nm.subscribe("/" + robot_name + "_gazebo/FL_hip_controller/state", 1, &multiThread::FLhipCallback, this);
        servo_sub[4] = nm.subscribe("/" + robot_name + "_gazebo/FL_thigh_controller/state", 1, &multiThread::FLthighCallback, this);
        servo_sub[5] = nm.subscribe("/" + robot_name + "_gazebo/FL_calf_controller/state", 1, &multiThread::FLcalfCallback, this);
        servo_sub[6] = nm.subscribe("/" + robot_name + "_gazebo/RR_hip_controller/state", 1, &multiThread::RRhipCallback, this);
        servo_sub[7] = nm.subscribe("/" + robot_name + "_gazebo/RR_thigh_controller/state", 1, &multiThread::RRthighCallback, this);
        servo_sub[8] = nm.subscribe("/" + robot_name + "_gazebo/RR_calf_controller/state", 1, &multiThread::RRcalfCallback, this);
        servo_sub[9] = nm.subscribe("/" + robot_name + "_gazebo/RL_hip_controller/state", 1, &multiThread::RLhipCallback, this);
        servo_sub[10] = nm.subscribe("/" + robot_name + "_gazebo/RL_thigh_controller/state", 1, &multiThread::RLthighCallback, this);
        servo_sub[11] = nm.subscribe("/" + robot_name + "_gazebo/RL_calf_controller/state", 1, &multiThread::RLcalfCallback, this);
    
        //// topic with first-layer MPC
        nrt_mpc_gait_subscribe_ = nm.subscribe("/MPC/Gait", 10, &multiThread::nrt_gait_sub_operation, this);
        gait_des_sub_ = nm.subscribe("/rtMPC/traj", 10,&multiThread::rt_gait_sub_operation, this);
    }

    void imuCallback(const sensor_msgs::Imu & msg)
    { 
        lowState.imu.quaternion[0] = msg.orientation.w;
        lowState.imu.quaternion[1] = msg.orientation.x;
        lowState.imu.quaternion[2] = msg.orientation.y;
        lowState.imu.quaternion[3] = msg.orientation.z;

        lowState.imu.gyroscope[0] = msg.angular_velocity.x;
        lowState.imu.gyroscope[1] = msg.angular_velocity.y;
        lowState.imu.gyroscope[2] = msg.angular_velocity.z;
        
        lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
        lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
        lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
        
    }

    void FRhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[0].mode = msg.mode;
        lowState.motorState[0].q = msg.q;
        lowState.motorState[0].dq = msg.dq;
        lowState.motorState[0].tauEst = msg.tauEst;
        
    }

    void FRthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[1].mode = msg.mode;
        lowState.motorState[1].q = msg.q;
        lowState.motorState[1].dq = msg.dq;
        lowState.motorState[1].tauEst = msg.tauEst;

    }

    void FRcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[2].mode = msg.mode;
        lowState.motorState[2].q = msg.q;
        lowState.motorState[2].dq = msg.dq;
        lowState.motorState[2].tauEst = msg.tauEst;

    }

    void FLhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[3].mode = msg.mode;
        lowState.motorState[3].q = msg.q;
        lowState.motorState[3].dq = msg.dq;
        lowState.motorState[3].tauEst = msg.tauEst;

    }

    void FLthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[4].mode = msg.mode;
        lowState.motorState[4].q = msg.q;
        lowState.motorState[4].dq = msg.dq;
        lowState.motorState[4].tauEst = msg.tauEst;

    }

    void FLcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[5].mode = msg.mode;
        lowState.motorState[5].q = msg.q;
        lowState.motorState[5].dq = msg.dq;
        lowState.motorState[5].tauEst = msg.tauEst;

    }

    void RRhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[6].mode = msg.mode;
        lowState.motorState[6].q = msg.q;
        lowState.motorState[6].dq = msg.dq;
        lowState.motorState[6].tauEst = msg.tauEst;

    }

    void RRthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[7].mode = msg.mode;
        lowState.motorState[7].q = msg.q;
        lowState.motorState[7].dq = msg.dq;
        lowState.motorState[7].tauEst = msg.tauEst;

    }

    void RRcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[8].mode = msg.mode;
        lowState.motorState[8].q = msg.q;
        lowState.motorState[8].dq = msg.dq;
        lowState.motorState[8].tauEst = msg.tauEst;

    }

    void RLhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[9].mode = msg.mode;
        lowState.motorState[9].q = msg.q;
        lowState.motorState[9].dq = msg.dq;
        lowState.motorState[9].tauEst = msg.tauEst;
    }

    void RLthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[10].mode = msg.mode;
        lowState.motorState[10].q = msg.q;
        lowState.motorState[10].dq = msg.dq;
        lowState.motorState[10].tauEst = msg.tauEst;
    }

    void RLcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[11].mode = msg.mode;
        lowState.motorState[11].q = msg.q;
        lowState.motorState[11].dq = msg.dq;
        lowState.motorState[11].tauEst = msg.tauEst;
    }

    void FRfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[0].x = msg.wrench.force.x;
        lowState.eeForce[0].y = msg.wrench.force.y;
        lowState.eeForce[0].z = msg.wrench.force.z;
        lowState.footForce[0] = msg.wrench.force.z;
    }

    void FLfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[1].x = msg.wrench.force.x;
        lowState.eeForce[1].y = msg.wrench.force.y;
        lowState.eeForce[1].z = msg.wrench.force.z;
        lowState.footForce[1] = msg.wrench.force.z;
    }

    void RRfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[2].x = msg.wrench.force.x;
        lowState.eeForce[2].y = msg.wrench.force.y;
        lowState.eeForce[2].z = msg.wrench.force.z;
        lowState.footForce[2] = msg.wrench.force.z;
    }

    void RLfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[3].x = msg.wrench.force.x;
        lowState.eeForce[3].y = msg.wrench.force.y;
        lowState.eeForce[3].z = msg.wrench.force.z;
        lowState.footForce[3] = msg.wrench.force.z;
    }


    //// ====================================== real-time mpc control //////////
    void nrt_gait_sub_operation(const sensor_msgs::JointState::ConstPtr &msg)
    {
	for (int jx = 0; jx<100; jx++)
	{
	    slow_mpc_gait(jx) = msg->position[jx]; 
	}
	    mpc_gait_flag = slow_mpc_gait(99);
    }
    
    
    void rt_gait_sub_operation(const sensor_msgs::JointState::ConstPtr &msg)
    {
	for (int jx = 0; jx<51; jx++)
	{
	    fast_mpc_gait(jx) = msg->position[36+jx]; 
	}
	fast_mpc_gait_flag = msg->position[99];
	count_in_mpc_max = msg->position[98];
	
    }    
    
  
  


private:
    ros::NodeHandle nm;
    ros::Subscriber servo_sub[12], footForce_sub[4], imu_sub, nrt_mpc_gait_subscribe_, gait_des_sub_;


    string robot_name;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "unitree_gazebo_servo");

    string robot_name;
    ros::param::get("/robot_name", robot_name);
    string control_mode;
    ros::param::get("/rctr", control_mode);
    cout << "robot_name: " << robot_name << endl;
    cout << "control_model: " << control_mode << endl;
    

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




  
    multiThread listen_publish_obj(robot_name);
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    usleep(300000); // must wait 300ms, to get first state

    ros::NodeHandle n;
    ros::Publisher lowState_pub; //for rviz visualization
    ros::Publisher gait_data_pub; // for data_analysis
    ros::Publisher gait_data_pubx;
    ros::Publisher control_to_rtmpc_pub_; /// state feedback to rtmpc
    
    rt_frequency = 1000; /// frequency of lower_level control
    time_programming = 1.0/rt_frequency;
    ros::Rate loop_rate(rt_frequency);

    joint2simulation.position.resize(100);
    n_count = 0;
    stand_duration = 5; /// stand up: 2s
    
    

    cout << "initial"<<endl;
   

    // the following nodes have been initialized by "gazebo.launch"
    lowState_pub = n.advertise<unitree_legged_msgs::LowState>("/" + robot_name + "_gazebo/lowState/state", 1);
    servo_pub[0] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_calf_controller/command", 1);
    gait_data_pub = n.advertise<sensor_msgs::JointState>("go1_gait_data",10);
    gait_data_pubx = n.advertise<sensor_msgs::JointState>("go1_gait_datax",10);
    control_to_rtmpc_pub_ = n.advertise<sensor_msgs::JointState>("/control2rtmpc/state", 10);
    
    
    
    ///// a must : parameter initailization for robot control
    paramInit();


    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////*************************************************** main control loop ********************************////
    while (ros::ok()){

        count_in_rt_ros += 1;
        // publisher measured state:
        lowState_pub.publish(lowState);/// for data analysis
        
        
        bjx1 = slow_mpc_gait(27);
        
        ////////////////////////// kinematic-based state estimation ///////////////////////////////////////////////
        // Forward kinematics: for kinematic-based state estimation //////////////////////////////////////////////
        FR_angle_mea(0,0) = lowState.motorState[0].q;
        FR_angle_mea(1,0) = lowState.motorState[1].q;
        FR_angle_mea(2,0) = lowState.motorState[2].q;
        FL_angle_mea(0,0) = lowState.motorState[3].q;
        FL_angle_mea(1,0) = lowState.motorState[4].q;
        FL_angle_mea(2,0) = lowState.motorState[5].q;
        RR_angle_mea(0,0) = lowState.motorState[6].q;
        RR_angle_mea(1,0) = lowState.motorState[7].q;
        RR_angle_mea(2,0) = lowState.motorState[8].q;
        RL_angle_mea(0,0) = lowState.motorState[9].q;
        RL_angle_mea(1,0) = lowState.motorState[10].q;
        RL_angle_mea(2,0) = lowState.motorState[11].q;

        FR_dq_mea(0,0) = lowState.motorState[0].dq;
        FR_dq_mea(1,0) = lowState.motorState[1].dq;
        FR_dq_mea(2,0) = lowState.motorState[2].dq;
        FL_dq_mea(0,0) = lowState.motorState[3].dq;
        FL_dq_mea(1,0) = lowState.motorState[4].dq;
        FL_dq_mea(2,0) = lowState.motorState[5].dq;
        RR_dq_mea(0,0) = lowState.motorState[6].dq;
        RR_dq_mea(1,0) = lowState.motorState[7].dq;
        RR_dq_mea(2,0) = lowState.motorState[8].dq;
        RL_dq_mea(0,0) = lowState.motorState[9].dq;
        RL_dq_mea(1,0) = lowState.motorState[10].dq;
        RL_dq_mea(2,0) = lowState.motorState[11].dq;


        //// IMU quaternion
        root_quat = Eigen::Quaterniond(lowState.imu.quaternion[0],
                                       lowState.imu.quaternion[1],
                                       lowState.imu.quaternion[2],
                                       lowState.imu.quaternion[3]);         

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

        
        for (int jx = 0; jx<25; jx++)
        {
          state_to_MPC.position[jx] = state_feedback(jx,0);
        }        
        control_to_rtmpc_pub_.publish(state_to_MPC); //// state feedback to MPC; 1+5*body p,v,a+3*3*foot position;
        
        
        //////////////////// gait control loop/////////////////////////////////////////////////////
        /// *****************joint cmd generation*******************////
        double targetPos[12]  = {0, 0.87, -1.5, 0, 0.87, -1.5, 0, 0.87, -1.5, 0, 0.87, -1.5};  
        // reference angle generation: a simple test
        if (n_count*time_programming <= stand_duration) 
        {
            //****************Homing_pose*******************
            for(int j=0; j<12; j++) lastPos[j] = lowState.motorState[j].q;

            percent = pow((n_count*time_programming)/stand_duration,2);
            for(int j=0; j<12; j++){
                lowCmd.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent; 
            }

            FR_angle_des(0,0) = lowCmd.motorCmd[0].q;
            FR_angle_des(1,0) = lowCmd.motorCmd[1].q;
            FR_angle_des(2,0) = lowCmd.motorCmd[2].q;
            FL_angle_des(0,0) = lowCmd.motorCmd[3].q;
            FL_angle_des(1,0) = lowCmd.motorCmd[4].q;
            FL_angle_des(2,0) = lowCmd.motorCmd[5].q;
            RR_angle_des(0,0) = lowCmd.motorCmd[6].q;
            RR_angle_des(1,0) = lowCmd.motorCmd[7].q;
            RR_angle_des(2,0) = lowCmd.motorCmd[8].q;
            RL_angle_des(0,0) = lowCmd.motorCmd[9].q;
            RL_angle_des(1,0) = lowCmd.motorCmd[10].q;
            RL_angle_des(2,0) = lowCmd.motorCmd[11].q;   

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
        else   
        {
             ///****************squat_down: only for test*******************
/*            percent = 0.2 * (cos(n_count*time_programming - stand_duration) + 4);
            for(int j=0; j<12; j++){
                lowCmd.motorCmd[j].q = lastPos[j]*percent; 
            }

            // Forward kinematics_test:
            FR_angle_des(0,0) = lowCmd.motorCmd[0].q;
            FR_angle_des(1,0) = lowCmd.motorCmd[1].q;
            FR_angle_des(2,0) = lowCmd.motorCmd[2].q;
            FL_angle_des(0,0) = lowCmd.motorCmd[3].q;
            FL_angle_des(1,0) = lowCmd.motorCmd[4].q;
            FL_angle_des(2,0) = lowCmd.motorCmd[5].q;
            RR_angle_des(0,0) = lowCmd.motorCmd[6].q;
            RR_angle_des(1,0) = lowCmd.motorCmd[7].q;
            RR_angle_des(2,0) = lowCmd.motorCmd[8].q;
            RL_angle_des(0,0) = lowCmd.motorCmd[9].q;
            RL_angle_des(1,0) = lowCmd.motorCmd[10].q;
            RL_angle_des(2,0) = lowCmd.motorCmd[11].q;

            FR_foot_relative_des = Kine.Forward_kinematics(FR_angle_des, 0);
            FL_foot_relative_des = Kine.Forward_kinematics(FL_angle_des, 1);
            RR_foot_relative_des = Kine.Forward_kinematics(RR_angle_des, 2);
            RL_foot_relative_des = Kine.Forward_kinematics(RL_angle_des, 3); */

            ///// locally inverse kinematics: local kinematics: using the reference trajectory generated by the FK;
/*             Eigen::Matrix<double,3,1>  q_ini;
            q_ini(0,0) = lastPos[0];
            q_ini(1,0) = lastPos[1];
            q_ini(2,0) = lastPos[2];
            FR_angle_des = Kine.Inverse_kinematics(FR_foot_relative_des,q_ini,0);

            q_ini(0,0) = lastPos[3];
            q_ini(1,0) = lastPos[4];
            q_ini(2,0) = lastPos[5];
            FL_angle_des = Kine.Inverse_kinematics(FL_foot_relative_des,q_ini,1);

            q_ini(0,0) = lastPos[6];
            q_ini(1,0) = lastPos[7];
            q_ini(2,0) = lastPos[8]; 
            RR_angle_des = Kine.Inverse_kinematics(RR_foot_relative_des,q_ini,2);

            q_ini(0,0) = lastPos[9];
            q_ini(1,0) = lastPos[10];
            q_ini(2,0) = lastPos[11];
            RL_angle_des = Kine.Inverse_kinematics(RL_foot_relative_des,q_ini,3); */

            /// hand tuned gait: global inverse kinematics test://///////////////////
            // ******************** body movement test *********************////
            /* body_p_des(0,0) = body_p_Homing(0,0) + (0.01 * (sin(n_count*time_programming - stand_duration)));
            body_p_des(1,0) = (0.1 * (cos(n_count*time_programming - stand_duration + PI/2)));
            body_r_des(0,0) = (0.15 * (sin(n_count*time_programming - stand_duration)));
            body_r_des(1,0) = (0.1 * (sin(n_count*time_programming - stand_duration)));  */
            

            // //  // ******************** troting: just a quick test *********************////
            // int nT = 400;   //// 0.001*500=0.5s;
            // int n_count_y;
            // n_count_y = n_count - (int)stand_duration/time_programming;

            // if (( n_count_y % nT) == nT/2)
            // {
            //     cout<< "body_p_des_hight:"<<body_p_Homing(2)<<endl;
            //     cout<< "leg_distance_x:"<< FR_foot_relative_des(0,0) - RR_foot_relative_des(0,0)<<endl;
            //     cout<< "leg_distance_y:"<< RL_foot_relative_des(1,0) - RR_foot_relative_des(1,0)<<endl;
            // }
            
            // if ( ( n_count_y % nT) <= nT/2 ) // first half period
            // {
            //     FR_foot_des(2,0) = RL_foot_des(2,0) = 0.04 * sin((n_count_y % nT) / ((nT/2.0))  * PI);
            // }
            // else             ///later half period
            // {
            //     FL_foot_des(2,0) = RR_foot_des(2,0) = 0.04 * sin(((n_count_y % nT) - (nT/2)) / ((nT/2.0))  * PI);
            // }
            
            
            ///////////////////////////////////// MPC gait planner //////////////////////////
            /////// rt loop counter 
            count_in_rt_loop += 1;  
            t_int = ((int) round(count_in_rt_loop/n_t_int));
            state_feedback(0,0) = t_int;

            
            //// non -real-time intepoloation: data filter: from hierachical convex optimization:

            com_des[0] = butterworthLPF1.filter(slow_mpc_gait(0,0));
            com_des[1] = butterworthLPF2.filter(slow_mpc_gait(1,0));
            com_des[2] = butterworthLPF3.filter(slow_mpc_gait(2,0));
            
            theta_des[0] = 0;
            theta_des[1] = 0;	    
            theta_des[2] = 0;            
            // theta_des[0] = butterworthLPF4.filter(slow_mpc_gait(36));
            // theta_des[1] = butterworthLPF5.filter(slow_mpc_gait(37));	    
            // theta_des[2] = 0;
               
            rfoot_des[0] = butterworthLPF6.filter(slow_mpc_gait(9));
            rfoot_des[1] = butterworthLPF7.filter(slow_mpc_gait(10));
            rfoot_des[2] = butterworthLPF8.filter(slow_mpc_gait(11));

            lfoot_des[0] = butterworthLPF9.filter(slow_mpc_gait(6));
            lfoot_des[1] = butterworthLPF10.filter(slow_mpc_gait(7));
            lfoot_des[2] = butterworthLPF11.filter(slow_mpc_gait(8));

            if (count_in_rt_loop>0)
            {
               comv_des[0] = (com_des[0] - com_des_pre[0]) /dtx;
               comv_des[1] = (com_des[1] - com_des_pre[1]) /dtx;
               comv_des[2] = (com_des[2] - com_des_pre[2]) /dtx;
            }


            coma_des[0] = butterworthLPF12.filter(slow_mpc_gait(39));
            coma_des[1] = butterworthLPF13.filter(slow_mpc_gait(40));
            coma_des[2] = butterworthLPF14.filter(slow_mpc_gait(41));		


            if (count_in_rt_loop * gait::t_program_cyclic >= 0.6)
            {
                switch (gait_mode)
                {
                case 101:  ////biped walking
                    body_p_des[0] = com_des[0];
                    body_p_des[1] = com_des[1] * y_offset;
                    body_p_des[2] = com_des[2];

                    body_r_des[0] = theta_des[0];
                    body_r_des[1] = theta_des[1];
                    body_r_des[2] = theta_des[2];

                    //// right two legs move synchronous
                    FR_foot_des[0] = FR_foot_Homing[0] + rfoot_des[0];
                    FR_foot_des[1] = FR_foot_Homing[1] + rfoot_des[1] + gait::RobotParaClass_HALF_HIP_WIDTH;
                    FR_foot_des[2] = FR_foot_Homing[2] + rfoot_des[2];
                    RR_foot_des[0] = RR_foot_Homing[0] + rfoot_des[0];
                    RR_foot_des[1] = RR_foot_Homing[1] + rfoot_des[1] + gait::RobotParaClass_HALF_HIP_WIDTH;
                    RR_foot_des[2] = RR_foot_Homing[2] + rfoot_des[2];

                    //// left two legs move synchronous
                    FL_foot_des[0] = FL_foot_Homing[0] + lfoot_des[0];
                    FL_foot_des[1] = FL_foot_Homing[1] + lfoot_des[1] - gait::RobotParaClass_HALF_HIP_WIDTH;
                    FL_foot_des[2] = FL_foot_Homing[2] + lfoot_des[2];
                    RL_foot_des[0] = RL_foot_Homing[0] + lfoot_des[0];
                    RL_foot_des[1] = RL_foot_Homing[1] + lfoot_des[1] - gait::RobotParaClass_HALF_HIP_WIDTH;
                    RL_foot_des[2] = RL_foot_Homing[2] + lfoot_des[2];


                    break;
                case 102:  ///troting
                    body_p_des[0] = com_des[0];
                    body_p_des[1] = com_des[1] * y_offset;
                    body_p_des[2] = com_des[2];

                    body_r_des[0] = theta_des[0];
                    body_r_des[1] = theta_des[1];
                    body_r_des[2] = theta_des[2];

                    //// FR, RL two legs move synchronous
                    FR_foot_des[0] = FR_foot_Homing[0] + rfoot_des[0];
                    FR_foot_des[1] = FR_foot_Homing[1] + rfoot_des[1] + gait::RobotParaClass_HALF_HIP_WIDTH;
                    FR_foot_des[2] = FR_foot_Homing[2] + rfoot_des[2];
                    RL_foot_des[0] = RL_foot_Homing[0] + rfoot_des[0];
                    RL_foot_des[1] = RL_foot_Homing[1] + rfoot_des[1] + gait::RobotParaClass_HALF_HIP_WIDTH;
                    RL_foot_des[2] = RL_foot_Homing[2] + rfoot_des[2];

                    //// FL, RR two legs move synchronous
                    FL_foot_des[0] = FL_foot_Homing[0] + lfoot_des[0];
                    FL_foot_des[1] = FL_foot_Homing[1] + lfoot_des[1] - gait::RobotParaClass_HALF_HIP_WIDTH;
                    FL_foot_des[2] = FL_foot_Homing[2] + lfoot_des[2];
                    RR_foot_des[0] = RR_foot_Homing[0] + lfoot_des[0];
                    RR_foot_des[1] = RR_foot_Homing[1] + lfoot_des[1] - gait::RobotParaClass_HALF_HIP_WIDTH;
                    RR_foot_des[2] = RR_foot_Homing[2] + lfoot_des[2];                


                    break;
                case 103:  ///gallop: alter the  x-y direction
                    body_p_des[0] = com_des[0];
                    body_p_des[1] = com_des[1] * y_offset;
                    body_p_des[2] = com_des[2];
                    
                    // pitch angle generation //////
                    
                    pitch_angle_W = 2 * gait::pi / (2 * gait::t_period);

                    body_r_des[0] = theta_des[0];
                    ///// 
                    if (count_in_rt_loop - 3* n_period <= 0)
                    {
                        body_r_des[0] = theta_des[1];
                    }
                    else
                    {
                      body_r_des[1] = -(0.1 * (sin(pitch_angle_W * (count_in_rt_loop - 3* n_period) * gait::t_program_cyclic))); /// pitch angle//
                    }
                    
                    body_r_des[2] = theta_des[2];                    

                    //// fore two legs move synchronous
                    FR_foot_des[0] = FR_foot_Homing[0] + rfoot_des[0];
                    FR_foot_des[1] = FR_foot_Homing[1] + rfoot_des[1] + gait::RobotParaClass_HALF_HIP_WIDTH;
                    FR_foot_des[2] = FR_foot_Homing[2] + rfoot_des[2];
                    FL_foot_des[0] = FL_foot_Homing[0] + rfoot_des[0];
                    FL_foot_des[1] = FL_foot_Homing[1] + rfoot_des[1] + gait::RobotParaClass_HALF_HIP_WIDTH;
                    FL_foot_des[2] = FL_foot_Homing[2] + rfoot_des[2];

                    //// rear two legs move synchronous
                    RR_foot_des[0] = RR_foot_Homing[0] + lfoot_des[0];
                    RR_foot_des[1] = RR_foot_Homing[1] + lfoot_des[1] - gait::RobotParaClass_HALF_HIP_WIDTH;
                    RR_foot_des[2] = RR_foot_Homing[2] + lfoot_des[2];             
                    RL_foot_des[0] = RL_foot_Homing[0] + lfoot_des[0];
                    RL_foot_des[1] = RL_foot_Homing[1] + lfoot_des[1] - gait::RobotParaClass_HALF_HIP_WIDTH;
                    RL_foot_des[2] = RL_foot_Homing[2] + lfoot_des[2];  

                    break;            
                default:
                    break;
                }
            }

            ///cout << "xyyyy"<<endl;
            Eigen::Matrix<double,3,1>  q_ini;
            q_ini = FR_angle_des;
            FR_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,FR_foot_des,q_ini,0);
            FR_Jaco = Kine.Jacobian_kin;

            q_ini = FL_angle_des;
            FL_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,FL_foot_des,q_ini,1);
            FL_Jaco = Kine.Jacobian_kin;

            q_ini = RR_angle_des;
            RR_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,RR_foot_des,q_ini,2);
            RR_Jaco = Kine.Jacobian_kin;

            q_ini = RL_angle_des;
            RL_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,RL_foot_des,q_ini,3);  
            RL_Jaco = Kine.Jacobian_kin;

            ///// Force distribute/// Grf generation         
            Eigen::Matrix<double, 12,1> leg_position;
            leg_position.block<3,1>(0,0) = FR_foot_des;
            leg_position.block<3,1>(3,0) = FL_foot_des; 
            leg_position.block<3,1>(6,0) = RR_foot_des;
            leg_position.block<3,1>(9,0) = RL_foot_des; 

            /////////////////////////////// desired foot position and foot velocity: relative to  body framework
            FR_foot_relative_des = (FR_foot_des - body_p_des);
            FL_foot_relative_des = (FL_foot_des - body_p_des);
            RR_foot_relative_des = (RR_foot_des - body_p_des);
            RL_foot_relative_des = (RL_foot_des - body_p_des);            
            if (count_in_rt_loop>0)
            {
               FR_v_relative = (FR_foot_relative_des - FR_foot_relative_des_old) /dtx;
               FL_v_relative = (FL_foot_relative_des - FL_foot_relative_des_old) /dtx;
               RR_v_relative = (RR_foot_relative_des - RR_foot_relative_des_old) /dtx;
               RL_v_relative = (RL_foot_relative_des - RL_foot_relative_des_old) /dtx;
            }

            double nt_slow_mpc;
            nt_slow_mpc = (( count_in_rt_loop % n_t_int )* dtx) / (gait::dt_mpc_slow);
            

            //// prediction force
            // F_sum(0) = gait::mass * slow_mpc_gait(80);
            // F_sum(1) = gait::mass * slow_mpc_gait(81);
            // F_sum(2) = gait::mass * gait::_g - gait::mass * slow_mpc_gait(82);
            
            F_sum(0) = gait::mass * coma_des[0];
            F_sum(1) = gait::mass * coma_des[1];
            F_sum(2) = gait::mass * gait::_g - gait::mass * coma_des[2];


            double com_rleg_dis = sqrt(pow(com_des[0]-rfoot_des[0], 2) + pow(com_des[1]-rfoot_des[1], 2) + pow(com_des[2]-rfoot_des[2], 2));
            double com_lleg_dis = sqrt(pow(com_des[0]-lfoot_des[0], 2) + pow(com_des[1]-lfoot_des[1], 2) + pow(com_des[2]-lfoot_des[2], 2));
            rleg_com = com_rleg_dis/(com_rleg_dis + com_lleg_dis);
            lleg_com = 1 - rleg_com;


            
            if(bjx1 >= 2)
            {
                if(bjx1 % 2 ==0) ////left support
                {
                    F_lr_predict(0) = F_sum(0);
                    F_lr_predict(1) = F_sum(1);
                    F_lr_predict(2) = F_sum(2);
                    F_lr_predict(3) = 0;
                    F_lr_predict(4) = 0;
                    F_lr_predict(5) = 0;


                    switch (gait_mode)
                        {
                        case 101:  ////biped walking
                            FR_swing = true;
                            RR_swing = true;
                            FR_swing = false;
                            RR_swing = false;
                            break;
                        case 102:  ///troting
                            FR_swing = true;
                            RL_swing = true;
                            FL_swing = false;
                            RR_swing = false;

                            break;
                        case 103:  ///gallop: alter the  x-y direction
                            FR_swing = true;
                            FL_swing = true;
                            RR_swing = false;
                            RL_swing = false;                    

                            break;            
                        default:
                            break;
                        } 
                }
                else
                {
                    F_lr_predict(0) = 0;
                    F_lr_predict(1) = 0;
                    F_lr_predict(2) = 0;
                    F_lr_predict(3) = F_sum(0);
                    F_lr_predict(4) = F_sum(1);
                    F_lr_predict(5) = F_sum(2);
                    switch (gait_mode)
                        {
                        case 101:  ////biped walking
                            FR_swing = false;
                            RR_swing = false;
                            FR_swing = true;
                            RR_swing = true;
                            break;
                        case 102:  ///troting
                            FR_swing = false;
                            RL_swing = false;
                            FL_swing = true;
                            RR_swing = true;

                            break;
                        case 103:  ///gallop: alter the  x-y direction
                            FR_swing = false;
                            FL_swing = false;
                            RR_swing = true;
                            RL_swing = true;                    

                            break;            
                        default:
                            break;
                        }
                                  
                }
            }
            else
            {
                F_lr_predict(0) = F_sum(0) * rleg_com;
                F_lr_predict(3) = F_sum(0) - F_lr_predict(0);
                F_lr_predict(1) = F_sum(1) * rleg_com;
                F_lr_predict(4) = F_sum(1) - F_lr_predict(1);
                F_lr_predict(2) = F_sum(2) * rleg_com;
                F_lr_predict(5) = F_sum(2) - F_lr_predict(2);


            }



            ////force:  left-right-leg   
            for(int j=0; j<6; j++)
            {
                //Force_L_R(j)= slow_mpc_gait(15+j) + nt_slow_mpc * (F_lr_predict(j) - slow_mpc_gait(15+j));  //continuous force profile
                //Force_L_R(j)= slow_mpc_gait(15+j);  //discontinuous force profile
                Force_L_R(j) = F_lr_predict(j);
            }                                
            
            Dynam.force_distribution(body_p_des,leg_position, Force_L_R, gait_mode, y_offset);
            
            FR_torque = Dynam.compute_joint_torques(FR_Jaco,FR_swing,FR_foot_relative_des,FR_foot_relative_mea,
                                                    FR_v_relative,FR_v_est_relative,0);

            FL_torque = Dynam.compute_joint_torques(FL_Jaco,FL_swing,FL_foot_relative_des,FL_foot_relative_mea,
                                                    FL_v_relative,FL_v_est_relative,1);
            

            RR_torque = Dynam.compute_joint_torques(RR_Jaco,RR_swing,RR_foot_relative_des,RR_foot_relative_mea,
                                                    RR_v_relative,RR_v_est_relative,2);

            RL_torque = Dynam.compute_joint_torques(RL_Jaco,RL_swing,RL_foot_relative_des,RL_foot_relative_mea,
                                                    RL_v_relative,RL_v_est_relative,3);
            ///cout << "xxxxxxxxxxxxxxxxx"<<endl;                                        
            Legs_torque.block<3,1>(0,0) = FR_torque;
            Legs_torque.block<3,1>(3,0) = FL_torque;
            Legs_torque.block<3,1>(6,0) = RR_torque;
            Legs_torque.block<3,1>(9,0) = RL_torque; 
            
            // Legs_torque(0,0) = butterworthLPF15.filter(FR_torque(0,0));
            // Legs_torque(1,0) = butterworthLPF16.filter(FR_torque(1,0));
            // Legs_torque(2,0) = butterworthLPF17.filter(FR_torque(2,0));
            // Legs_torque(3,0) = butterworthLPF18.filter(FL_torque(0,0));
            // Legs_torque(4,0) = butterworthLPF19.filter(FL_torque(1,0));
            // Legs_torque(5,0) = butterworthLPF20.filter(FL_torque(2,0));
            // Legs_torque(6,0) = butterworthLPF21.filter(RR_torque(0,0));
            // Legs_torque(7,0) = butterworthLPF22.filter(RR_torque(1,0));
            // Legs_torque(8,0) = butterworthLPF23.filter(RR_torque(2,0));
            // Legs_torque(9,0) = butterworthLPF24.filter(RL_torque(0,0));
            // Legs_torque(10,0) = butterworthLPF25.filter(RL_torque(1,0));
            // Legs_torque(11,0) = butterworthLPF26.filter(RL_torque(2,0));



            if (control_mode == "position")
            {
                // desired angle: generatee by inverse kinematics
                lowCmd.motorCmd[0].q = FR_angle_des(0,0);
                lowCmd.motorCmd[1].q = FR_angle_des(1,0);
                lowCmd.motorCmd[2].q = FR_angle_des(2,0);
                lowCmd.motorCmd[3].q = FL_angle_des(0,0);
                lowCmd.motorCmd[4].q = FL_angle_des(1,0);
                lowCmd.motorCmd[5].q = FL_angle_des(2,0);
                lowCmd.motorCmd[6].q = RR_angle_des(0,0);
                lowCmd.motorCmd[7].q = RR_angle_des(1,0);
                lowCmd.motorCmd[8].q = RR_angle_des(2,0); 
                lowCmd.motorCmd[9].q = RL_angle_des(0,0);
                lowCmd.motorCmd[10].q = RL_angle_des(1,0);
                lowCmd.motorCmd[11].q = RL_angle_des(2,0); 
            }
            else
            {
                // desired angle: generatee by inverse kinematics
                lowCmd.motorCmd[0].q = FR_angle_des(0,0);
                lowCmd.motorCmd[1].q = FR_angle_des(1,0);
                lowCmd.motorCmd[2].q = FR_angle_des(2,0);
                lowCmd.motorCmd[3].q = FL_angle_des(0,0);
                lowCmd.motorCmd[4].q = FL_angle_des(1,0);
                lowCmd.motorCmd[5].q = FL_angle_des(2,0);
                lowCmd.motorCmd[6].q = RR_angle_des(0,0);
                lowCmd.motorCmd[7].q = RR_angle_des(1,0);
                lowCmd.motorCmd[8].q = RR_angle_des(2,0); 
                lowCmd.motorCmd[9].q = RL_angle_des(0,0);
                lowCmd.motorCmd[10].q = RL_angle_des(1,0);
                lowCmd.motorCmd[11].q = RL_angle_des(2,0);                 
            }
            // if (bjx1>=4)
            // {
            //     for (int i = 0; i < 12; i++) 
            //     {
            //         lowCmd.motorCmd[i].mode = 0x0A;
            //         lowCmd.motorCmd[i].q = PosStopF;
            //         lowCmd.motorCmd[i].dq = VelStopF;
            //         lowCmd.motorCmd[i].Kp = 0;
            //         lowCmd.motorCmd[i].Kd = 0;
            //         lowCmd.motorCmd[i].tau = Legs_torque(i);
            //     }
            // }
        }

        ////////////////////////////////////////////////////////////////////////
        // joint command pub to servors:
        for(int m=0; m<12; m++){
            servo_pub[m].publish(lowCmd.motorCmd[m]);
        } 


        FR_foot_relative_des_old = FR_foot_relative_des;
        FL_foot_relative_des_old = FL_foot_relative_des;
        RR_foot_relative_des_old = RR_foot_relative_des;
        RL_foot_relative_des_old = RL_foot_relative_des;

        FR_foot_relative_mea_old = FR_foot_relative_mea;
        FL_foot_relative_mea_old = FL_foot_relative_mea;
        RR_foot_relative_mea_old = RR_foot_relative_mea;
        RL_foot_relative_mea_old = RL_foot_relative_mea;

        com_des_pre[0] = com_des[0];
        com_des_pre[1] = com_des[1];
        com_des_pre[2] = com_des[2];

         ///********************* data saving ************************************///////
        joint2simulation.header.stamp = ros::Time::now();
        joint2simulationx.header.stamp = ros::Time::now();
        for(int j=0; j<12; j++){
                joint2simulation.position[j] = lowCmd.motorCmd[j].q; // desired joint angles; 
        }
        for(int j=0; j<12; j++)
        {
            joint2simulation.position[12+j] = lowState.motorState[j].q;   // measured joint angles;
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

        /// force and torque distribution//// calculated by qp or MPC, this work proposes close-form solution
        // for(int j=0; j<12; j++)
        // {
        //     joint2simulation.position[54+j] = slow_mpc_gait(15+j);  // desired force and momentum;
        // } 
        for(int j=0; j<6; j++)
        {
            joint2simulation.position[54+j] = Force_L_R(j);  // desired force;
        }


        // FR force
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[66+j] = Dynam.F_leg_ref(j,0); 
        }

        // FL force
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[69+j] = Dynam.F_leg_ref(j,1);
        } 

        // RR force
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[72+j] = Dynam.F_leg_ref(j,2); 
        }  

        // RL force
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[75+j] = Dynam.F_leg_ref(j,3);
        }                                

        // joint torque desired 
        for(int j=0; j<12; j++)
        {
            joint2simulation.position[78+j] = Legs_torque(j);
        }
        
        //// 
        for(int j=0; j<12; j++)
        {
            joint2simulationx.position[j] = lowState.motorState[j].tauEst;
        }        


        gait_data_pub.publish(joint2simulation);
        gait_data_pubx.publish(joint2simulationx);
        
        
        ros::spinOnce();

        loop_rate.sleep();
        n_count++;   
        
    }
    
    return 0;
}
