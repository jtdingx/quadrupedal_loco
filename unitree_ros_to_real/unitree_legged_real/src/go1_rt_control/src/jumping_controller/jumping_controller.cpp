#include "go1_const.h"
#include "Robotpara/robot_const_para_config.h"
#include "geometry_msgs/Twist.h"
#include "jumping_controller.h"
#include "cnpy.h"
//###############################

void get_data(Eigen::MatrixXd& mat, const std::string& file_name) {
    cnpy::NpyArray history = cnpy::npy_load(file_name);
    double* ptr = history.data<double>();
    int data_row = history.shape[0];
    int data_col = history.shape[1];
    mat = Eigen::MatrixXd::Map(ptr, data_row, data_col);
}


double jointLinearInterpolation(double initPos, double targetPos, double rate, int j)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}


Quadruped::Quadruped(){
    bool on_ground = false;
    PIDgains(on_ground);
    zeroStates();
}

void Quadruped::upload_history(const std::string& history_file, int& n_cols) {
    get_data(history, history_file);
    n_cols = history.cols();  // history length
}

void Quadruped::zeroStates(){
    FR_angle_mea.setZero();
    FL_angle_mea.setZero();
    RR_angle_mea.setZero();
    RL_angle_mea.setZero();

    FR_dq_mea.setZero();
    FL_dq_mea.setZero();
    RR_dq_mea.setZero();
    RL_dq_mea.setZero();

    base_pos.setZero();
    base_rpy.setZero();

};

void Quadruped::updateStates(unitree_legged_msgs::LowState RecvLowROS){
    //"""This function assigns the received robot states to the respective variables."""
    FR_angle_mea(0,0) = RecvLowROS.motorState[0].q;
    FR_angle_mea(1,0) = RecvLowROS.motorState[1].q;
    FR_angle_mea(2,0) = RecvLowROS.motorState[2].q;
    FL_angle_mea(0,0) = RecvLowROS.motorState[3].q;
    FL_angle_mea(1,0) = RecvLowROS.motorState[4].q;
    FL_angle_mea(2,0) = RecvLowROS.motorState[5].q;
    RR_angle_mea(0,0) = RecvLowROS.motorState[6].q;
    RR_angle_mea(1,0) = RecvLowROS.motorState[7].q;
    RR_angle_mea(2,0) = RecvLowROS.motorState[8].q;
    RL_angle_mea(0,0) = RecvLowROS.motorState[9].q;
    RL_angle_mea(1,0) = RecvLowROS.motorState[10].q;
    RL_angle_mea(2,0) = RecvLowROS.motorState[11].q;

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


    Eigen::Quaterniond root_quat = Eigen::Quaterniond(RecvLowROS.imu.quaternion[0],
                                    RecvLowROS.imu.quaternion[1],
                                    RecvLowROS.imu.quaternion[2],
                                    RecvLowROS.imu.quaternion[3]); 

    base_rpy = Utils::quat_to_euler(root_quat);
};

void Quadruped::PIDgains(bool on_ground=false){
    double Kp[3];
    if (on_ground){
        Kp[0] = 20.0;
        Kp[1] = 30.0;
        Kp[2] = 50.0;
    }
    else{
        Kp[0] = 10.0;
        Kp[1] = 10.0;
        Kp[2] = 12.0;
    }
    
    for(int j=0; j<4; j++)
    {
        Kp_joint[j*3] = Kp[0];
        Kp_joint[j*3+1] = Kp[1];
        Kp_joint[j*3+2] = Kp[2];

        Kd_joint[j*3] = 1.0; 
        Kd_joint[j*3+1] = 1.0; 
        Kd_joint[j*3+2] = 1.0; 
    }

}

unitree_legged_msgs::LowCmd Quadruped::publishCommand(unitree_legged_msgs::LowCmd SendLowROS,float qDes[12]){
    for(int j=0; j<12;j++)
    {   
        ////// joint-level + toruqe
        SendLowROS.motorCmd[j].q = qDes[j];
        SendLowROS.motorCmd[j].dq = 0;
        SendLowROS.motorCmd[j].Kp = Kp_joint[j];
        SendLowROS.motorCmd[j].Kd = Kd_joint[j];
        SendLowROS.motorCmd[j].tau = 0;                 
    }
    SendLowROS.motorCmd[FR_0].tau = -0.65f;
    SendLowROS.motorCmd[FL_0].tau = +0.65f;
    SendLowROS.motorCmd[RR_0].tau = -0.65f;
    SendLowROS.motorCmd[RL_0].tau = +0.65f;

    return SendLowROS;
}

void Quadruped::stand_up(float* qDes, int rate_count, const float qInit[12], const float homing_pose_q[12]){
    rate = pow(rate_count/400.0,2); 
    for(int j=0; j<12;j++)
    {
        qDes[j] = jointLinearInterpolation(qInit[j], homing_pose_q[j], rate, 0);
    }
}

void Quadruped::select_reference(float* q_des, int rc) {
    for(int j=0; j<12;j++)
    {
        q_des[j] = history(j, rc);
    }
}



template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}


//###############################

template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    bool VERBOSE = true;

    int ctrl_estimation = 1000;

    ros::NodeHandle n;
    ros::Rate loop_rate(ctrl_estimation);

    ros::Publisher gait_data_pub; // for data_analysis
    ros::Publisher gait_data_pubx;  

    const float pi=3.1415926;

    long motiontime=0;

    Quadruped robot = Quadruped();

    TCmd SendLowLCM = {0};
    TState RecvLowLCM = {0};
    unitree_legged_msgs::LowCmd SendLowROS;
    unitree_legged_msgs::LowState RecvLowROS;

    sensor_msgs::JointState joint2simulation, joint2simulationx;

    // ------------------ Initialise some parameters ----------------------
    int count = 0;   
    int rate_count = 0;
    float qInit[12]= {0}; // Initial angle
    float qDes[12] = {0}; // Current desired angle
    // Desired homing pose
    float homing_pose_q[12] = {0.0, pi/4, -pi/2,0.0, pi/4, -pi/2,0.0, pi/4, -pi/2,0.0, pi/4, -pi/2};    
    
    // Jumping variable and counters
    int history_length;
    robot.upload_history("history.npy", history_length);
    int rc = 0;  
    int sub_rc = 1;
    float q_ref_current[12];
    float q_ref_next[12];
    robot.select_reference(q_ref_current, 0);
    robot.select_reference(q_ref_next, 1);

    //===desired joint angles
    FR_angle_des.setZero(); FL_angle_des.setZero(); RR_angle_des.setZero(); RL_angle_des.setZero(); 


    roslcm.SubscribeState();
    gait_data_pub = n.advertise<sensor_msgs::JointState>("go1_gait_data",10);
    gait_data_pubx = n.advertise<sensor_msgs::JointState>("go1_gait_datax",10);       

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    SendLowROS.levelFlag = LOWLEVEL;
    for(int i = 0; i<12; i++){
        SendLowROS.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
    }

//------------------ MAIN LOOP -------------------------
    while (ros::ok()){
        roslcm.Get(RecvLowLCM);
        RecvLowROS = ToRos(RecvLowLCM);

        robot.updateStates(RecvLowROS);
        
        motiontime++;
        // Record initial position
        if( motiontime >= 0 && motiontime < 10){
            for(int j=0; j<12;j++)
            {
                qInit[j] = RecvLowROS.motorState[j].q;
                qDes[j] = qInit[j];
            }
        }
        // Move to homing pose
        if( motiontime >= 10 && motiontime <=  400){
            // printf("%f %f %f\n", );
            rate_count++;
            robot.stand_up(qDes, rate_count, qInit, homing_pose_q);
        }
        // After 1000 steps, switch to on_ground PID gains (make sure the robot is on the ground)
        if (motiontime > 1000){
            bool on_ground = true;
            robot.PIDgains(on_ground);
        }


        // Jumping:
        if (motiontime >= 2000 && rc < history_length){
            if (sub_rc == 10) {
                rc++;
                sub_rc = 1;
                robot.select_reference(q_ref_current, rc);
                robot.select_reference(q_ref_next, rc + 1);
            }
            
            for(int j=0; j<12;j++)
                {
                qDes[j] = jointLinearInterpolation(q_ref_current[j],
                                                    q_ref_next[j],
                                                    sub_rc / 10.0,
                                                    0);
                }
            sub_rc++;
        }
        // map the commands to the ROS message:
        // SendLowROS = robot.publishCommand(SendLowROS,qDes);
        // SendLowLCM = ToLcm(SendLowROS, SendLowLCM);
        // roslcm.Send(SendLowLCM);

        if (VERBOSE){
        std::cout << "---------------------------------------" << std::endl;
        std::cout << "Force(1) is: " << RecvLowROS.footForce[0] << std::endl;
        std::cout << "Force(2) is: " << RecvLowROS.footForce[1] << std::endl;
        std::cout << "Force(3) is: " << RecvLowROS.footForce[2] << std::endl;
        std::cout << "Force(4) is: " << RecvLowROS.footForce[3] << std::endl;

        std::cout << "Error FL: " << robot.FL_angle_mea(0,0) - qDes[3] << std::endl;
        std::cout << robot.FL_angle_mea(1,0) - qDes[4] << std::endl;
        std::cout << robot.FL_angle_mea(2,0) - qDes[5] << std::endl;

        std::cout << "Gain for FR[0] is: " << robot.Kp_joint[0] << std::endl;
        }

        ros::spinOnce();
        loop_rate.sleep();
    };

    return 0;
}


int main(int argc, char *argv[]){
    ros::init(argc, argv, "torque_jumping_controller");

    UNITREE_LEGGED_SDK::LCM roslcm(LOWLEVEL);
    mainHelper<UNITREE_LEGGED_SDK::LowCmd, UNITREE_LEGGED_SDK::LowState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
}