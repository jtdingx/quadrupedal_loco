#include <ros/ros.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "convert.h"
#include "sensor_msgs/JointState.h"
#include <Eigen/Dense>
#include "go1_const.h"
#include "torque_mode.h"
#include "Robotpara/robot_const_para_config.h"
#include "geometry_msgs/Twist.h"

//###############################


class Quadruped{
    public:

    Eigen::Matrix<double,3,1> FR_angle_mea, FL_angle_mea, RR_angle_mea, RL_angle_mea;
    Eigen::Matrix<double,3,1> FR_dq_mea, FL_dq_mea, RR_dq_mea, RL_dq_mea;
    // Body position and orientation:
    Eigen::Vector3d base_pos;
    Eigen::Vector3d base_rpy; // LATER CORRECT FOR DRIFT
    float[12] Kp_joint;
    float[12] Kd_joint;

    Quadruped(){
        PIDgains();
        zeroStates();
    }

    
    void zeroStates(){
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

    void updateStates(unitree_legged_msgs::LowState RecvLowROS){
        """This function assigns the received robot states to the respective variables."""
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


        base_rpy = Eigen::Vector3d(RecvLowROS.imu.rpy[0],
                                RecvLowROS.imu.rpy[1],
                                RecvLowROS.imu.rpy[2]); 
    };

    void PIDgains(){

        for(int j=0; j<12; j++)
        {
            Kp_joint[j] = 5.0;
            Kd_joint[j] = 1.0; 
        }

    }

    void publishCommand(unitree_legged_msgs::LowCmd SendLowROS,float qDes){
        for(int j=0; j<12;j++)
        {   
            ////// joint-level + toruqe
            SendLowROS.motorCmd[j].q = qDes[j];
            SendLowROS.motorCmd[j].dq = 0;
            SendLowROS.motorCmd[j].Kp = Kp_joint[j];
            SendLowROS.motorCmd[j].Kd = Kd_joint[j];
            SendLowROS.motorCmd[j].tau = 0;                 
        }
        cmd.motorCmd[FR_0].tau = -0.65f;
        cmd.motorCmd[FL_0].tau = +0.65f;
        cmd.motorCmd[RR_0].tau = -0.65f;
        cmd.motorCmd[RL_0].tau = +0.65f;

        return SendLowROS
    }
};




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
    // dqDes[j] = 0.8*dqDes_old[j] + 0.2*(p - qDes[j])/(1.0/ctrl_estimation);
    // dqDes_old[j] = dqDes[j];
    return p;
}


//###############################

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

    robot = Quadruped()

    TCmd SendLowLCM = {0};
    TState RecvLowLCM = {0};
    unitree_legged_msgs::LowCmd SendLowROS;
    unitree_legged_msgs::LowState RecvLowROS;

    sensor_msgs::JointState joint2simulation, joint2simulationx;
    joint2simulation.position.resize(100);
    joint2simulationx.position.resize(100);    

    bool initiated_flag = false;  // initiate need time


    //////// for trajectory generation  ======================
    int count = 0;   

    int rate_count = 0;
    float qInit[12]={0};
    float qDes[12] = {0};

    float sin_mid_q[12] = {0.0, pi/4, -pi/2,0.0, pi/4, -pi/2,0.0, pi/4, -pi/2,0.0, pi/4, -pi/2};
    double sin_joint[12] = {0};

    stand_count = 0;
    stand_up_count = 0;
    dynamic_count = 0;

    ///////// leg controll ==============================//////////////////
    

    //===desired joint angles
    FR_angle_des.setZero(); FL_angle_des.setZero(); RR_angle_des.setZero(); RL_angle_des.setZero(); 

    angle_des.setZero();


    n_count = 0;
    stand_duration = 2; /// stand up: 2s


    roslcm.SubscribeState();
    gait_data_pub = n.advertise<sensor_msgs::JointState>("go1_gait_data",10);
    gait_data_pubx = n.advertise<sensor_msgs::JointState>("go1_gait_datax",10);       

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    SendLowROS.levelFlag = LOWLEVEL;
    for(int i = 0; i<12; i++){
        SendLowROS.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
    }



    while (ros::ok()){
        roslcm.Get(RecvLowLCM);
        RecvLowROS = ToRos(RecvLowLCM);
        robot.updateStates();
        
        Eigen::Vector3d root_euler = Utils::quat_to_euler(robot.root_quat);

        motiontime++;
        // Record initial position
        if( motiontime >= 0 && motiontime < 10){
            for(int j=0; j<12;j++)
            {
                qInit[j] = RecvLowROS.motorState[j].q;
                qDes[j] = qInit[j];
            }
        }
        //// move to the homing pose
        if( motiontime >= 10 && motiontime <=  400){
            // printf("%f %f %f\n", );
            rate_count++;
            rate = rate_count/200.0;//pow(rate_count/400.0,2); 
            for(int j=0; j<12;j++)
            {
                qDes[j] = jointLinearInterpolation(qInit[j], sin_mid_q[j], rate, 0);
            }
        }

        SendLowROS = robot.publishCommand(SendLowROS,qDes);
        SendLowLCM = ToLcm(SendLowROS, SendLowLCM);
        roslcm.Send(SendLowLCM);

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