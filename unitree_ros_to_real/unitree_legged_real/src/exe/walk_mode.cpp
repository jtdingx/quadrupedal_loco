/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <pthread.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "convert.h"
#include "sensor_msgs/JointState.h"

using namespace UNITREE_LEGGED_SDK;

template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    // SetLevel(HIGHLEVEL);
    long motiontime = 0;
    TCmd SendHighLCM = {0};
    TState RecvHighLCM = {0};
    unitree_legged_msgs::HighCmd SendHighROS;
    unitree_legged_msgs::HighState RecvHighROS;


    // sensor_msgs::JointState joint2simulation, joint2simulationx;
    // joint2simulation.position.resize(100);
    // joint2simulationx.position.resize(100);


    roslcm.SubscribeState();

    // gait_data_pub = n.advertise<sensor_msgs::JointState>("go1_gait_data",10);
    // gait_data_pubx = n.advertise<sensor_msgs::JointState>("go1_gait_datax",10);      

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    while (ros::ok()){
        motiontime = motiontime+2;
        roslcm.Get(RecvHighLCM);
        RecvHighROS = ToRos(RecvHighLCM);
        printf("fr leg force: %d\n",RecvHighROS.footForce[0]);
        printf("fr leg force_est: %d\n",RecvHighROS.footForceEst[1]);
        printf("fl leg force: %d\n",RecvHighROS.footForce[2]);
        printf("fl leg force_est: %d\n",RecvHighROS.footForceEst[3]);
        //printf("body height: %f\n",RecvHighROS.bodyHeight);

        SendHighROS.mode = 0;      
        SendHighROS.gaitType = 0;
        SendHighROS.speedLevel = 0;
        SendHighROS.footRaiseHeight = 0;
        SendHighROS.bodyHeight = 0;
        SendHighROS.euler[0]  = 0;
        SendHighROS.euler[1] = 0;
        SendHighROS.euler[2] = 0;
        SendHighROS.velocity[0] = 0.0f;
        SendHighROS.velocity[1] = 0.0f;
        SendHighROS.yawSpeed = 0.0f;
        SendHighROS.reserve = 0;

        if(motiontime > 0 && motiontime < 1000){
            SendHighROS.mode = 1;
            SendHighROS.euler[0] = -0.3;
        }
        if(motiontime > 1000 && motiontime < 2000){
            SendHighROS.mode = 1;
            SendHighROS.euler[0] = 0.3;
        }
        if(motiontime > 2000 && motiontime < 3000){
            SendHighROS.mode = 1;
            SendHighROS.euler[1] = -0.2;
        }
        if(motiontime > 3000 && motiontime < 4000){
            SendHighROS.mode = 1;
            SendHighROS.euler[1] = 0.2;
        }
        if(motiontime > 4000 && motiontime < 5000){
            SendHighROS.mode = 1;
            SendHighROS.euler[2] = -0.2;
        }
        if(motiontime > 5000 && motiontime < 6000){
            SendHighROS.mode = 1;
            SendHighROS.euler[2] = 0.2;
        }
        if(motiontime > 6000 && motiontime < 7000){
            SendHighROS.mode = 1;
            SendHighROS.bodyHeight = -0.2;
        }
        if(motiontime > 7000 && motiontime < 8000){
            SendHighROS.mode = 1;
            SendHighROS.bodyHeight = 0.1;
        }
        if(motiontime > 8000 && motiontime < 9000){
            SendHighROS.mode = 1;
            SendHighROS.bodyHeight = 0.0;
        }
        if(motiontime > 9000 && motiontime < 11000){
            SendHighROS.mode = 5;
        }
        if(motiontime > 11000 && motiontime < 13000){
            SendHighROS.mode = 6;
        }
        if(motiontime > 13000 && motiontime < 14000){
            SendHighROS.mode = 0;
        }
        if(motiontime > 14000 && motiontime < 18000){
            SendHighROS.mode = 2;
            SendHighROS.gaitType = 2;
            SendHighROS.velocity[0] = 0.4f; // -1  ~ +1
            SendHighROS.yawSpeed = 2;
            SendHighROS.footRaiseHeight = 0.1;
            // printf("walk\n");
        }
        if(motiontime > 18000 && motiontime < 20000){
            SendHighROS.mode = 0;
            SendHighROS.velocity[0] = 0;
        }
        if(motiontime > 20000 && motiontime < 24000){
            SendHighROS.mode = 2;
            SendHighROS.gaitType = 1;
            SendHighROS.velocity[0] = 0.2f; // -1  ~ +1
            SendHighROS.bodyHeight = 0.1;
            // printf("walk\n");
        }
        if(motiontime>24000 ){
            SendHighROS.mode = 1;
        }

        // ///********************* data saving ************************************///////
        // joint2simulation.header.stamp = ros::Time::now();
        // joint2simulationx.header.stamp = ros::Time::now();
        // ////
        // for(int j=0; j<12; j++){
        //         joint2simulation.position[j] = SendLowROS.motorCmd[j].q; // desired joint angles; 
        // }
        // for(int j=0; j<12; j++)
        // {
        //     joint2simulation.position[12+j] = RecvLowROS.motorState[j].q;   // measured joint angles;
        // } 


        // // joint torque desired 
        // for(int j=0; j<12; j++)
        // {
        //     joint2simulation.position[78+j] = SendLowROS.motorCmd[j].tau;
        // }

        // //// torque measured
        // for(int j=0; j<12; j++)
        // {
        //     joint2simulationx.position[j] = RecvLowROS.motorState[j].tauEst;
        // }

        // gait_data_pub.publish(joint2simulation);
        // gait_data_pubx.publish(joint2simulationx);


        SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
        roslcm.Send(SendHighLCM);
        ros::spinOnce();
        loop_rate.sleep(); 
    }
    return 0;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "walk_ros_mode");

    UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
    mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
    
}