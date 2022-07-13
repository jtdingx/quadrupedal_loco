#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <sys/types.h>
#include <iostream>
#include <time.h>
#include <fstream>   
#include <string>  
#include <cassert>
#include <vector>
#include "eigen3/Eigen/Dense" //for hardware experiments
#include "math.h"
#include "NLPRTControl/NLPRTControlClass.h"
#include "Robotpara/robot_const_para_config.h"


using namespace Eigen;
using namespace std;
using namespace gait;


sensor_msgs::JointState joint2simulation;

NLPRTControlClass nlp_planner;

bool mpc_start = false;

Eigen::Vector3d Rfoot_location_feedback;

Eigen::Vector3d Lfoot_location_feedback;

Eigen::Matrix<double,18,1> state_est;
Eigen::Matrix<double,25,1> state_feedback_receieved;

Matrix<double,100,1> mpc_gait_result;

extern double count;
extern double count_old;

void state_feed_sub_operation(const sensor_msgs::JointState::ConstPtr &msg)
{
    for (int jx = 0; jx<25; jx++)
    {
        
        state_feedback_receieved(jx) = msg->position[jx]; 
        if (jx<18)
        {
            state_est(jx) = state_feedback_receieved(jx+1);
        }

    }
    
    Lfoot_location_feedback(0) = msg->position[19];
    Lfoot_location_feedback(1) = msg->position[20];
    Lfoot_location_feedback(2) = msg->position[21];
    
    Rfoot_location_feedback(0) = msg->position[22];
    Rfoot_location_feedback(1) = msg->position[23];
    Rfoot_location_feedback(2) = msg->position[24];    
}



int main(int argc, char *argv[])
{

    ros::init(argc,argv,"mpc");
    ros::NodeHandle nh;

    joint2simulation.position.resize(100);
    state_est.setZero();
    state_feedback_receieved.setZero();
    mpc_gait_result.setZero();

    int count = 0;
    int count_old = 0;   
    double mpc_stop = 0;


    ros::Subscriber state_feedback_subscribe_ = nh.subscribe<sensor_msgs::JointState>("/rt2nrt/state", 10,state_feed_sub_operation);
    
    ros::Publisher mpc_gait_pub_ = nh.advertise<sensor_msgs::JointState>("/MPC/Gait", 10);

    int pub_rate  = (int) (round(1/dt_mpc_slow));

    ros::Rate nrt_rate(pub_rate);

    
    ros::Duration duratione_des(dt_mpc_slow);
    while (ros::ok())
    {   
        ros::Time start = ros::Time::now(); 

        
        // if (state_feedback_receieved(0) > 0)
        // {
        //     mpc_start =true;
        //     count += 1;
        // }

        mpc_start =true;
        count += 1;        

        ///// input: obtained from the ros::subscriber
        /////////// time interval
        /////////// estimated_statex
        /////////// _Rfoot_location_feedback / _Lfoot_location_feedback
        //if ((mpc_stop <1)&&(count > count_old))
        if ((mpc_stop <1))
        {
          mpc_gait_result = nlp_planner.WalkingReactStepping(count,mpc_start,state_est,Rfoot_location_feedback, Lfoot_location_feedback); 
	  
        }

        mpc_stop = mpc_gait_result(97,0);

        ros::Time end = ros::Time::now();
	
        ros::Duration duration = end -start;

        double t_slow_mpc = duration.toSec();	

        mpc_gait_result(98,0) = t_slow_mpc;

        for (int jx = 0; jx<100; jx++)
        {
          joint2simulation.position[jx] = mpc_gait_result(jx,0);
        }
        
        joint2simulation.header.stamp = ros::Time::now();
         

        


        mpc_gait_pub_.publish(joint2simulation);

        //count_old = (int) state_feedback_receieved(0,0);
 
        ros::spinOnce(); 
        
	
	
	    //// for simulation /////
        nrt_rate.sleep();
        /* code */
    }

    


    return 0;
}


