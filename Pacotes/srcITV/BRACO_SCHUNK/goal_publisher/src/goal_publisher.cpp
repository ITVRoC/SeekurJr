#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include "std_msgs/String.h"
#include <iostream>
//#include "geomagic_control/PhantomButtonEvent.h"
#include <pthread.h>

// Global variables
double arm_1;
double arm_2;
double arm_3;
double arm_4;
double arm_5;
double arm_6;
bool button_go;


void joint_stateCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
	arm_1 = joint_state->position[0];
	arm_2 = joint_state->position[1];
	arm_3 = joint_state->position[2];
	arm_4 = joint_state->position[3];
	arm_5 = joint_state->position[4];
	arm_6 = joint_state->position[5];
}

void buttonCallback(const sensor_msgs::Joy::ConstPtr& joy_test)
{
	button_go = joy_test->buttons[0];
	//std::cout << button->grey_button;
}


int main(int argc, char **argv)   
{
    ros::init(argc, argv, "goal_publisher");   
    ros::NodeHandle n;  

    ros::Publisher Goal_pub = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/arm_controller/follow_joint_trajectory/goal", 1000);
    ros::Subscriber Phantom_sub = n.subscribe("joint_states_test", 1000, joint_stateCallback);
    //ros::Subscriber Button_sub = n.subscribe("button", 1000, buttonCallback);
    ros::Subscriber Button_sub = n.subscribe("joy_test", 1000, buttonCallback);

    control_msgs::FollowJointTrajectoryActionGoal goal;

// Atribui valores para goal no formato da msg FollowJointTrajectoryActionGoal

// First, the joint names, which apply to all waypoints
    goal.goal.trajectory.joint_names.push_back("arm_1_joint");
    goal.goal.trajectory.joint_names.push_back("arm_2_joint");
    goal.goal.trajectory.joint_names.push_back("arm_3_joint");
    goal.goal.trajectory.joint_names.push_back("arm_4_joint");
    goal.goal.trajectory.joint_names.push_back("arm_5_joint");
    goal.goal.trajectory.joint_names.push_back("arm_6_joint");
	
// We will have one waypoints in this goal trajectory
    goal.goal.trajectory.points.resize(1);

// First trajectory point
    // Positions
    int ind = 0;
    goal.goal.trajectory.points[ind].positions.resize(6);        //home
    goal.goal.trajectory.points[ind].positions[0] = 0.0;
    goal.goal.trajectory.points[ind].positions[1] = 0.0;
    goal.goal.trajectory.points[ind].positions[2] = 0.0;
    goal.goal.trajectory.points[ind].positions[3] = 0.0;
    goal.goal.trajectory.points[ind].positions[4] = 0.0;
    goal.goal.trajectory.points[ind].positions[5] = 0.0;


    // Velocities
    goal.goal.trajectory.points[ind].velocities.resize(6);
    for (size_t j = 0; j < 6; ++j)
    {
      goal.goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 3 seconds after starting along the trajectory
    goal.goal.trajectory.points[ind].time_from_start = ros::Duration(3.0);


// Imprime valores da string  (cout) na linha de comando que eh executado o no (rosrun)

	//ROS_INFO("%s", goal);


 while(Goal_pub.getNumSubscribers()==0)
{
   ROS_ERROR("Waiting for subscribers");
   sleep(5);
}
   ROS_INFO("Got subscribers");

	// publica a mensagem criada (goal) no topico /armcontro... atraves do objeto criado anteriormente (Goal_pub)
		

while (ros::ok())    // enquanto a comunicacao com o ros estiver ok execute o programa abaixo
{	
	


	goal.goal.trajectory.points[ind].positions[0] = arm_1;
	goal.goal.trajectory.points[ind].positions[1] = arm_2;
//	if ((arm_1 < 0.3) && (arm_1 > -0.3) && (arm_2 < -1.05) && (arm_2 > -1.4) && (arm_3 > 1.65)){
	if ((arm_2 < -1.05) && (arm_2 > -1.4) && (arm_3 > 1.65)){
		goal.goal.trajectory.points[ind].positions[2] = 1.65;
	    }
	else if ((arm_2 < -1.4) && (arm_3 > 1.50)){
		goal.goal.trajectory.points[ind].positions[2] = 1.50;
	    }
	else{
		goal.goal.trajectory.points[ind].positions[2] = arm_3;
	    }
	
	//goal.goal.trajectory.points[ind].positions[2] = arm_3;
    	if ((arm_4 > -1.1) && (arm_4 < 1.1)){
    		goal.goal.trajectory.points[ind].positions[3] = arm_4;
    	    }
    	if (arm_4 < -1.1){
    		goal.goal.trajectory.points[ind].positions[3] = -1.1;	
    	    }	
    	if (arm_4 > 1.1){
    		goal.goal.trajectory.points[ind].positions[3] = 1.1;	
    	    }	
    	
    	
    	goal.goal.trajectory.points[ind].positions[4] = arm_5;
    	goal.goal.trajectory.points[ind].positions[5] = 0.0;
	
	if (button_go != 0){
	Goal_pub.publish(goal);
	sleep(2);
	}
// verificacao de callbacks ( exemplo: apertou um botao.. muda valor de variaveis )
	ros::spinOnce();

}

return 0;

}
