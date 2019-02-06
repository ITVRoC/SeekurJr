#include <ros/ros.h>
//#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "goal_publisher/SetGoal.h"


bool goal_constructor(goal_publisher::SetGoal::Request  &req,
         goal_publisher::SetGoal::Response &res)
{

// criar FollowJOintTrajectoryActionGoal chamada goal

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
    goal.goal.trajectory.points[ind].positions[0] = req.pos_0;
    goal.goal.trajectory.points[ind].positions[1] = req.pos_1;
    goal.goal.trajectory.points[ind].positions[2] = req.pos_2;
    goal.goal.trajectory.points[ind].positions[3] = req.pos_3;
    goal.goal.trajectory.points[ind].positions[4] = req.pos_4;
    goal.goal.trajectory.points[ind].positions[5] = req.pos_5;

/*	ROS_INFO("requisicao: %f ", req.pos_0);
	ROS_INFO("response:  %f ", goal.goal.trajectory.points[0].positions[0]);
*/
    // Velocities
    goal.goal.trajectory.points[ind].velocities.resize(6);
    for (size_t j = 0; j < 6; ++j)
    {
      goal.goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 3 seconds after starting along the trajectory
    goal.goal.trajectory.points[ind].time_from_start = ros::Duration(3.0);

    //escreve na mensagem de resposta (response)
    res.goal = goal;

    return true;

}



int main(int argc, char **argv)   /// argc e argv sao para receber parametros na linha de comando
{
    ros::init(argc, argv, "goal_publisher_server");      // iniciando ros com parametros e nome do no

    ros::NodeHandle n;  // ponto de acesso de comunicacao com o ros -- inicializa/cria o no

   

    ros::ServiceServer service = n.advertiseService("set_goal", goal_constructor);
    ROS_INFO("Ready to construsct goal");
    ros::spin();

return 0;

}



