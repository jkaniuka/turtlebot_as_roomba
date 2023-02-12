#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include "package_303762/path_generator_alg.hpp"
#include <iostream>
#include <chrono>
#include <unistd.h>
#include "std_msgs/String.h"

#define TIME_LIMIT 10 // timeout for reaching the next point



float x_start;
float y_start;
float x_end;
float y_end;


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


using namespace std;
int action_status = 0;
string room_to_clean;
bool start_cleaning = 0;
bool is_done_once = 1;
bool stop_now = 0;


void roomba_cmd_callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  string received_msg =  (msg->data.c_str());
  if(received_msg.compare("STOP") == 0)
  {
    stop_now = 1;
  }
  else{
  room_to_clean = (msg->data.c_str());
  start_cleaning = 1;
  is_done_once = 0;
  }
}


// retrieve action execution status from /move_base/status topic
void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
	if(msg->status_list.empty()){
	}
	else{
		action_status = msg->status_list[0].status;
	}
}

// build move_base_msgs::MoveBaseGoal message struct
move_base_msgs::MoveBaseGoal build_action_goal_struct(int goal_num, vector<point_t> result_array)
{
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = result_array[goal_num].x;
  goal.target_pose.pose.position.y = result_array[goal_num].y;
  goal.target_pose.pose.orientation.z = result_array[goal_num].z;
  goal.target_pose.pose.orientation.w = result_array[goal_num].w;
  return goal;
}




int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh("~");
  ros::Subscriber sub = nh.subscribe("/move_base/status", 1000, statusCallback);
  ros::Subscriber sub_cmd = nh.subscribe("/simple_navigation_goals/roomba_commands", 1000, roomba_cmd_callback);
  ros::Publisher status_pub = nh.advertise<std_msgs::String>("/simple_navigation_goals/roomba_cleaning_status", 1000);

  MoveBaseClient ac("/move_base", true);
  vector<point_t> result_array;
  // number of current goal (subgoal)
  int goal_num = 0;
  bool too_long = 0; // flag indicating that robot didn't reach goal within timelimit
  int cancel_num; // cancelled goal counter


  ros::Rate loop_rate(10);   
  while (ros::ok())
  {

    if (is_done_once == 0){

    // List of start points in each room
	if(room_to_clean.compare("office") == 0)
	{
		x_start = 5.5; y_start = -1;
		x_end = 7; y_end = -4.5;
	}
	 else if(room_to_clean.compare("cafeteria") == 0)
	{
		x_start = 2.5; y_start = 0.5;
		x_end = 7; y_end = 4.5;
	}
	 else if(room_to_clean.compare("dumpster") == 0)
	{
		x_start = 2; y_start = 4.5;
		x_end = 0.5; y_end = 1.5;
	}
	 else if(room_to_clean.compare("kitchen") == 0)
	{
		x_start = -0.5; y_start = 0.5;
		x_end = -4.5; y_end = 4.5;
	}
	 else if(room_to_clean.compare("dressing_room") == 0)
	{
		x_start = -5.5; y_start = 4.5;
		x_end = -6.5; y_end = 1.5;
	}
	 else if(room_to_clean.compare("bathroom") == 0)
	{
		x_start = -5.5; y_start = 0.5;
		x_end = -6.5; y_end = -3.5;
	}
	else
	{
		ROS_ERROR("Wrong room name !!!");
	} 




    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
    }


    // get discrete path from path generator
    result_array = generate_discrete_path(x_start, y_start, x_end, y_end);
    

    // move to start point
    // the starting point was chosen in such a place that the robot will always be able to get there
    ROS_INFO("Sending goal");
    ac.sendGoal(build_action_goal_struct(goal_num, result_array));
    ac.waitForResult(); 

    is_done_once = 1;
    }


    if (start_cleaning == 1 && stop_now == 0){

  	if (cancel_num == 2)
  	{
  		ROS_WARN("Two subgoals has been skipped - waiting for DWAPlanner");
  	}



  	if (goal_num == result_array.size()-1)
  	{
  		ROS_INFO("ROOM HAS BEEN CLEANED");
		std_msgs::String msg;
		msg.data = "FINISHED";
		while (status_pub.getNumSubscribers() < 1) {
			}
		status_pub.publish(msg);
		start_cleaning = 0;
  		//return 0;
  	}

  	else if (action_status == 2 || action_status == 4 || action_status == 5 || (too_long && cancel_num <=2) ) // PREEMPTED, ABORTED, REJECTED
  	{
  		too_long = 0;
  		ROS_WARN("Cannot reach");
   		goal_num++;
			ROS_INFO("Sending goal");
		  ac.sendGoal(build_action_goal_struct(goal_num, result_array));
		  bool finished_before_timeout = ac.waitForResult(ros::Duration(TIME_LIMIT));
		  if (finished_before_timeout)
		  {
		    ROS_INFO("Action finished in time");
		  }
		  else
		  {
		    ROS_WARN("Action did not finish before the time out.");
		  	too_long = 1;
		  	cancel_num++;
		  }

  	}
  	else if (action_status == 3) //SUCCEEDED
  	{
  		ROS_INFO("Point achieved");
  		cancel_num = 0;
  		goal_num++;
			ROS_INFO("Sending goal");
		  ac.sendGoal(build_action_goal_struct(goal_num, result_array));
		  bool finished_before_timeout = ac.waitForResult(ros::Duration(TIME_LIMIT));
		  if (finished_before_timeout)
		  {
		    ROS_INFO("Action finished in time");
		  }
		  else
		  {
		    ROS_WARN("Action did not finish before the time out.");
		  	too_long = 1;
		  	cancel_num++;
		  }
  	}

    }

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
