#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  float pickup[3] = {-18.0,3.8,1.0};
  float dropoff[3] = {1,-4,1.0};

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";

  ////////// Pickup //////////

  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach (pickup [-18,3.8], dropoff [1,-4]
  goal.target_pose.pose.position.x = pickup[0]; 
  goal.target_pose.pose.position.y = pickup[1]; 
  goal.target_pose.pose.orientation.w = pickup[2];

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Atempting to reach the pickup zone");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, pickup zone reached");
  else
    ROS_INFO("Failed to reach the pickup zone for some reason");
  
  ros::Duration(5.0).sleep();

  /////////// Dropoff ////////////////
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach (pickup [-18,3.8], dropoff [1,-4]
  goal.target_pose.pose.position.x = dropoff[0]; 
  goal.target_pose.pose.position.y = dropoff[1]; 
  goal.target_pose.pose.orientation.w = dropoff[2];

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Atempting to reach the dropoff zone");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, dropoff zone reached");
  else
    ROS_INFO("Failed to reach the dropoff zone for some reason");


  return 0;
}
