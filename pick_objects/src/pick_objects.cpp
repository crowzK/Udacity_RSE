#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 4.0;
  goal.target_pose.pose.position.y = -3.0;
  goal.target_pose.pose.orientation.w = 2.0;

  ROS_INFO("Robot is traveling to the pickup zone");

  ac.sendGoal(goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Robot picked up the virtual object");
  else
    ROS_INFO("Robot failed to move meter for some reason");
  ros::Duration(5).sleep();

  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = -2.0;
  ROS_INFO("Robot is traveling to the drop off zone");

  ac.sendGoal(goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Robot dropped the virtual object");
  else
    ROS_INFO("Robot failed to move meter for some reason");

  ros::spin();
  return 0;
}