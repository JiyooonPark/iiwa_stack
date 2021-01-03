#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/state/joint_position.hpp>
#include <iiwa_ros/command/cartesian_pose.hpp>
#include <iiwa_ros/command/joint_position.hpp>
#include <iiwa_ros/service/time_to_destination.hpp>
#include <cmath>

geometry_msgs::PoseStamped current_pose;

void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  current_pose = *msg;
}

float checkDiff(geometry_msgs::PoseStamped cp, geometry_msgs::PoseStamped goal_pose){
  float diff = goal_pose.pose.position.x - cp.pose.position.x;
  float tot = abs(diff);

  diff = goal_pose.pose.position.y - cp.pose.position.y;
  tot += abs(diff);
  diff = goal_pose.pose.position.z - cp.pose.position.z;
  tot += abs(diff);

  std::cout << tot << std::endl;

  return tot;
}

int main (int argc, char **argv) {

	// Initialize ROS
	ros::init(argc, argv, "CommandRobot");
	ros::NodeHandle nh("~");

  iiwa_ros::command::CartesianPose iiwa_pose_command;
  iiwa_pose_command.init("iiwa");

  // ROS spinner.
	ros::AsyncSpinner spinner(1);
	spinner.start();

  ros::Subscriber sub = nh.subscribe("/iiwa/state/CartesianPose", 1000, chatterCallback);

  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = current_pose.header.frame_id;
  goal_pose.pose.position.x = 0.6;
  goal_pose.pose.position.y = 0;
  goal_pose.pose.position.z = 0.4;
  goal_pose.pose.orientation.x = 0.0;
  goal_pose.pose.orientation.y = 0.8;
  goal_pose.pose.orientation.z = 0.0;
  goal_pose.pose.orientation.w = 0.8;

  while(checkDiff(current_pose, goal_pose) > 0.2){
    iiwa_pose_command.setPose(goal_pose);
    
    ros::Duration(2.0).sleep();
    std::cout << "-------------------" << std::endl;
  }
  
  /*
  iiwa_ros::state::CartesianPose iiwa_pose_state;
  iiwa_ros::state::JointPosition iiwa_joint_state;
  iiwa_ros::command::CartesianPose iiwa_pose_command;
  iiwa_ros::command::JointPosition iiwa_joint_command;
  iiwa_ros::service::TimeToDestinationService iiwa_time_destination;

  iiwa_pose_state.init("iiwa");
  iiwa_pose_command.init("iiwa");
  iiwa_joint_state.init("iiwa");
  iiwa_joint_command.init("iiwa");
  iiwa_time_destination.init("iiwa"); */
}