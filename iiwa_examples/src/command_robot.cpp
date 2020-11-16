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

int main (int argc, char **argv) {

	// Initialize ROS
  ros::init(argc, argv, "CommandRobot");
  ros::NodeHandle nh("~");

  //iiwa_ros::state::CartesianPose iiwa_pose_state;
  iiwa_ros::command::CartesianPose iiwa_pose_command;
  //iiwa_pose_state.init("iiwa");
  iiwa_pose_command.init("iiwa");

  iiwa_msgs::CartesianPose command_cartesian_position;

  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Subscriber sub = nh.subscribe("/iiwa/state/CartesianPose", 1000, chatterCallback);

  // Dynamic parameter to choose the rate at wich this node should run
  double ros_rate;
  nh.param("ros_rate", ros_rate, 0.1); // 0.1 Hz = 10 seconds
  ros::Rate* loop_rate_ = new ros::Rate(ros_rate);
  int direction = 0.2;

  while(ros::ok()){
    ros::Duration(2.0).sleep();
    //command_cartesian_position = iiwa_pose_state.getPose();

    std::cout << "POSE: \n" << current_pose.pose.position << std::endl;
    current_pose.pose.position.z -= direction * 0.10;
    iiwa_pose_command.setPose(command_cartesian_position.poseStamped);

    std::cout << "-------------------" << std::endl;

    direction *= -1;
	loop_rate_->sleep();
  }

  /*
  iiwa_ros::state::JointPosition iiwa_joint_state;
  iiwa_ros::command::JointPosition iiwa_joint_command;
  iiwa_ros::service::TimeToDestinationService iiwa_time_destination;

  iiwa_joint_state.init("iiwa");
  iiwa_joint_command.init("iiwa");
  iiwa_time_destination.init("iiwa"); */
}