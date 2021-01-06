#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/state/joint_position.hpp>
#include <iiwa_ros/command/cartesian_pose.hpp>
#include <iiwa_ros/command/joint_position.hpp>
#include <iiwa_ros/service/time_to_destination.hpp>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <ros/package.h>

#define TXT_FILE "/input/Bear_Coordinates.txt"

using namespace std;

iiwa_ros::command::CartesianPose iiwa_pose_command;
iiwa_ros::state::CartesianPose iiwa_pose_state;
iiwa_msgs::CartesianPose command_cartesian_position;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped command_pose;

// get current pose of robot in gazebo (not used for real robot)
void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  current_pose = *msg;
}

vector<string> split(string input, char delimiter){
	vector<string> ans;
	stringstream str(input);
	string temp;
	
	while(getline(str, temp, delimiter)){
		ans.push_back(temp);
	}
	
	return ans;
}

void commandRobot(bool sim, double y, double z, double x=0.6){
  // give command to robot
    if(sim == false){ // if real robot is operated
      command_cartesian_position.poseStamped.pose.position.x = x;
      command_cartesian_position.poseStamped.pose.position.y = y;
      command_cartesian_position.poseStamped.pose.position.z = z;
      iiwa_pose_command.setPose(command_cartesian_position.poseStamped);
      //waitForDestination(command_cartesian_position.poseStamped);
    }
    else{
      command_pose.pose.position.x = x;
      command_pose.pose.position.y = y;
      command_pose.pose.position.z = z;
      iiwa_pose_command.setPose(command_pose);
      //waitForDestination(command_pose);
    } 
}

/*
void waitForDestination(geometry_msgs::PoseStamped pose){
  while(){
    ros::Duration(0.5).sleep();
  }
}*/

int main (int argc, char **argv) {
	// Initialize ROS
  ros::init(argc, argv, "CommandRobot");
  ros::NodeHandle nh("~");
  ros::Subscriber sub;
  
  bool sim;
  nh.param("sim", sim, true);

  // init nodes
  iiwa_pose_command.init("iiwa");
  if(sim == false){ // if real robot is operated
    iiwa_pose_state.init("iiwa");
    std::cout << "SIMULATION OFF" << std::endl;
  }
  else {  // if simulation
    sub = nh.subscribe("/iiwa/state/CartesianPose", 5, chatterCallback);
    std::cout << "SIMULATION ON" << std::endl;
  }
 

  // TXT file with list of coordinates
  ifstream txt(ros::package::getPath("iiwa_examples")+TXT_FILE);
  // check if text file is well opened
  if(!txt.is_open()){
    cout << "FILE NOT FOUND \n" << endl;
    return 1;
  }


  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Dynamic parameter to choose the rate at wich this node should run
  double ros_rate;
  nh.param("ros_rate", ros_rate, 0.85); // 0.1 Hz = 10 seconds
  ros::Rate* loop_rate_ = new ros::Rate(ros_rate);

  string line;

  // initial orientation and position x
  command_pose.pose.orientation.x = 0.7;
  command_pose.pose.orientation.z = 0.7;
  command_cartesian_position.poseStamped = command_pose;

  // to draw lines in rviz
  ros::Publisher point_pub = nh.advertise<geometry_msgs::Point>("drawing_point", 100);
  geometry_msgs::Point point;

  while(ros::ok() && getline(txt, line)){
    loop_rate_->sleep();

    double y, z;

    if(line == "End"){
      // Get IIWA end-effector off the wall
      commandRobot(sim, y, z, 0.5);
      loop_rate_->sleep();

      cout << "-- END --" << endl;
      
      if(sim == true){  // draw lines in rviz if simulation
        point.x = current_pose.pose.position.x;

        point_pub.publish(point);
      }

      // Go to next stroke starting point (off the wall)
      getline(txt, line);
      vector<string> tempSplit = split(line, ' ');

      y = stod(tempSplit[0]);
			z = stod(tempSplit[1]);

      commandRobot(sim, y, z, 0.5);
      loop_rate_->sleep();
    }
    else{
      vector<string> tempSplit = split(line, ' ');

      y = stod(tempSplit[0]);
			z = stod(tempSplit[1]);

      if(sim == true){  // draw lines in rviz if simulation
        
        point.x = current_pose.pose.position.x;
        point.y = current_pose.pose.position.y;
        point.z = current_pose.pose.position.z; 
        
        /*
        point.x = current_pose.pose.position.x;
        point.y = y;
        point.z = z;*/

        point_pub.publish(point);
      }
      

      cout << y << " " << z << endl;
    }
    
    commandRobot(sim, y, z);
  }
}