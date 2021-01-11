#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/state/joint_position.hpp>
#include <iiwa_ros/command/cartesian_pose.hpp>
#include <iiwa_ros/command/joint_position.hpp>
#include <iiwa_ros/service/time_to_destination.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/package.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#define TXT_FILE "/input/Bear_Coordinates.txt"

using namespace std;
using moveit::planning_interface::MoveItErrorCode;

iiwa_ros::command::JointPosition iiwa_joint_command;
iiwa_ros::command::CartesianPose iiwa_pose_command;
iiwa_msgs::JointPosition current_joint_position;
iiwa_msgs::JointPosition iiwa_joint_position;
iiwa_msgs::CartesianPose iiwa_cartesian_command;
geometry_msgs::PoseStamped current_cartesian_position, command_cartesian_position, start, end;
std::string joint_position_topic, cartesian_position_topic;
std::vector<geometry_msgs::Pose> drawing_stroke;
std::vector<geometry_msgs::Pose> linear_path;
geometry_msgs::Pose drawing_point;
geometry_msgs::Pose path_point;

// Create MoveGroup
static const std::string PLANNING_GROUP = "manipulator";
static const std::string EE_LINK = "iiwa_link_ee";

bool sim;

vector<string> split(string input, char delimiter){
    vector<string> ans;
    stringstream str(input);
    string temp;
    
    while(getline(str, temp, delimiter)){
        ans.push_back(temp);
    }
    
    return ans;
}

int main (int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "CommandRobotMoveit");
    ros::NodeHandle nh("~");
    ros::Subscriber sub;

    // ROS spinner.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    std::string movegroup_name, ee_link;
    nh.param("sim", sim, true);

    // Create Move Group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.001;
    const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("iiwa_link_0");
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    // Configure Move Group
    move_group.setPlanningTime(0.5);
    move_group.setPlannerId(PLANNING_GROUP+"[RRTConnectkConfigDefault]");
    move_group.setEndEffectorLink(ee_link);
    
    if(sim == false){ // if real robot is operated
        // init nodes
        iiwa_pose_command.init("iiwa");
        iiwa_joint_command.init("iiwa");
        ROS_INFO("SIMULATION OFF");
    }
    else {  // if simulation
        ROS_INFO("SIMULATION ON");
        ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());
        ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
    }
  

    // TXT file with list of coordinates
    ifstream txt(ros::package::getPath("iiwa_examples")+TXT_FILE);
    // check if text file is well opened
    if(!txt.is_open()){
        cout << "FILE NOT FOUND" << endl;
        return 1;
    }

    // // Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
    // nh.param<string>("joint_position_topic", joint_position_topic, "/iiwa/state/JointPosition");
    // nh.param<string>("cartesian_position_topic", cartesian_position_topic, "/iiwa/state/CartesianPose");

    // Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
    nh.param<std::string>("move_group", movegroup_name, PLANNING_GROUP);
    nh.param<std::string>("ee_link", ee_link, EE_LINK);

    // // Dynamic parameter to choose the rate at wich this node should run
    // double ros_rate;
    // nh.param("ros_rate", ros_rate, 0.01); // 0.1 Hz = 10 seconds
    // ros::Rate* loop_rate_ = new ros::Rate(ros_rate);

    // // Subscribers and publishers
    // ros::Subscriber sub_joint_position = nh.subscribe(joint_position_topic, 1, jointPositionCallback);
    // ros::Subscriber sub_cartesian_position = nh.subscribe(cartesian_position_topic, 1, cartesianPositionCallback);

    string line;
    bool init = false;
    double x, y, z, fraction;
    MoveItErrorCode success_plan = MoveItErrorCode::FAILURE, motion_done = MoveItErrorCode::FAILURE;

    // to draw lines in rviz
    ros::Publisher point_pub = nh.advertise<geometry_msgs::Point>("drawing_point", 100);
    geometry_msgs::Point point;
    
    // initialization befroe start drawing
    while (ros::ok() && !init){
        ros::Duration(5).sleep(); // wait for 2 sec
        ROS_INFO("Sleeping 5 seconds before starting ... ");

        // set all the joint values to the init joint position
        move_group.setStartStateToCurrentState();
        move_group.setJointValueTarget("iiwa_joint_1", 0.0);
        move_group.setJointValueTarget("iiwa_joint_2", 0.435332);
        move_group.setJointValueTarget("iiwa_joint_3", 0.0);
        move_group.setJointValueTarget("iiwa_joint_4", -1.91986);
        move_group.setJointValueTarget("iiwa_joint_5", 0.0);
        move_group.setJointValueTarget("iiwa_joint_6", -0.785399);
        move_group.setJointValueTarget("iiwa_joint_7", 0.0);
        success_plan = move_group.plan(my_plan);
        if (success_plan == MoveItErrorCode::SUCCESS) {
            motion_done = move_group.execute(my_plan);
            
        }
        ROS_INFO("Moved to the initial position");
        ROS_INFO("Sleeping 3 seconds before starting ... ");
        ros::Duration(3).sleep(); // wait for 2 sec
        
        init = true;
        current_cartesian_position = move_group.getCurrentPose(ee_link);   
        command_cartesian_position = current_cartesian_position;
        drawing_point = current_cartesian_position.pose;

        x = current_cartesian_position.pose.position.x + 0.04; // default x position

        // linear_path.push_back(current_cartesian_position.pose);
    }

    int stroke_num = 0;
    bool ready_to_draw = false;
    while(ros::ok() && getline(txt, line) && init){

        if(line == "End"){
            stroke_num++;
            // move forward first to draw
            command_cartesian_position.pose = drawing_stroke[0];
            linear_path.push_back(command_cartesian_position.pose);
            fraction = move_group.computeCartesianPath(linear_path, eef_step, jump_threshold, trajectory);
            my_plan.trajectory_ = trajectory;
            move_group.execute(my_plan);  //ros::Duration(0.1).sleep();
            if (fraction < 0.5) ROS_WARN_STREAM("MOVE FORWARD ERROR");

            linear_path.clear();

            // draw a stroke
            ROS_INFO("Drawing %d th stroke ...", stroke_num);

            fraction = move_group.computeCartesianPath(drawing_stroke, eef_step, jump_threshold, trajectory);
            my_plan.trajectory_ = trajectory;
            
            ROS_INFO("Visualizing drawing plan (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
            if (fraction < 0.5) ROS_WARN_STREAM("LINE DRAWING ERROR");
            
            motion_done = move_group.execute(my_plan); //ros::Duration(0.1).sleep();
            if (motion_done == MoveItErrorCode::SUCCESS){
                visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group, rviz_visual_tools::colors::WHITE);
                visual_tools.trigger();
            }
            else {
                ROS_WARN_STREAM("LINE EXECUTION ERROR");
                visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group, rviz_visual_tools::colors::RED);
                visual_tools.trigger();
            }

            // cout << "################ drawing line status: " << draw_line << endl;

            command_cartesian_position = move_group.getCurrentPose(ee_link);  

            // move backward
            // linear_path.push_back(command_cartesian_position.pose);
            command_cartesian_position.pose.position.x -= 0.02;
            linear_path.push_back(command_cartesian_position.pose);
            fraction = move_group.computeCartesianPath(linear_path, eef_step, jump_threshold, trajectory);
            my_plan.trajectory_ = trajectory;
            move_group.execute(my_plan);  //ros::Duration(0.1).sleep();
            if (fraction < 0.5) ROS_WARN_STREAM("MOVE BACKWARD ERROR");

            linear_path.clear();
            // linear_path.push_back(command_cartesian_position.pose);

            ready_to_draw = false;
            drawing_stroke.clear();
        }
        else{
            // read drawing
            vector<string> tempSplit = split(line, ' ');
            y = stod(tempSplit[0]);
            z = stod(tempSplit[1]);

            if (!ready_to_draw){
                // move to the ready position (off the wall)
                command_cartesian_position.pose.position.x = x-0.02;
                command_cartesian_position.pose.position.y = y;
                command_cartesian_position.pose.position.z = z;

                linear_path.push_back(command_cartesian_position.pose);
                fraction = move_group.computeCartesianPath(linear_path, eef_step, jump_threshold, trajectory);
                my_plan.trajectory_ = trajectory;
                move_group.execute(my_plan);  //ros::Duration(0.1).sleep();
                if (fraction < 0.5) ROS_WARN_STREAM("MOVE READY POSITION ERROR");

                linear_path.clear();
                // linear_path.push_back(command_cartesian_position.pose);
                ready_to_draw = true;
            }

            drawing_point.position.x = x;
            drawing_point.position.y = y;
            drawing_point.position.z = z;
            drawing_stroke.push_back(drawing_point); // push the point
            // cout << y << " " << z << endl;

        }
    }

    cerr<<"Stopping spinner..."<<endl;
    spinner.stop();

    cerr<<"Bye!"<<endl;

    return 0;
}