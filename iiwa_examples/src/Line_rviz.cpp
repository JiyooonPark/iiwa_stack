#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

geometry_msgs::Point sub_msg;

// get current pose of robot in gazebo (not used for real robot)
void chatterCallback(const geometry_msgs::Point::ConstPtr& msg){
  sub_msg = *msg;
  //std::cout << "msg: " << sub_msg.x << " " << sub_msg.y << " " << sub_msg.z << std::endl;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Subscriber point_sub = nh.subscribe("/CommandRobot/drawing_point", 10, chatterCallback);

  //ros::Rate r(30);
  // Dynamic parameter to choose the rate at wich this node should run
  double ros_rate;
  nh.param("ros_rate", ros_rate, 0.5); // 0.1 Hz = 10 seconds
  ros::Rate* loop_rate_ = new ros::Rate(ros_rate);

  ros::spinOnce();

  geometry_msgs::Point p;
  p.x = 0;

  float id = 0.0;

  while (ros::ok())
  {
    // marker init
    visualization_msgs::Marker line;
    line.header.frame_id = "/my_frame";
    line.header.stamp =  ros::Time::now();
    line.id = 0.0 + id;
    line.ns = "points_and_lines";
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.scale.x = 0.01;   // markers use only the x component of scale, for the line width
    // Line strip is blue
    line.color.r = 1.0;
    line.color.g = 1.0;
    line.color.b = 1.0;
    line.color.a = 1.0;

    while(sub_msg.x > 0.55){
      p.y = sub_msg.y*10;
      p.z = sub_msg.z*10;

      line.points.push_back(p);

      std::cout << p.y << " " << p.z << std::endl;
      
      loop_rate_->sleep();
      ros::spinOnce();
    }
    marker_pub.publish(line);

    std::cout << "END: " << sub_msg.x << " " << sub_msg.y << " " << sub_msg.z << std::endl;

    id++;
    loop_rate_->sleep();
    ros::spinOnce();
  }
}