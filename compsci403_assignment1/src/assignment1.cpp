#include <iostream>
#include <ros/ros.h>
#include <sstream>

#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>
#include "compsci403_assignment1/Assignment1CallerSrv.h"
#include "compsci403_assignment1/Assignment1ProviderSrv.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

// Include any additional header or service files

using std::cout;
using std::string;

// Declare class variables and publishers

ros::Publisher publish_test;
ros::Subscriber laser_scan_sub;
ros::ServiceClient client;
ros::ServiceServer provider_service;
ros::Publisher publish_point_cloud;
// Define service and callback functions

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  sensor_msgs::PointCloud pc;
  pc.header = msg->header;
  pc.points.resize(msg->ranges.size());
  for(unsigned int i = 0; i < msg->ranges.size(); i = i + 1 ){
    geometry_msgs::Point32 curr;
    curr.x = msg->ranges[i] * cos(msg->angle_increment);
    curr.y = msg->ranges[i] * sin(msg->angle_increment);
    curr.z = 0;
    pc.points[i]=curr;
  }
  publish_point_cloud.publish(pc);
  ros::spinOnce();
}

bool ProviderTestService(
    compsci403_assignment1::Assignment1ProviderSrv::Request& req,
    compsci403_assignment1::Assignment1ProviderSrv::Response& res){
  ROS_INFO("Received:%ld + %ld", (long int)req.x, (long int)req.y);
  res.result = req.x+req.y;
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "assignment1");
  ros::NodeHandle n;
  // Perform operations defined in Assignment 1
  ros::Rate loop(10);

  publish_test = n.advertise<std_msgs::String>("/COMPSCI403/PublishTest", 1000);
  laser_scan_sub = n.subscribe("/COMPSCI403/LaserScan", 1000, laserScanCallback);
  client = n.serviceClient<compsci403_assignment1::Assignment1CallerSrv>("/COMPSCI403/CallerTest");
  provider_service = n.advertiseService("/COMPSCI403/ProviderTest", ProviderTestService);
  publish_point_cloud = n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/PointCloud", 1000);
 
  compsci403_assignment1::Assignment1CallerSrv srv;
  while (ros::ok()) {
    if(client.call(srv)){
      std_msgs::String msg;
      std::string str = srv.response.str;
      std::string trunc_str;
      if(str.length()>8){
	trunc_str = str.substr(0,8);
      } else {
	trunc_str = str.substr(0,str.length());
      }
      msg.data = trunc_str;
      publish_test.publish(msg);
    }
    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}
