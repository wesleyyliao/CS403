#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>

#include "compsci403_assignment4/ObstacleMsg.h"
#include "compsci403_assignment4/CheckPointSrv.h"
#include "compsci403_assignment4/GetFreePathSrv.h"
#include "compsci403_assignment4/GetCommandVelSrv.h"

// Include any additional header or service/message files

using Eigen::Matrix3f;
using Eigen::Vector3f;
using geometry_msgs::Point32;


int main(int argc, char **argv) {
  ros::init(argc, argv, "testClient");
  ros::NodeHandle n;
  
  compsci403_assignment4::GetFreePathSrv srv;
  ros::ServiceClient inliersClient = n.serviceClient<compsci403_assignment4::GetFreePathSrv>("/COMPSCI403/GetFreePath");
  ros::Rate loop(10);
  srv.request.v = 5;
  srv.request.w = 4;
  ROS_INFO("Starting");
  while (ros::ok()) {
    if(inliersClient.call(srv)){
      ROS_INFO("CALLING");
      bool isObstacle = srv.response.is_obstacle;
      float fpl = srv.response.free_path_length;
      std::cout << isObstacle << std::endl;
      std::cout << fpl << std::endl;
      /*std::vector<Point32> resultPoints = srv.response.P;
      ROS_INFO("result points = %f %f %f", resultPoints.front().x, resultPoints.front().y, resultPoints.front().z);
      ROS_INFO("result points = %f %f %f", resultPoints.at(1).x, resultPoints.at(1).y, resultPoints.at(1).z);
      ROS_INFO("result points = %f %f %f", resultPoints.at(2).x, resultPoints.at(2).y, resultPoints.at(2).z);*/
      break;
    }
    ros::spinOnce();
    loop.sleep();
  }
  ros::spin();

  return 0;
}
