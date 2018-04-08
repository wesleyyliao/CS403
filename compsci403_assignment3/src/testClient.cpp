#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>

#include "compsci403_assignment3/TransformPointSrv.h"
#include "compsci403_assignment3/FitMinimalPlaneSrv.h"
#include "compsci403_assignment3/FindInliersSrv.h"
#include "compsci403_assignment3/FitBestPlaneSrv.h"
#include "compsci403_assignment3/PlaneParametersMsg.h"

// Include any additional header or service/message files

using Eigen::Matrix3f;
using Eigen::Vector3f;
using geometry_msgs::Point32;


int main(int argc, char **argv) {
  ros::init(argc, argv, "testClient");
  ros::NodeHandle n;
  
  compsci403_assignment3::FitBestPlaneSrv srv;
  ros::ServiceClient inliersClient = n.serviceClient<compsci403_assignment3::FitBestPlaneSrv>("/COMPSCI403/FitBestPlane");
  Point32 p0;
  Point32 testPoint;
  testPoint.x = 4;
  testPoint.y = -4;
  testPoint.z = 3;
  Point32 testPoint2;
  testPoint2.x = 2;
  testPoint2.y = -1;
  testPoint2.z = 0;
  
  Point32 testPoint3;
  testPoint3.x = 10;
  testPoint3.y = -5;
  testPoint3.z = 5;
  p0.x = -2;
  p0.y = 2;
  p0.z = 0;
  Point32 pn;
  pn.x = 2;
  pn.y = -2;
  pn.z = 5;
  //srv.request.epsilon = 10;
  //srv.request.P0 = p0;
  //srv.request.n = pn;
  ros::Rate loop(10);
  std::vector<Point32> testPoints;
  testPoints.push_back(testPoint);
  testPoints.push_back(testPoint2);
  testPoints.push_back(testPoint3);
  srv.request.P = testPoints;
  ROS_INFO("Starting");
  while (ros::ok()) {
    if(inliersClient.call(srv)){
      ROS_INFO("CALLING");
      Point32 n = srv.response.n;
      Point32 P0 = srv.response.P0;
      ROS_INFO("N = %f %f %f", n.x, n.y, n.z);
      ROS_INFO("P0 = %f %f %f", P0.x, P0.y, P0.z);
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
