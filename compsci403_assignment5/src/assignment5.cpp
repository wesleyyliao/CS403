#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <math.h>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include "compsci403_assignment5/CheckPointSrv.h"
#include "compsci403_assignment5/GetFreePathSrv.h"
#include "compsci403_assignment5/ObstacleLaserScanSrv.h"
#include "compsci403_assignment5/GetCommandVelSrv.h"
#include "compsci403_assignment5/GetTransformationSrv.h"

using Eigen::Matrix3f;
using Eigen::MatrixXf;
using Eigen::MatrixXd;
using Eigen::Vector3f;
using Eigen::Vector2f;
using geometry_msgs::Point32;
using geometry_msgs::Point;
using geometry_msgs::Twist;
using nav_msgs::Odometry;
using visualization_msgs::Marker;
using sensor_msgs::LaserScan;
using sensor_msgs::PointCloud;
using std::cout;
using std::vector;
using geometry_msgs::Vector3;
using namespace std;

ros::ServiceServer checkPointSrv;
ros::ServiceServer getFreePathSrv;
ros::ServiceServer obstacleLaserScanSrv;
ros::ServiceServer  getCommandVelSrv;
ros::Subscriber laserScanSub;
ros::Subscriber odomSub;
ros::Publisher publishDrive;

// Global Parameters
const float gRobotRadius = 0.18;
const float gRobotHeight = 0.36;
const float gEpsilon = 0.15;
const float gAMaxLin = 1 * 0.5; // m/s^2
const float gAMaxRot = 1 * 2.0; // rad/s^2
const float gVMaxLin = 0.5; // m/s
const float gVMaxRot = 1.5; // rad/s
const float gDT = 0.05; // s

//std::vector<float> gR;
boost::array<float,9> gR;
Point32 gT;
float gV;
float gW;
sensor_msgs::LaserScan gLaserScan;
float gAlpha;
float gBeta;
float gGamma;
float gNoObstFPL;
const float pi = 3.14159265358979323846;

float detectObstacle(Point32 P, float v, float w){
  float radius = gRobotRadius;
  float fpl = nan("");
  Eigen::Vector2f p (P.x, P.y);
  if(w == 0){
    if (fabs(p.y()) <= radius && std::signbit(p.x()) == std::signbit(v)){
      //obstacle
      fpl = p.x() - radius;
    }
  } else {
    Eigen::Vector2f c (0,(float) v / w);
    float r = c.norm();
    float distPoint = fabs((c - p).norm() - r);
    if(distPoint < radius){
      //In path so obstacle
      //Vector CP = P-C
      Eigen::Vector2f cp = p-c;
      //Vector CO = 0-C
      Eigen::Vector2f co = -c;
      float cpNorm = cp.norm();
      float coNorm = co.norm();
      float pco = acos(cp.dot(co)/(cpNorm*coNorm));
      float pcl = atan(radius / r);
      float lco = std::abs(pco - pcl); // free path angle
      //free path length = arc length = aLCO * r
      fpl = lco * r;
    }
  }
  return fpl;
}

bool CheckPoint(
    compsci403_assignment5::CheckPointSrv::Request& req,
    compsci403_assignment5::CheckPointSrv::Response& res){
  float fpl = detectObstacle(req.P, req.v, req.w);
  res.free_path_length = fpl;
  if(isnan(fpl)){
    res.is_obstacle = false;
  } else {
    res.is_obstacle = true;
  }
  return true;
}

bool GetFreePath(
    compsci403_assignment5::GetFreePathSrv::Request& req,
    compsci403_assignment5::GetFreePathSrv::Response& res){
  //req.v, req.w, req.laser_scan
  float currAngle = req.laser_scan.angle_min;
  float shortestFPL = nan("");
  for(unsigned int i = 0; i < req.laser_scan.ranges.size(); i = i + 1 ){
    geometry_msgs::Point32 P;
    float range = req.laser_scan.ranges[i];
    if(range > req.laser_scan.range_max || range < req.laser_scan.range_min){
      currAngle += req.laser_scan.angle_increment;
      continue;
    }
    P.x = range * cos(currAngle);
    P.y = range * sin(currAngle);
    P.z = 0;
    float fpl = detectObstacle(P, req.v, req.w);
    if(!isnan(fpl)){
      if(isnan(shortestFPL)){
	shortestFPL = fpl;
      } else if(fpl < shortestFPL){
	shortestFPL = fpl;
      }
    }
    currAngle += req.laser_scan.angle_increment;
  }
  if(isnan(shortestFPL)){
    res.is_obstacle = false;
  } else {
    res.is_obstacle = true;
  }
  res.free_path_length = shortestFPL;
  return true;
}


std::pair<float, float> commandVelHelper(float v_0, float w_0, float angle, std::vector<Point32> points){
  float deltaT = gDT; //.05?
  //vmax = .5, wmax = 1.5
  //vaccelmax = .5, waccelmax = 2
  float vMax = gVMaxLin;
  float vMin = 0;
  float wMax = gVMaxRot;
  float wMin = -1 * gVMaxRot;
  float aVMax = gAMaxLin;
  float aWMax = gAMaxRot;
  
  float dynVMax = v_0 + aVMax * deltaT;
  float dynVMin = v_0 - aVMax * deltaT;
  float dynWMax = w_0 + aWMax * deltaT;
  float dynWMin = w_0 - aWMax * deltaT;
  float currentAngle = angle; 
  //dynamic window bounded by v [0, .5] w [-1.5, 1.5]
  if(dynVMax > vMax){
    dynVMax = vMax;
  }
  if(dynVMin < vMin){
    dynVMin = vMin;
  }
  if(dynWMax > wMax){
    dynWMax = wMax;
  }
  if(dynWMin < wMin){
    dynWMin = wMin;
  }
  //Need to iterate over dynamic window in some number of steps [hardcoded value]
  //Divide dynamic window into cells and for each cell, do computation
  float vIt = (dynVMax - dynVMin) / 80;
  float wIt = (dynWMax - dynWMin) / 80;
  float bestCost = nan("");
  float bestV = 0;
  float bestW = 0;
  for(int i = 0; i < 80; i++){
    for(int j = 0; j < 80; j++){
      float currV = vIt * i + dynVMin;
      float currW = wIt * j + dynWMin;
      //Bound by maxes (may be unnecessary, but just in case)
      if(currV > vMax){
	currV = vMax;
      }
      if(currW > wMax){
	currW = wMax;
      }
      //For each velocity and w, compute the smallest fpl
      float shortestFPL = nan("");
      for(std::vector<Point32>::iterator it = points.begin(); it!=points.end(); ++it){
	Point32 P = *it;
	//For each point in laserscan, detect obstacle. If obstacle, geting stopping distance
	float fpl = detectObstacle(P, currV, currW);
	if(!isnan(fpl)){
	  if(isnan(shortestFPL)){
	    shortestFPL = fpl;
	  } else if(fpl < shortestFPL){
	    shortestFPL = fpl;
	  }
	}
      }
      //compute stopping distance
      //vcritical = sqrt(2*a*s), a = max accel
      //wcritical = sqrt(2*a*deltaangle) a = max angular accel
      float vCriticalSq = nan("");
      if(!isnan(shortestFPL)) {
	vCriticalSq = (float)(2*aVMax*shortestFPL);
      }
      float deltaTheta = std::abs(currW * deltaT);
      float wCriticalSq = (float)(2*aWMax*deltaTheta);
      //If shortestFPL is nan, then stopping distance is irrelevant
      //If (there is no shortestFPL) or (v and w  < v crit and w crit)
      if((isnan(shortestFPL)) || (((currV*currV) < vCriticalSq) && ((currW*currW) < wCriticalSq))){
	//admissable, compute cost function
	if(isnan(shortestFPL)){
	  shortestFPL = gNoObstFPL; //If nan, then no obstacles
	  //Assign it some positive value such that it is a priority
	}
	float modAngle = currentAngle+currW*deltaT;
	float finalAngle = fmod(modAngle, pi);
	//abs value the angle (so that negative angles aren't favored)
	float cost = gAlpha*std::abs(finalAngle) + gBeta*shortestFPL + gGamma*currV;
	//update best cost and velocities
	if(isnan(bestCost)){
	  bestCost = cost;
	  bestV = currV;
	  bestW = currW;
	} else if(cost < bestCost){
	  bestCost = cost;
	  bestV = currV;
	  bestW = currW;
	}
      }
    }
  }
  std::pair<float, float> vw;
  vw.first = bestV;
  vw.second = bestW;
  return vw;
}


std::vector<Point32> convertLaserScan(const sensor_msgs::LaserScan& laser_scan, boost::array<float, 9> R, Point32 T){
  float currAngle = laser_scan.angle_min;
  std::vector<Point32> points;
  for(unsigned int i = 0; i < laser_scan.ranges.size(); ++i ){
    Point32 P;
    float range = laser_scan.ranges[i];
    if(range > laser_scan.range_max || range < laser_scan.range_min){
      currAngle += laser_scan.angle_increment;
      continue;
    }
    P.x = range * std::cos(currAngle);
    P.y = range * std::sin(currAngle);
    P.z = 0;
    Vector3f v = Vector3f(P.x, P.y, P.z);
    Matrix3f RM;
    RM(0,0) = R[0];
    RM(0,1) = R[1];
    RM(0,2) = R[2];
    RM(1,0) = R[3];
    RM(1,1) = R[4];
    RM(1,2) = R[5];
    RM(2,0) = R[6];
    RM(2,1) = R[7];
    RM(2,2) = R[8];
    Vector3f t = Vector3f(T.x, T.y, T.z);
    Vector3f tv = RM*v + t;
    Point32 result;
    result.x = tv.x();
    result.y = tv.y();
    result.z = tv.z();
    points.push_back(result);
    currAngle += laser_scan.angle_increment;
  }
  return points;
}

bool ObstacleLaserScan(
    compsci403_assignment5::ObstacleLaserScanSrv::Request& req,
    compsci403_assignment5::ObstacleLaserScanSrv::Response& res){
  Point32 t = req.T;
  std::vector<Point32> points = convertLaserScan(req.S, req.R, t);
  res.S_prime = points;
  return true;
}

bool GetCommandVel(
    compsci403_assignment5::GetCommandVelSrv::Request& req,
    compsci403_assignment5::GetCommandVelSrv::Response& res){
  std::vector<Point32> points = convertLaserScan(gLaserScan, gR, gT); //Convert laser scan to points and pass to commandVelHelper
  std::pair<float, float> vw = commandVelHelper(req.v_0, req.w_0, 0, points);
  res.C_v = vw.first;
  res.C_w = vw.second;
  return true;
}

void LaserScanCallback(const sensor_msgs::LaserScan& laser_scan) {
  gLaserScan = laser_scan; //assign global var
}
void OdomCallback(const nav_msgs::Odometry& odom) {
  //Point pos = odom.pose.pose.position;
  Vector3 lVel = odom.twist.twist.linear;
  Vector3 aVel = odom.twist.twist.angular;
  gV = sqrt(pow(lVel.x, 2.0f)+pow(lVel.y, 2.0f));
  gW = aVel.z;
}

int main(int argc, char **argv) {
  if (argc != 5){
    std::cout << "Invalid input: ( try 30, -5, -20, 10 )" << std::endl;
    return 0;
  }
  gAlpha = atof(argv[1]);//30
  gBeta = atof(argv[2]);//-5
  gGamma = atof(argv[3]);//-20
  gNoObstFPL = atof(argv[4]);//10
  
  ros::init(argc, argv, "compsci403_assignment5");
  ros::NodeHandle n;
  
  ros::ServiceClient transformationClient = n.serviceClient<compsci403_assignment5::GetTransformationSrv>("/COMPSCI403/GetTransformation");
  compsci403_assignment5::GetTransformationSrv srv;
  if(transformationClient.call(srv)){
    gR = srv.response.R;
    gT = srv.response.T;
  }

  checkPointSrv = n.advertiseService("/COMPSCI403/CheckPoint", CheckPoint);
  getFreePathSrv = n.advertiseService("/COMPSCI403/GetFreePath", GetFreePath);
  obstacleLaserScanSrv = n.advertiseService("/COMPSCI403/ObstacleLaserScan", ObstacleLaserScan);
  getCommandVelSrv = n.advertiseService("/COMPSCI403/GetCommandVel", GetCommandVel);
  //part 5
  laserScanSub = n.subscribe("/COMPSCI403/LaserScan", 1000, LaserScanCallback);
  odomSub = n.subscribe("/odom", 1000, OdomCallback);
  publishDrive = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);
  ros::Rate loop_rate(20);
  //Assume initial velocities. Afterwards just update velocities with what values we get from part 4.
  float v = gV; // probably 0 
  float w = gW; // probably 0
  float angle = 0;
  while (ros::ok()){  
    if(transformationClient.call(srv)){
      gR = srv.response.R;
      gT = srv.response.T;
    } 
    sensor_msgs::LaserScan currLaserScan = gLaserScan;
    std::vector<Point32> points = convertLaserScan(currLaserScan, gR, gT); //Convert laser scan to points and pass to commandVelHelper
    v = gV; //current vel of robot
    w = gW; //current vel of robot
    //gV and gW get updated by odom
    std::pair<float, float> vw = commandVelHelper(v,w,angle,points);
    v = vw.first;
    w = vw.second;
    //Update angle accordingly, angle = angle+delta theta % pi
    float modAngle = angle+w*gDT; // .05?
    angle = fmod(modAngle, pi);
    geometry_msgs::Twist twist;
    Vector3 currLinear;
    Vector3 currAngular;
    currLinear.x = v;
    currLinear.y = 0;
    currLinear.z = 0;
    currAngular.x = 0;
    currAngular.y = 0;
    currAngular.z = w;
    twist.linear = currLinear;
    twist.angular = currAngular;
    publishDrive.publish(twist);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
