#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include "cobot_msgs/CobotDriveMsg.h"
#include "compsci403_assignment4/ObstacleMsg.h"
#include "compsci403_assignment4/CheckPointSrv.h"
#include "compsci403_assignment4/GetFreePathSrv.h"
#include "compsci403_assignment4/GetCommandVelSrv.h"

// Include any additional header or service/message files

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>

using geometry_msgs::Point32;
using Eigen::Matrix3f;
using Eigen::Vector3f;
using std::cout;

// Declare class variables, subscribers, publishers, messages

ros::ServiceServer checkPointSrv;
ros::ServiceServer getFreePathSrv;
ros::Subscriber cobotLaserSub;
ros::Publisher publishObstacle;
ros::ServiceServer getCommandVelSrv;
ros::Publisher publishCobotDrive;

sensor_msgs::LaserScan laserScan;

float pi = 3.14159265358979323846;
float gAlpha;
float gBeta;
float gGamma;

// Define service and callback functions
float detectObstacle(Point32 P, float v, float w){
  float radius = .18;
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

std::vector<Point32> convertLaserScan(const sensor_msgs::LaserScan& laser_scan){
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
    Matrix3f R;
    R(0,0) = 1;
    R(0,1) = 0;
    R(0,2) = 0;
    R(1,0) = 0;
    R(1,1) = 1;
    R(1,2) = 0;
    R(2,0) = 0;
    R(2,1) = 0;
    R(2,2) = 1;
    Vector3f t = Vector3f(.145, 0, .23);
    Vector3f tv = R*v + t;
    Point32 result;
    result.x = tv.x();
    result.y = tv.y();
    result.z = tv.z();
    points.push_back(result);
    currAngle += laser_scan.angle_increment;
  }
  return points;
}

std::pair<float, float> commandVelHelper(float v_0, float w_0, float angle, std::vector<Point32> points){
  float deltaT = .05;
  //vmax = .5, wmax = 1.5
  //vaccelmax = .5, waccelmax = 2
  float vMax = .5;
  float vMin = 0;
  float wMax = 1.5;
  float wMin = -1.5;
  float aVMax = .5;
  float aWMax = 2;
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
    for(int j = 0; j <80; j++){
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
	float fpl = detectObstacle(P,  currV, currW);
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
	  shortestFPL = 10; //If nan, then no obstacles
	  //Assign it some positive value such that it is a priority
	}
	//float alpha = 10; //Positive alpha to prioritize angles close to 0.
	//float beta = -5; //Negative beta to prioritize staying away from obstacles (maximizing shortest FPL)
	//float gamma = -5; //Negative gamma to prioritize high velocity
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

void CobotLaserCallback(const sensor_msgs::LaserScan& laser_scan) {
  std::vector<Point32> points = convertLaserScan(laser_scan);
  compsci403_assignment4::ObstacleMsg om;
  om.header = laser_scan.header;
  om.obstacle_points = points;
  laserScan = laser_scan; //assign global var
  publishObstacle.publish(om);
  ros::spinOnce();
}

bool CheckPoint(
    compsci403_assignment4::CheckPointSrv::Request& req,
    compsci403_assignment4::CheckPointSrv::Response& res){
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
    compsci403_assignment4::GetFreePathSrv::Request& req,
    compsci403_assignment4::GetFreePathSrv::Response& res){
  //req.v, req.w, req.laser_scan
  float currAngle = req.laser_scan.angle_min;
  float shortestFPL = nan("");
  for(unsigned int i = 0; i < req.laser_scan.ranges.size(); ++i ){
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

bool GetCommandVel(
    compsci403_assignment4::GetCommandVelSrv::Request& req,
    compsci403_assignment4::GetCommandVelSrv::Response& res){
  std::vector<Point32> points = convertLaserScan(laserScan); //Convert laser scan to points and pass to commandVelHelper
  std::pair<float, float> vw = commandVelHelper(req.v_0, req.w_0, 0, points);
  res.C_v = vw.first;
  res.C_w = vw.second;
  return true;
}

int main(int argc, char **argv) {
  if (argc != 4){
    std::cout << "Invalid input" << std::endl;
    return 0;
  }
  gAlpha = atof(argv[1]);
  gBeta = atof(argv[2]);
  gGamma = atof(argv[3]);
  std::cout << gAlpha << std::endl;
  std::cout << gBeta << std::endl;
  std::cout << gGamma << std::endl;
  ros::init(argc, argv, "assignment4");
  ros::NodeHandle n;

  // Perform operations defined in Assignment 4

  checkPointSrv = n.advertiseService("/COMPSCI403/CheckPoint", CheckPoint);
  getFreePathSrv = n.advertiseService("/COMPSCI403/GetFreePath", GetFreePath);
  cobotLaserSub = n.subscribe("/Cobot/Laser", 1000, CobotLaserCallback);
  publishObstacle = n.advertise<compsci403_assignment4::ObstacleMsg>("/COMPSCI403/Obstacles", 1000);
  getCommandVelSrv = n.advertiseService("/COMPSCI403/GetCommandVel", GetCommandVel);

  //Part 5
  publishCobotDrive = n.advertise<cobot_msgs::CobotDriveMsg>("/Cobot/Drive", 1000);
  ros::Rate loop_rate(20);
  //Assume initial velocities. Afterwards just update velocities with what values we get from part 4.
  float v = 0;
  float w = 0;
  float angle = 0;
  while (ros::ok()){
    sensor_msgs::LaserScan currLaserScan = laserScan;
    std::vector<Point32> points = convertLaserScan(currLaserScan); //Convert laser scan to points and pass to commandVelHelper
    std::pair<float, float> vw = commandVelHelper(v,w,angle,points);
    v = vw.first;
    w = vw.second;
    //Update angle accordingly, angle = angle+delta theta % pi
    float modAngle = angle+w*.05;
    angle = fmod(modAngle, pi);
    cobot_msgs::CobotDriveMsg cm;
    cm.header = currLaserScan.header;
    cm.v = v;
    cm.w = w;
    publishCobotDrive.publish(cm);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return(0);
}
