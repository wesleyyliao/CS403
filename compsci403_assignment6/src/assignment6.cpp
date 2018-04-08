#include <algorithm>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include "compsci403_assignment6/RandomConfigSrv.h"
#include "compsci403_assignment6/ExtendNodeSrv.h"
#include "compsci403_assignment6/CheckExtensionSrv.h"
#include "compsci403_assignment6/BuildRRTSrv.h"
#include "compsci403_assignment6/RRTPlanSrv.h"

using compsci403_assignment6::RandomConfigSrv;
using compsci403_assignment6::ExtendNodeSrv;
using compsci403_assignment6::CheckExtensionSrv;
using compsci403_assignment6::BuildRRTSrv;
using compsci403_assignment6::RRTPlanSrv;
using compsci403_assignment6::RRTNode;

using Eigen::Vector2f;
using geometry_msgs::Point;
using std::fabs;
using std::max;
using std::atan2;
using std::cout;
using std::vector;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

// Publisher for marker messages.
ros::Publisher markers_publisher_;

// Markers for visualization.
Marker vertices_marker_;
Marker qrand_marker_;
Marker edges_marker_;
Marker map_marker_;
Marker plan_marker_;
//MAGIC NUMBERS
const float goal_bias = .05f;
const float goal_epsilon = .5f;
const float float_epsilon = .00001f;
const float dist_epsilon = .1f; //maybe try other values
const float delta_q = .5f; //gonna need to try other values
const float K = 100000;
Point goal;
Point start;
vector<Vector2f> obstacles_start; //start coords of obstacles
vector<Vector2f> obstacles_end; //end coords of obstacles (creates a line seg with start)
vector<Vector2f> previous_samples; //previously sampled points
// Return a random value between min and max.
float RandomValue(const float min, const float max) {
  const float scale = max - min;
  const float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  return (min + r * scale);
}

// Helper function to convert a 2D vector into a ros 3D point.
Point VectorToPoint(const Vector2f& v) {
  Point point;
  point.x = v.x();
  point.y = v.y();
  point.z = 0;
  return point;
}

// Helper function to visualize a point.
void DrawPoint(const Vector2f& p, Marker* marker) {
  marker->points.push_back(VectorToPoint(p));
}

// Helper function to visualize an edge.
void DrawLine(const Vector2f& p1, const Vector2f& p2, Marker* marker) {
  marker->points.push_back(VectorToPoint(p1));
  marker->points.push_back(VectorToPoint(p2));
}

// Initialize all markers.
void InitMarkers() {
  vertices_marker_.header.frame_id = "map";
  vertices_marker_.id = 1;
  vertices_marker_.type = Marker::POINTS;
  vertices_marker_.action = Marker::MODIFY;
  vertices_marker_.scale.x = 0.2;
  vertices_marker_.scale.y = 0.2;
  vertices_marker_.color.a = 1.0;
  vertices_marker_.color.r = 0.0;
  vertices_marker_.color.g = 0.0;
  vertices_marker_.color.b = 1.0;


  qrand_marker_.header.frame_id = "map";
  qrand_marker_.id = 2;
  qrand_marker_.type = Marker::POINTS;
  qrand_marker_.action = Marker::MODIFY;
  qrand_marker_.scale.x = 0.2;
  qrand_marker_.scale.y = 0.2;
  qrand_marker_.color.a = 1.0;
  qrand_marker_.color.r = 1.0;
  qrand_marker_.color.g = 0.0;
  qrand_marker_.color.b = 0.0;

  edges_marker_.header.frame_id = "map";
  edges_marker_.id = 3;
  edges_marker_.type = Marker::LINE_LIST;
  edges_marker_.action = Marker::MODIFY;
  edges_marker_.scale.x = 0.05;
  edges_marker_.scale.y = 0.05;
  edges_marker_.color.a = 1.0;
  edges_marker_.color.r = 0.0;
  edges_marker_.color.g = 1.0;
  edges_marker_.color.b = 0.0;

  map_marker_.header.frame_id = "map";
  map_marker_.id = 4;
  map_marker_.type = Marker::LINE_LIST;
  map_marker_.action = Marker::MODIFY;
  map_marker_.scale.x = 0.05;
  map_marker_.scale.y = 0.05;
  map_marker_.color.a = 1.0;
  map_marker_.color.r = 1.0;
  map_marker_.color.g = 1.0;
  map_marker_.color.b = 1.0;

  plan_marker_.header.frame_id = "map";
  plan_marker_.id = 5;
  plan_marker_.type = Marker::LINE_LIST;
  plan_marker_.action = Marker::MODIFY;
  plan_marker_.scale.x = 0.05;
  plan_marker_.scale.y = 0.05;
  plan_marker_.color.a = 1.0;
  plan_marker_.color.r = 1.0;
  plan_marker_.color.g = 0.0;
  plan_marker_.color.b = 0.0;
}

//Domain of maze is [-4,4] x [-4,4]
//Robot starts at (3, -3.99)
bool isObstacle(Vector2f rand_point_vec){
  vector<Vector2f>::iterator i = obstacles_start.begin();
  vector<Vector2f>::iterator j = obstacles_end.begin();
  while(true){
    Vector2f start = *i;
    Vector2f end = *j;
    //Vector2f cross = (end - start).cross(rand_point_vec - start);
    float cross = (rand_point_vec.y() - start.y()) * (end.x() - start.x()) - (rand_point_vec.x() - start.x()) * (end.y() - start.y());
    //check if cross product is 0, use float_epsilon since floating point numbers and 0s are tricky
    //if(std::abs(cross.x()) < float_epsilon && std::abs(cross.y()) < float_epsilon){
    if(std::abs(cross) <= float_epsilon){
      float dot_product = (end - start).dot(rand_point_vec - start);
      if(dot_product >= 0){
	float squared_length = (end.x() - start.x())*(end.x() - start.x()) + (end.y() - start.y())*(end.y() - start.y());
	if(dot_product <= squared_length){
	  return true;
	}
      }
    }
    if(i != obstacles_start.end()){
      ++i;
    }
    if(j != obstacles_end.end()){
      ++j;
    }
    if(i == obstacles_start.end() && j == obstacles_end.end()){
      break;
    }
  }
  
  /*for(i(obstacles_start.begin()), j(obstacles_end.begin()); i != obstacles_start.end() && j != obstacles_end.end(); ++i, ++j){
    Vector2f start = *i;
    Vector2f end = *j;
    //check if cross product between (end-start) and (point-start) is 0
    //also check if dot product of (end-start) and (point-start) is positive and less than the square of dist between end and start
    Vector2f cross = (end - start).cross(rand_point_vec - start);
    //check if cross product is 0, use float_epsilon since floating point numbers and 0s are tricky
    if(std::abs(cross.x()) < float_epsilon && std::abs(cross.y()) < float_epsilon){
      float dot_product = (end - start).dot(rand_point_vec - start);
      if(dot_product > 0){
	float squared_length = (end.x() - start.x())*(end.x() - start.x()) + (end.y() - start.y())*(end.y() - start.y());
	if(dot_product < squared_length){
	  return true;
	}
      }
    }
    }*/
  return false;
}
bool nearPreviousSamples(Vector2f rand_point){
  for(vector<Vector2f>::iterator it = previous_samples.begin(); it != previous_samples.end(); ++it){
    Vector2f previous_sample = *it;
    float dist = sqrtf((previous_sample.x() - rand_point.x())*(previous_sample.x() - rand_point.x()) + (previous_sample.y() - rand_point.y())*(previous_sample.y() - rand_point.y()));
    if(dist < dist_epsilon){ //too close
      return true;
    }
  }
  return false;
}

Point randomConfigHelper(){
    
  float r = RandomValue(0, 1);

  if(r<goal_bias){
    //return goal
    return goal;
  } else {
    //add a hard limit just in case
    int i = 0;
    while(i < 10000){
      i=i+1;
      //Randomly pick a point.
      Point rand_point;
      float x = RandomValue(-4.0f, 4.0f);
      float y = RandomValue(-4.0f, 4.0f);
      rand_point.x = x;
      rand_point.y = y; 
      rand_point.z = .0f;
      Vector2f rand_point_vec = Vector2f(rand_point.x, rand_point.y);
      //check if point is in walls (between obstacles).
      //iterate through concurrently
      bool is_obstacle = isObstacle(rand_point_vec);
      if(is_obstacle){
	continue;
      } 
      bool near_previous = nearPreviousSamples(rand_point_vec);
      if(near_previous){
	continue;
      }
      //Otherwise, add to global vector and return
      previous_samples.push_back(rand_point_vec);
      return rand_point;
    }
    if(i == 10000){
      cout << "!!! Somehow managed to not find a valid random point !!!" << std::endl;
    }
  }

  return goal; //if we somehow managed to not find a valid pt
}

bool RandomConfigService(
    RandomConfigSrv::Request& req, RandomConfigSrv::Response& res) {
  //At start, get a random number 0 to 1. If less than goal bias, then return goal point as sample
  //otherwise create a loop and find random points (samples) until a valid one is found.

  Point rand_point = randomConfigHelper();
  res.q_rand = rand_point;
  return true;
}

std::pair<int, Point> extendNodeHelper(vector<Point> p, Point q) {
  int closest_index = 0;
  float lowest_dist = nan("");
  for(vector<Point>::iterator it = p.begin(); it != p.end(); ++it){
    int index = std::distance(p.begin(), it);
    Point curr_p = *it;
    //calculate distance between curr_p and q
    float dist = sqrtf((curr_p.x - q.x)*(curr_p.x - q.x) + (curr_p.y - q.y)*(curr_p.y - q.y));
    if(isnan(lowest_dist)){
      closest_index = index;
      lowest_dist = dist;
    } else if (dist < lowest_dist){
      closest_index = index;
      lowest_dist = dist;
    }
  }
  Point qnear = p.at(closest_index);
  Vector2f qnear_vec = Vector2f(qnear.x, qnear.y);
  //extend qnear to q by some distance
  Vector2f v = Vector2f(q.x-qnear_vec.x(), q.y-qnear_vec.y());
  Point qnew;
  float dist = sqrtf((qnear.x - q.x)*(qnear.x - q.x) + (qnear.y - q.y)*(qnear.y - q.y));
  if(v.x() == 0 && v.y() == 0){ //Somehow got a point that is exactly q
    qnew.x = qnear.x;
    qnew.y = qnear.y;
  } else if(dist < delta_q){
    //if q's distance from q_near is < delta_q, just use q's coords.
    qnew.x = q.x;
    qnew.y = q.y;
  } else {
    Vector2f u = v/v.norm();
    //extend in direction by delta_q
    Vector2f qnew_vec = qnear_vec + u*delta_q;
    qnew.x = qnew_vec.x();
    qnew.y = qnew_vec.y();
  }
  std::pair<int, Point> ret;
  ret.first = closest_index;
  ret.second = qnew;
  return ret;
}

//geometry_msgs/Point[] P
//geometry_msgs/Point q
bool ExtendNodeService(
    ExtendNodeSrv::Request& req, ExtendNodeSrv::Response& res) {

  std::pair<int, Point> nodePair = extendNodeHelper(req.P,req.q);
  res.q_near_index = nodePair.first;
  res.q_new = nodePair.second;
  return true;
}

bool checkExtensionHelper(Point qnear, Point qnew){
  Vector2f qnear_vec = Vector2f(qnear.x, qnear.y);
  Vector2f qnew_vec = Vector2f(qnew.x, qnew.y);
  Vector2f q_norm = Vector2f(-(qnew_vec.y()-qnear_vec.y()), qnew_vec.x()-qnear_vec.x());
  //normalize q_norm?
  if(!(q_norm.x() == 0 && q_norm.y() == 0)){
    q_norm = q_norm/q_norm.norm();
  }
  vector<Vector2f>::iterator i = obstacles_start.begin();
  vector<Vector2f>::iterator j = obstacles_end.begin();
  while(true){
    Vector2f start = *i;
    Vector2f end = *j;
    Vector2f obstacle_norm = Vector2f(-(end.y()-start.y()), end.x()-start.x());
    //normalize obstacle_norm?
    if(!(obstacle_norm.x() == 0 && obstacle_norm.y() == 0)){
      obstacle_norm = obstacle_norm/obstacle_norm.norm();
    }
    // cross or dot?
    if(((start - qnear_vec).dot(q_norm)) * ((end - qnear_vec).dot(q_norm)) <= 0){
      if(((qnear_vec - start).dot(obstacle_norm)) * ((qnew_vec - start).dot(obstacle_norm)) <= 0){
	//thus intersection
	return false;
      }
    }
    if(i != obstacles_start.end()){
      ++i;
    }
    if(j != obstacles_end.end()){
      ++j;
    }
    if(i == obstacles_start.end() && j == obstacles_end.end()){
      break;
    }
  }
  return true;
}

bool CheckExtensionService(
    CheckExtensionSrv::Request& req, CheckExtensionSrv::Response& res) {
  res.valid = checkExtensionHelper(req.q_near, req.q_new);
  return true;
}

vector<RRTNode> buildRRTHelper(){
  vector<Vector2f>::iterator i = obstacles_start.begin();
  vector<Vector2f>::iterator j = obstacles_end.begin();
  while(true){
    Vector2f start = *i;
    Vector2f end = *j;
    DrawLine(start, end, &map_marker_);
    //DrawLine(start, end, &plan_marker_);
    if(i != obstacles_start.end()){
      ++i;
    }
    if(j != obstacles_end.end()){
      ++j;
    }
    if(i == obstacles_start.end() && j == obstacles_end.end()){
      break;
    }
  }
  vector<RRTNode> G;
  vector<Point> G_vec;
  RRTNode qinit;
  qinit.location = start;
  qinit.parent = -1;
  G.push_back(qinit);
  G_vec.push_back(qinit.location);
  int n;
  for(n = 0; n < K; ++n){
    //get qrand
    Point q_rand = randomConfigHelper();
    Vector2f q_rand_vec = Vector2f(q_rand.x, q_rand.y);
    DrawPoint(q_rand_vec, &qrand_marker_);
    std::pair<int, Point> nodePair = extendNodeHelper(G_vec,q_rand);
    int q_near_index = nodePair.first;
    Point q_new = nodePair.second;
    bool validExtension = checkExtensionHelper(G_vec.at(q_near_index), q_new);
    if(validExtension){
      RRTNode newNode;
      newNode.parent = q_near_index;
      newNode.location = q_new;
      G.push_back(newNode);
      G_vec.push_back(q_new);
      //distance between new and goal
      float dist = sqrtf((q_new.x - goal.x)*(q_new.x - goal.x) + (q_new.y - goal.y)*(q_new.y - goal.y));
      if(dist < goal_epsilon){
	//done
	//add to g vec the goal?
	RRTNode goalRRT;
	goalRRT.parent = G.size()-1;
	goalRRT.location = goal;
	G.push_back(goalRRT);
	G_vec.push_back(goal);
	break;
      }
    }
  }

  if(n == K){
    cout << "Did not reach goal within K iterations " << std::endl;
  }
  RRTNode prev_node;
  for(std::vector<RRTNode>::iterator it = G.begin(); it != G.end(); ++it){
    int index = std::distance(G.begin(), it);
    RRTNode curr_node = *it;
    Vector2f curr_node_vec = Vector2f(curr_node.location.x, curr_node.location.y);
    if(curr_node.parent!=-1){
      //Draw line from curr to its parent
      RRTNode parent_node = G.at(curr_node.parent);
      Vector2f parent_node_vec = Vector2f(parent_node.location.x, parent_node.location.y);
      DrawLine(parent_node_vec, curr_node_vec, &edges_marker_);
    }
  }
  return G;
}

bool BuildRRTService(
    BuildRRTSrv::Request& req, BuildRRTSrv::Response& res) {

  // Sample code to visualize the RRT
  MarkerArray markers;
  markers.markers.clear();
  // Draw maze:
  // for (size_t i = 0; i < map.size(); ++i) {
  //   DrawLine(map[i].p1, map[i].p2, &map_marker_);
  //   DrawLine(map[i].p1, map[i].p2, &plan_marker_);
  // }
  //
  // Draw q_rand:
  // DrawPoint(q_rand, &qrand_marker_);
  //
  // Draw the tree.
  // ...
  //
  // Publish all the markers.
  
  //draw maze
  vector<RRTNode> G = buildRRTHelper();
  markers.markers.push_back(vertices_marker_);
  markers.markers.push_back(qrand_marker_);
  markers.markers.push_back(edges_marker_);
  markers.markers.push_back(map_marker_);
  markers.markers.push_back(plan_marker_);
  markers_publisher_.publish(markers);
  std::reverse(G.begin(),G.end());
  //because we had to reverse the list, all the parent locations are now inverted
  vector<RRTNode> reversedG;
  for(std::vector<RRTNode>::iterator it = G.begin(); it != G.end(); ++it){
    RRTNode curr_node = *it;
    if(curr_node.parent != -1){
      curr_node.parent = (G.size()-1) - curr_node.parent;
    }
    reversedG.push_back(curr_node);
  }
  res.rrt = reversedG;
  return true;
}

bool RRTPlanService(
    RRTPlanSrv::Request& req, RRTPlanSrv::Response& res) {
  MarkerArray markers;
  markers.markers.clear();
  //markers.markers.push_back(vertices_marker_);
  //markers.markers.push_back(qrand_marker_);
  //markers.markers.push_back(edges_marker_);
  vector<RRTNode> G = buildRRTHelper();
  std::reverse(G.begin(),G.end());
  //because we had to reverse the list, all the parent locations are now inverted
  vector<RRTNode> reversedG;
  for(std::vector<RRTNode>::iterator it = G.begin(); it != G.end(); ++it){
    RRTNode curr_node = *it;
    if(curr_node.parent != -1){
      curr_node.parent = (G.size()-1) - curr_node.parent;
    }
    reversedG.push_back(curr_node);
  }
  
  vector<Point> path;
  int parentIndex = 0;
  RRTNode prev_node;
  int count = 0;
  while(parentIndex!=-1){
    RRTNode curr_node = reversedG.at(parentIndex);
    path.push_back(curr_node.location);
    parentIndex = curr_node.parent;
    if(count > 0){
      Vector2f prev_vec = Vector2f(prev_node.location.x, prev_node.location.y);
      Vector2f curr_vec = Vector2f(curr_node.location.x, curr_node.location.y);
      DrawLine(prev_vec, curr_vec, &plan_marker_);
    }
    prev_node = curr_node;
    count+=1;
  }
  res.path = path;

  markers.markers.push_back(vertices_marker_);
  markers.markers.push_back(qrand_marker_);
  markers.markers.push_back(edges_marker_);
  markers.markers.push_back(map_marker_);
  markers.markers.push_back(plan_marker_);
  markers_publisher_.publish(markers);
  return true;
}

bool RRTPlanBonusService(
    RRTPlanSrv::Request& req, RRTPlanSrv::Response& res) {
  return true;
}

void initializeObstacles(){
  Vector2f point;
  point = Vector2f(2,-2);
  obstacles_start.push_back(point);
  point = Vector2f(0,-2);
  obstacles_start.push_back(point);
  point = Vector2f(-2,-2);
  obstacles_start.push_back(point);
  point = Vector2f(-4,0);
  obstacles_start.push_back(point);
  point = Vector2f(-4,2);
  obstacles_start.push_back(point);
  point = Vector2f(2,2);
  obstacles_start.push_back(point);
  point = Vector2f(-2,4);
  obstacles_start.push_back(point);
  point = Vector2f(-4,-4);
  obstacles_start.push_back(point);
  point = Vector2f(4,-4);
  obstacles_start.push_back(point);
  point = Vector2f(-4,-4);
  obstacles_start.push_back(point);

  point = Vector2f(4,-2);
  obstacles_end.push_back(point);
  point = Vector2f(0,0);
  obstacles_end.push_back(point);
  point = Vector2f(-2,0);
  obstacles_end.push_back(point);
  point = Vector2f(2,0);
  obstacles_end.push_back(point);
  point = Vector2f(0,2);
  obstacles_end.push_back(point);
  point = Vector2f(4,2);
  obstacles_end.push_back(point);
  point = Vector2f(4,4);
  obstacles_end.push_back(point);
  point = Vector2f(-4,4);
  obstacles_end.push_back(point);
  point = Vector2f(4,4);
  obstacles_end.push_back(point);
  point = Vector2f(2,-4);
  obstacles_end.push_back(point);
}

int main(int argc, char **argv) {
  InitMarkers();

  ros::init(argc, argv, "compsci403_assignment6");
  ros::NodeHandle n;

  markers_publisher_ = 
  	n.advertise<visualization_msgs::MarkerArray>("/COMPSCI403/RRT_Display", 10);

  //initialize global variables
  goal.x = -3;
  goal.y = 3.99;
  start.x = 3;
  start.y = -3.99;
  initializeObstacles();
  
  ros::ServiceServer server1 = n.advertiseService(
      "COMPSCI403/RandomConfig", RandomConfigService);
  ros::ServiceServer server2 = n.advertiseService(
      "COMPSCI403/ExtendNode", ExtendNodeService);
  ros::ServiceServer server3 = n.advertiseService(
      "COMPSCI403/CheckExtension", CheckExtensionService);
  ros::ServiceServer server4 = n.advertiseService(
      "COMPSCI403/BuildRRT", BuildRRTService);
  ros::ServiceServer server5 = n.advertiseService(
      "COMPSCI403/RRTPlan", RRTPlanService);
  ros::ServiceServer server6 = n.advertiseService(
      "COMPSCI403/RRTPlanBonus", RRTPlanBonusService);

  ros::spin();
  return 0;
}
