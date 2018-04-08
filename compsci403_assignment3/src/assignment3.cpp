#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <algorithm>

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
using Eigen::MatrixXf;
using Eigen::VectorXf;

// Declare class variables, publishers, messages
ros::ServiceServer transformPointSrv;
ros::ServiceServer fitMinimalPlaneSrv;
ros::ServiceServer findInliersSrv;
ros::ServiceServer fitBestPlaneSrv;
ros::Subscriber pointCloudSub;
ros::Publisher filteredPCPublisher;
ros::Publisher planeParametersPublisher;
// Define service and callback functions

//Helper method for part 4 and 5
void createBestFitPlaneHelper(std::vector<Point32> points, Point32& nHat, Point32& p0){
  std::vector<Point32>::iterator it;
  float x = 0;
  float y = 0;
  float z = 0;
  float size = 0;
  Point32 P0;
  //Compute average of points to be P0
  for(it = points.begin(); it != points.end(); ++it){
    Point32 P = *it;
    x += P.x;
    y += P.y;
    z += P.z;
    size++;
  }
  P0.x = x/size;
  P0.y = y/size;
  P0.z = z/size;
  MatrixXf M((int)size,3);
  MatrixXf MT(3,(int)size);
  int row = 0;
  //M = Pi - P0
  for(it = points.begin(); it != points.end(); ++it){
    Point32 P = *it;
    M(row, 0) = P.x - P0.x;
    M(row, 1) = P.y - P0.y;
    M(row, 2) = P.z - P0.z;
    row++;
  }
  MT = M.transpose();
  // MT * M n = 0
  MatrixXf product = MT*M;
  //Get smallest eigenvalue of the resulting matrix
  //The corresponding eigenvector will be n.
  Eigen::EigenSolver<MatrixXf> eigen_solver;
  eigen_solver.compute(product);
  VectorXf eigenValues = eigen_solver.eigenvalues().real();
  MatrixXf eigenVectors = eigen_solver.eigenvectors().real();
  float smallestEigenValue = eigenValues(0);
  Vector3f smallestEigenVector = eigenVectors.col(0);
  int eigenIndex = 0;
  for(int i =0; i < eigenValues.size(); ++i){
    float eigenValue = eigenValues(i);
    if(eigenValue < smallestEigenValue){
      eigenIndex = i;
      smallestEigenValue = eigenValue;
    }
  }
  //For some reason, eigenVectors.col(i) does not play well in that loop.
  //So set it using the index below
  //set references
  p0 = P0;
  smallestEigenVector = eigenVectors.col(eigenIndex);
  nHat.x = smallestEigenVector(0);
  nHat.y = smallestEigenVector(1);
  nHat.z = smallestEigenVector(2);
}

bool TransformPoint(
    compsci403_assignment3::TransformPointSrv::Request& req,
    compsci403_assignment3::TransformPointSrv::Response& res){
  // Declare a 3x3 matrix.
  //P' = RP+T
  Vector3f point;
  Vector3f transform;
  Matrix3f rotation;
  point = Vector3f(req.P.x, req.P.y, req.P.z);
  transform = Vector3f(req.T.x, req.T.y, req.T.z);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      rotation(i,j) = req.R[i*3+j];
    }
  }
  Vector3f result = (rotation * point) + transform;
  res.P_prime.x = result.x();
  res.P_prime.y = result.y();
  res.P_prime.z = result.z();
  return true;
}

bool FitMinimalPlane(
    compsci403_assignment3::FitMinimalPlaneSrv::Request& req,
    compsci403_assignment3::FitMinimalPlaneSrv::Response& res){
  //P1, P2, P3, P0, n
  //direction vectors, p1->p2, p1->p3
  //normal = d1xd2
  Vector3f p1p2 = Vector3f(req.P2.x - req.P1.x, req.P2.y - req.P1.y, req.P2.z - req.P1.z);
  Vector3f p1p3 = Vector3f(req.P3.x - req.P1.x, req.P3.y - req.P1.y, req.P3.z - req.P1.z);
  Vector3f normal = p1p2.cross(p1p3);
  res.P0.x = req.P1.x;
  res.P0.y = req.P1.y;
  res.P0.z = req.P1.z;
  res.n.x = normal.x();
  res.n.y = normal.y();
  res.n.z = normal.z();
  return true;
}

bool FindInliers(
    compsci403_assignment3::FindInliersSrv::Request& req,
    compsci403_assignment3::FindInliersSrv::Response& res){
  //calculate D, n.x = a, n.y = b, n.z = c
  // = -n.x * p.x - n.y * p.y - n.z + p.z
  std::vector<Point32> inliers;
  float D = -(req.n.x*req.P0.x) - (req.n.y*req.P0.y) - (req.n.z*req.P0.z);
  std::vector<Point32>::iterator it;
  for(it = req.P.begin(); it != req.P.end(); ++it){
    Point32 P = *it;
    float distance = (req.n.x*P.x) + (req.n.y*P.y) + (req.n.z*P.z) + D;
    if(distance < 0){
      distance = distance*-1;
    }
    distance = distance / sqrt((req.n.x*req.n.x + req.n.y*req.n.y + req.n.z*req.n.z));
    if(distance < req.epsilon){
      inliers.push_back(P);
    }
  }
  res.P = inliers;
  return true;
}

bool FitBestPlane(
    compsci403_assignment3::FitBestPlaneSrv::Request& req,
    compsci403_assignment3::FitBestPlaneSrv::Response& res){
  Point32 n;
  Point32 P0;
  //Use helper method and pass in n and P0 as references
  createBestFitPlaneHelper(req.P, n, P0);
  res.n = n;
  res.P0 = P0;
  return true;
}


void PointCloudCallback(const sensor_msgs::PointCloud& pc) {
  //Need to create a best fit plane from n points. (randomly selected)
  //Then need to compare the rest of the points to the plane and only take those that satifactorily fit.
  //Then get that count/n and compare to f
  compsci403_assignment3::PlaneParametersMsg planeParams;
  sensor_msgs::PointCloud filteredPC;
  filteredPC.header = pc.header;
  std::vector<Point32> points = pc.points;
  std::vector<Point32>::iterator it;
  bool satisfied = false;
  float epsilon = .01;
  float f = .7;
  float nPoints = 5;
  int size = points.size();
  //Loop until we find a sample that matches our thresholds
  while(!satisfied){
    Point32 nHat;
    Point32 p0;
    float D;
    std::vector<Point32> randomPoints;
    std::vector<Point32> inliers;
    //Acquire n random points
    while(randomPoints.size() <= nPoints){
      int random = rand()%size;
      Point32 randomPoint = points.at(random);
      bool contains = false;
      for(unsigned int i = 0; i < randomPoints.size(); ++i){
	if((randomPoints.at(i).x == randomPoint.x) && (randomPoints.at(i).y == randomPoint.y) && (randomPoints.at(i).z == randomPoint.z)){
	  contains = true;
	  break;
	}
      }
      if(!contains){
	randomPoints.push_back(randomPoint);
      }
    }
    createBestFitPlaneHelper(randomPoints, nHat, p0); //pass nHat and p0 as references
    //Compute distance for each point and get the inliers
    D = -(nHat.x*p0.x) - (nHat.y*p0.y) - (nHat.z*p0.z);
    for(it = points.begin(); it != points.end(); ++it){
      Point32 P = *it;
      float distance = 0;
      distance = (nHat.x*P.x) + (nHat.y*P.y) + (nHat.z*P.z) + D;
      if(distance < 0){
	distance = distance*-1;
      }
      float squareroot = sqrtf((nHat.x*nHat.x + nHat.y*nHat.y + nHat.z*nHat.z));
      distance = distance / squareroot;
      if(distance < epsilon){
	inliers.push_back(P);
      }
    }
    float percentage = ((float)inliers.size()) / points.size();
    if(percentage > f){
      //Found match
      //Fit plane against the inliers again and return those plane params.
      createBestFitPlaneHelper(inliers, nHat, p0);
      filteredPC.points = inliers;
      planeParams.n = nHat;
      planeParams.P0 = p0;
      satisfied = true;
      break;
    }
  }
  filteredPCPublisher.publish(filteredPC); //Publish point cloud
  planeParametersPublisher.publish(planeParams); //Publish point cloud
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "assignment3");
  ros::NodeHandle n;
  transformPointSrv = n.advertiseService("/COMPSCI403/TransformPoint", TransformPoint);
  fitMinimalPlaneSrv = n.advertiseService("/COMPSCI403/FitMinimalPlane", FitMinimalPlane);
  findInliersSrv = n.advertiseService("/COMPSCI403/FindInliers", FindInliers);
  fitBestPlaneSrv = n.advertiseService("/COMPSCI403/FitBestPlane", FitBestPlane);
  pointCloudSub = n.subscribe("/COMPSCI403/PointCloud", 1000, PointCloudCallback);

  filteredPCPublisher = n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/FilteredPointCloud", 1000);
  planeParametersPublisher = n.advertise<compsci403_assignment3::PlaneParametersMsg>("/COMPSCI403/PlaneParameters", 1000);
  ros::spin();

  return 0;
}
