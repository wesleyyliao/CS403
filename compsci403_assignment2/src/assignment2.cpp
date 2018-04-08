#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>

#include "compsci403_assignment2/Get3DPointFromDepthSrv.h"
#include "compsci403_assignment2/Get3DPointFromDisparitySrv.h"
#include "compsci403_assignment2/GetDepthFromDisparitySrv.h"
#include "compsci403_assignment2/GetIntrinsicsSrv.h"
#include "compsci403_assignment2/GetPixelFrom3DPointSrv.h"

// Include any additional header or service files

// Declare class variables, messages, publishers
//intrinsics
float i_fx;
float i_fy;
float i_px;
float i_py;
float i_a;
float i_b;
ros::ServiceServer getPix3DPointService;
ros::ServiceServer get3DPointFromDepthService;
ros::ServiceServer getDepthFromDisparityService;
ros::ServiceServer get3DPointFromDisparityService;
ros::ServiceClient getIntrinsicsClient;
ros::Subscriber depthImageSub;
ros::Publisher pcPublisher;
// Define service and callback functions

void DepthImageCallback(const sensor_msgs::Image& image) {
  const int image_width = image.width;
  const int image_height = image.height;
  sensor_msgs::PointCloud pc;
  int pixelCount = image_width*image_height;
  int i = 0;
  pc.header = image.header;
  pc.points.resize(pixelCount); //initialize pointcloud size
  for (int y = 0; y < image_height; ++y) {
    for (int x = 0; x < image_width; ++x) {
      uint16_t byte0 = image.data[2 * (x + y * image_width) + 0];
      uint16_t byte1 = image.data[2 * (x + y * image_width) + 1];
      if (!image.is_bigendian) {
        std::swap(byte0, byte1);
      }
      // Combine the two bytes to form a 16 bit value, and disregard the
      // most significant 4 bits to extract the lowest 12 bits.
      const uint16_t raw_depth = ((byte0 << 8) | byte1) & 0x7FF;

      // Reconstruct 3D point from x, y, raw_depth, and sensor intrinsics
      geometry_msgs::Point32 curr;
      //Compute depth from d = 1/a+br
      //Need to cast raw_depth, x, y to floats
      float depth = (1/((i_a)+(i_b)* (float)(raw_depth)));
      //Get focal length
      float i_f = (sqrt((i_fx * i_fx) + (i_fy * i_fy)));
      //Acquire X and Y using the below equation
      //y = (fY)/z + py   ->   Solve for Y, Y = z(f-py)/f
      curr.x = ((depth*((float)x - i_px))/i_f);
      curr.y = ((depth*((float)y - i_py))/i_f);
      curr.z = depth;
      pc.points[i]=curr; //Add point to point cloud
      i++;
    }
  }
  pcPublisher.publish(pc); //Publish point cloud
}

bool GetPixelFrom3DPoint(
    compsci403_assignment2::GetPixelFrom3DPointSrv::Request& req,
    compsci403_assignment2::GetPixelFrom3DPointSrv::Response& res){
  //Compute focal length
  float f = sqrt((req.fx * req.fx) + (req.fy * req.fy));
  //y = (fY)/z + py
  res.x = (f * req.X)/req.Z + req.px;
  res.y = (f * req.Y)/req.Z + req.py;
  return true;
}

bool Get3DPointFromDepth(
    compsci403_assignment2::Get3DPointFromDepthSrv::Request& req,
    compsci403_assignment2::Get3DPointFromDepthSrv::Response& res){
  //x,y,depth,fx,fy,px,py
  //x and y are ints, cast to float
  //Y = z(y-py)/f     <-- derived from y = fY/z + py, solve for Y
  //X = z(x-px)/f
  float f = sqrt((req.fx * req.fx) + (req.fy * req.fy));
  res.X = (req.depth*((float)req.x - req.px))/f;
  res.Y = (req.depth*((float)req.y - req.py))/f;
  res.Z = req.depth;
  return true;
}

bool GetDepthFromDisparity(
    compsci403_assignment2::GetDepthFromDisparitySrv::Request& req,
    compsci403_assignment2::GetDepthFromDisparitySrv::Response& res){
  //disparity,a,b
  //apply d=1/a+br
  res.depth = 1/(req.a+req.b*req.disparity);
  return true;
}

bool Get3DPointFromDisparity(
    compsci403_assignment2::Get3DPointFromDisparitySrv::Request& req,
    compsci403_assignment2::Get3DPointFromDisparitySrv::Response& res){
  //x,y,disparity,fx,fy,px,py,a,b
  //apply d=1/a+br for depth
  //Y = z(y-py)/f
  //X = z(x-px)/f
  float f = sqrt((req.fx * req.fx) + (req.fy * req.fy));
  float depth = 1/(req.a+req.b*req.disparity);
  res.X = (depth*(req.x - req.px))/f;
  res.Y = (depth*(req.y - req.py))/f;
  res.Z = depth;
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "assignment2");
  ros::NodeHandle n;
  // Perform operations defined in Assignment 2
  //Parts 1-4
  getPix3DPointService = n.advertiseService("/COMPSCI403/GetPixelFrom3DPoint", GetPixelFrom3DPoint);
  get3DPointFromDepthService = n.advertiseService("/COMPSCI403/Get3DPointFromDepth", Get3DPointFromDepth);
  getDepthFromDisparityService = n.advertiseService("/COMPSCI403/GetDepthFromDisparity", GetDepthFromDisparity);
  get3DPointFromDisparityService = n.advertiseService("/COMPSCI403/Get3DPointFromDisparity", Get3DPointFromDisparity);

  //Part 5
  getIntrinsicsClient = n.serviceClient<compsci403_assignment2::GetIntrinsicsSrv>("/COMPSCI403/GetIntrinsics");
  compsci403_assignment2::GetIntrinsicsSrv srv;
  //Get intrinsics for part 5
  ros::Rate loop(10);
  while (ros::ok()) {
    if(getIntrinsicsClient.call(srv)){
      i_fx = srv.response.fx;
      i_fy = srv.response.fy;
      i_px = srv.response.px;
      i_py = srv.response.py;
      i_a = srv.response.a;
      i_b = srv.response.b;
      break; //Only need to get intrinsics once (after storing it)
    }
    ros::spinOnce();
    loop.sleep();
  }

  pcPublisher = n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/PointCloud", 1000);
  depthImageSub = n.subscribe("/COMPSCI403/DepthImage", 1000, DepthImageCallback);
  ros::spin();

  return 0;
}
