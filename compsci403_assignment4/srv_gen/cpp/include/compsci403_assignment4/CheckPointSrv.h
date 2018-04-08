/* Auto-generated by genmsg_cpp for file /home/wyliao/Desktop/compsci403_assignment4/srv/CheckPointSrv.srv */
#ifndef COMPSCI403_ASSIGNMENT4_SERVICE_CHECKPOINTSRV_H
#define COMPSCI403_ASSIGNMENT4_SERVICE_CHECKPOINTSRV_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"

#include "geometry_msgs/Point32.h"



namespace compsci403_assignment4
{
template <class ContainerAllocator>
struct CheckPointSrvRequest_ {
  typedef CheckPointSrvRequest_<ContainerAllocator> Type;

  CheckPointSrvRequest_()
  : P()
  , v(0.0)
  , w(0.0)
  {
  }

  CheckPointSrvRequest_(const ContainerAllocator& _alloc)
  : P(_alloc)
  , v(0.0)
  , w(0.0)
  {
  }

  typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _P_type;
   ::geometry_msgs::Point32_<ContainerAllocator>  P;

  typedef float _v_type;
  float v;

  typedef float _w_type;
  float w;


  typedef boost::shared_ptr< ::compsci403_assignment4::CheckPointSrvRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::compsci403_assignment4::CheckPointSrvRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct CheckPointSrvRequest
typedef  ::compsci403_assignment4::CheckPointSrvRequest_<std::allocator<void> > CheckPointSrvRequest;

typedef boost::shared_ptr< ::compsci403_assignment4::CheckPointSrvRequest> CheckPointSrvRequestPtr;
typedef boost::shared_ptr< ::compsci403_assignment4::CheckPointSrvRequest const> CheckPointSrvRequestConstPtr;



template <class ContainerAllocator>
struct CheckPointSrvResponse_ {
  typedef CheckPointSrvResponse_<ContainerAllocator> Type;

  CheckPointSrvResponse_()
  : is_obstacle(false)
  , free_path_length(0.0)
  {
  }

  CheckPointSrvResponse_(const ContainerAllocator& _alloc)
  : is_obstacle(false)
  , free_path_length(0.0)
  {
  }

  typedef uint8_t _is_obstacle_type;
  uint8_t is_obstacle;

  typedef float _free_path_length_type;
  float free_path_length;


  typedef boost::shared_ptr< ::compsci403_assignment4::CheckPointSrvResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::compsci403_assignment4::CheckPointSrvResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct CheckPointSrvResponse
typedef  ::compsci403_assignment4::CheckPointSrvResponse_<std::allocator<void> > CheckPointSrvResponse;

typedef boost::shared_ptr< ::compsci403_assignment4::CheckPointSrvResponse> CheckPointSrvResponsePtr;
typedef boost::shared_ptr< ::compsci403_assignment4::CheckPointSrvResponse const> CheckPointSrvResponseConstPtr;


struct CheckPointSrv
{

typedef CheckPointSrvRequest Request;
typedef CheckPointSrvResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct CheckPointSrv
} // namespace compsci403_assignment4

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::compsci403_assignment4::CheckPointSrvRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::compsci403_assignment4::CheckPointSrvRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::compsci403_assignment4::CheckPointSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "88ccfd3745917ca69db13060dd7c532a";
  }

  static const char* value(const  ::compsci403_assignment4::CheckPointSrvRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x88ccfd3745917ca6ULL;
  static const uint64_t static_value2 = 0x9db13060dd7c532aULL;
};

template<class ContainerAllocator>
struct DataType< ::compsci403_assignment4::CheckPointSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "compsci403_assignment4/CheckPointSrvRequest";
  }

  static const char* value(const  ::compsci403_assignment4::CheckPointSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::compsci403_assignment4::CheckPointSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "geometry_msgs/Point32 P\n\
float32 v\n\
float32 w\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point32\n\
# This contains the position of a point in free space(with 32 bits of precision).\n\
# It is recommeded to use Point wherever possible instead of Point32.  \n\
# \n\
# This recommendation is to promote interoperability.  \n\
#\n\
# This message is designed to take up less space when sending\n\
# lots of points at once, as in the case of a PointCloud.  \n\
\n\
float32 x\n\
float32 y\n\
float32 z\n\
";
  }

  static const char* value(const  ::compsci403_assignment4::CheckPointSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::compsci403_assignment4::CheckPointSrvRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::compsci403_assignment4::CheckPointSrvResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::compsci403_assignment4::CheckPointSrvResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::compsci403_assignment4::CheckPointSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "79c5379529f76e6a7f5e100191a058dd";
  }

  static const char* value(const  ::compsci403_assignment4::CheckPointSrvResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x79c5379529f76e6aULL;
  static const uint64_t static_value2 = 0x7f5e100191a058ddULL;
};

template<class ContainerAllocator>
struct DataType< ::compsci403_assignment4::CheckPointSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "compsci403_assignment4/CheckPointSrvResponse";
  }

  static const char* value(const  ::compsci403_assignment4::CheckPointSrvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::compsci403_assignment4::CheckPointSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool is_obstacle\n\
float32 free_path_length\n\
\n\
\n\
";
  }

  static const char* value(const  ::compsci403_assignment4::CheckPointSrvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::compsci403_assignment4::CheckPointSrvResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::compsci403_assignment4::CheckPointSrvRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.P);
    stream.next(m.v);
    stream.next(m.w);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct CheckPointSrvRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::compsci403_assignment4::CheckPointSrvResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.is_obstacle);
    stream.next(m.free_path_length);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct CheckPointSrvResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<compsci403_assignment4::CheckPointSrv> {
  static const char* value() 
  {
    return "942f1fcf986c084288bf027e6e35f386";
  }

  static const char* value(const compsci403_assignment4::CheckPointSrv&) { return value(); } 
};

template<>
struct DataType<compsci403_assignment4::CheckPointSrv> {
  static const char* value() 
  {
    return "compsci403_assignment4/CheckPointSrv";
  }

  static const char* value(const compsci403_assignment4::CheckPointSrv&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<compsci403_assignment4::CheckPointSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "942f1fcf986c084288bf027e6e35f386";
  }

  static const char* value(const compsci403_assignment4::CheckPointSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<compsci403_assignment4::CheckPointSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "compsci403_assignment4/CheckPointSrv";
  }

  static const char* value(const compsci403_assignment4::CheckPointSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<compsci403_assignment4::CheckPointSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "942f1fcf986c084288bf027e6e35f386";
  }

  static const char* value(const compsci403_assignment4::CheckPointSrvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<compsci403_assignment4::CheckPointSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "compsci403_assignment4/CheckPointSrv";
  }

  static const char* value(const compsci403_assignment4::CheckPointSrvResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // COMPSCI403_ASSIGNMENT4_SERVICE_CHECKPOINTSRV_H

