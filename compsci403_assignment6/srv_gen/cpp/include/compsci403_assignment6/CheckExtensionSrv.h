/* Auto-generated by genmsg_cpp for file /home/wyliao/Desktop/compsci403_assignment6/srv/CheckExtensionSrv.srv */
#ifndef COMPSCI403_ASSIGNMENT6_SERVICE_CHECKEXTENSIONSRV_H
#define COMPSCI403_ASSIGNMENT6_SERVICE_CHECKEXTENSIONSRV_H
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

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point.h"



namespace compsci403_assignment6
{
template <class ContainerAllocator>
struct CheckExtensionSrvRequest_ {
  typedef CheckExtensionSrvRequest_<ContainerAllocator> Type;

  CheckExtensionSrvRequest_()
  : q_near()
  , q_new()
  {
  }

  CheckExtensionSrvRequest_(const ContainerAllocator& _alloc)
  : q_near(_alloc)
  , q_new(_alloc)
  {
  }

  typedef  ::geometry_msgs::Point_<ContainerAllocator>  _q_near_type;
   ::geometry_msgs::Point_<ContainerAllocator>  q_near;

  typedef  ::geometry_msgs::Point_<ContainerAllocator>  _q_new_type;
   ::geometry_msgs::Point_<ContainerAllocator>  q_new;


  typedef boost::shared_ptr< ::compsci403_assignment6::CheckExtensionSrvRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::compsci403_assignment6::CheckExtensionSrvRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct CheckExtensionSrvRequest
typedef  ::compsci403_assignment6::CheckExtensionSrvRequest_<std::allocator<void> > CheckExtensionSrvRequest;

typedef boost::shared_ptr< ::compsci403_assignment6::CheckExtensionSrvRequest> CheckExtensionSrvRequestPtr;
typedef boost::shared_ptr< ::compsci403_assignment6::CheckExtensionSrvRequest const> CheckExtensionSrvRequestConstPtr;



template <class ContainerAllocator>
struct CheckExtensionSrvResponse_ {
  typedef CheckExtensionSrvResponse_<ContainerAllocator> Type;

  CheckExtensionSrvResponse_()
  : valid(false)
  {
  }

  CheckExtensionSrvResponse_(const ContainerAllocator& _alloc)
  : valid(false)
  {
  }

  typedef uint8_t _valid_type;
  uint8_t valid;


  typedef boost::shared_ptr< ::compsci403_assignment6::CheckExtensionSrvResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::compsci403_assignment6::CheckExtensionSrvResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct CheckExtensionSrvResponse
typedef  ::compsci403_assignment6::CheckExtensionSrvResponse_<std::allocator<void> > CheckExtensionSrvResponse;

typedef boost::shared_ptr< ::compsci403_assignment6::CheckExtensionSrvResponse> CheckExtensionSrvResponsePtr;
typedef boost::shared_ptr< ::compsci403_assignment6::CheckExtensionSrvResponse const> CheckExtensionSrvResponseConstPtr;


struct CheckExtensionSrv
{

typedef CheckExtensionSrvRequest Request;
typedef CheckExtensionSrvResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct CheckExtensionSrv
} // namespace compsci403_assignment6

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::compsci403_assignment6::CheckExtensionSrvRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::compsci403_assignment6::CheckExtensionSrvRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::compsci403_assignment6::CheckExtensionSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6c0e960d7093358e2eddebf1e3eb523f";
  }

  static const char* value(const  ::compsci403_assignment6::CheckExtensionSrvRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x6c0e960d7093358eULL;
  static const uint64_t static_value2 = 0x2eddebf1e3eb523fULL;
};

template<class ContainerAllocator>
struct DataType< ::compsci403_assignment6::CheckExtensionSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "compsci403_assignment6/CheckExtensionSrvRequest";
  }

  static const char* value(const  ::compsci403_assignment6::CheckExtensionSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::compsci403_assignment6::CheckExtensionSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "geometry_msgs/Point q_near\n\
geometry_msgs/Point q_new\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
";
  }

  static const char* value(const  ::compsci403_assignment6::CheckExtensionSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::compsci403_assignment6::CheckExtensionSrvRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::compsci403_assignment6::CheckExtensionSrvResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::compsci403_assignment6::CheckExtensionSrvResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::compsci403_assignment6::CheckExtensionSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "a04c5033e7efda95fc3786e8d449c6e7";
  }

  static const char* value(const  ::compsci403_assignment6::CheckExtensionSrvResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xa04c5033e7efda95ULL;
  static const uint64_t static_value2 = 0xfc3786e8d449c6e7ULL;
};

template<class ContainerAllocator>
struct DataType< ::compsci403_assignment6::CheckExtensionSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "compsci403_assignment6/CheckExtensionSrvResponse";
  }

  static const char* value(const  ::compsci403_assignment6::CheckExtensionSrvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::compsci403_assignment6::CheckExtensionSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool valid\n\
\n\
\n\
";
  }

  static const char* value(const  ::compsci403_assignment6::CheckExtensionSrvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::compsci403_assignment6::CheckExtensionSrvResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::compsci403_assignment6::CheckExtensionSrvRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.q_near);
    stream.next(m.q_new);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct CheckExtensionSrvRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::compsci403_assignment6::CheckExtensionSrvResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.valid);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct CheckExtensionSrvResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<compsci403_assignment6::CheckExtensionSrv> {
  static const char* value() 
  {
    return "103e7eef778bb10302817d90a24b89ed";
  }

  static const char* value(const compsci403_assignment6::CheckExtensionSrv&) { return value(); } 
};

template<>
struct DataType<compsci403_assignment6::CheckExtensionSrv> {
  static const char* value() 
  {
    return "compsci403_assignment6/CheckExtensionSrv";
  }

  static const char* value(const compsci403_assignment6::CheckExtensionSrv&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<compsci403_assignment6::CheckExtensionSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "103e7eef778bb10302817d90a24b89ed";
  }

  static const char* value(const compsci403_assignment6::CheckExtensionSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<compsci403_assignment6::CheckExtensionSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "compsci403_assignment6/CheckExtensionSrv";
  }

  static const char* value(const compsci403_assignment6::CheckExtensionSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<compsci403_assignment6::CheckExtensionSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "103e7eef778bb10302817d90a24b89ed";
  }

  static const char* value(const compsci403_assignment6::CheckExtensionSrvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<compsci403_assignment6::CheckExtensionSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "compsci403_assignment6/CheckExtensionSrv";
  }

  static const char* value(const compsci403_assignment6::CheckExtensionSrvResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // COMPSCI403_ASSIGNMENT6_SERVICE_CHECKEXTENSIONSRV_H
