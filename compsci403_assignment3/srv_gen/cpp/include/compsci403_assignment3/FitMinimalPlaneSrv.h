/* Auto-generated by genmsg_cpp for file /home/wyliao/Desktop/compsci403_assignment3/srv/FitMinimalPlaneSrv.srv */
#ifndef COMPSCI403_ASSIGNMENT3_SERVICE_FITMINIMALPLANESRV_H
#define COMPSCI403_ASSIGNMENT3_SERVICE_FITMINIMALPLANESRV_H
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
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Point32.h"


#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Point32.h"

namespace compsci403_assignment3
{
template <class ContainerAllocator>
struct FitMinimalPlaneSrvRequest_ {
  typedef FitMinimalPlaneSrvRequest_<ContainerAllocator> Type;

  FitMinimalPlaneSrvRequest_()
  : P1()
  , P2()
  , P3()
  {
  }

  FitMinimalPlaneSrvRequest_(const ContainerAllocator& _alloc)
  : P1(_alloc)
  , P2(_alloc)
  , P3(_alloc)
  {
  }

  typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _P1_type;
   ::geometry_msgs::Point32_<ContainerAllocator>  P1;

  typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _P2_type;
   ::geometry_msgs::Point32_<ContainerAllocator>  P2;

  typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _P3_type;
   ::geometry_msgs::Point32_<ContainerAllocator>  P3;


  typedef boost::shared_ptr< ::compsci403_assignment3::FitMinimalPlaneSrvRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::compsci403_assignment3::FitMinimalPlaneSrvRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct FitMinimalPlaneSrvRequest
typedef  ::compsci403_assignment3::FitMinimalPlaneSrvRequest_<std::allocator<void> > FitMinimalPlaneSrvRequest;

typedef boost::shared_ptr< ::compsci403_assignment3::FitMinimalPlaneSrvRequest> FitMinimalPlaneSrvRequestPtr;
typedef boost::shared_ptr< ::compsci403_assignment3::FitMinimalPlaneSrvRequest const> FitMinimalPlaneSrvRequestConstPtr;



template <class ContainerAllocator>
struct FitMinimalPlaneSrvResponse_ {
  typedef FitMinimalPlaneSrvResponse_<ContainerAllocator> Type;

  FitMinimalPlaneSrvResponse_()
  : P0()
  , n()
  {
  }

  FitMinimalPlaneSrvResponse_(const ContainerAllocator& _alloc)
  : P0(_alloc)
  , n(_alloc)
  {
  }

  typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _P0_type;
   ::geometry_msgs::Point32_<ContainerAllocator>  P0;

  typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _n_type;
   ::geometry_msgs::Point32_<ContainerAllocator>  n;


  typedef boost::shared_ptr< ::compsci403_assignment3::FitMinimalPlaneSrvResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::compsci403_assignment3::FitMinimalPlaneSrvResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct FitMinimalPlaneSrvResponse
typedef  ::compsci403_assignment3::FitMinimalPlaneSrvResponse_<std::allocator<void> > FitMinimalPlaneSrvResponse;

typedef boost::shared_ptr< ::compsci403_assignment3::FitMinimalPlaneSrvResponse> FitMinimalPlaneSrvResponsePtr;
typedef boost::shared_ptr< ::compsci403_assignment3::FitMinimalPlaneSrvResponse const> FitMinimalPlaneSrvResponseConstPtr;


struct FitMinimalPlaneSrv
{

typedef FitMinimalPlaneSrvRequest Request;
typedef FitMinimalPlaneSrvResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct FitMinimalPlaneSrv
} // namespace compsci403_assignment3

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::compsci403_assignment3::FitMinimalPlaneSrvRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::compsci403_assignment3::FitMinimalPlaneSrvRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::compsci403_assignment3::FitMinimalPlaneSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "31aaecb61f65fa1b1cf756f63eacadf1";
  }

  static const char* value(const  ::compsci403_assignment3::FitMinimalPlaneSrvRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x31aaecb61f65fa1bULL;
  static const uint64_t static_value2 = 0x1cf756f63eacadf1ULL;
};

template<class ContainerAllocator>
struct DataType< ::compsci403_assignment3::FitMinimalPlaneSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "compsci403_assignment3/FitMinimalPlaneSrvRequest";
  }

  static const char* value(const  ::compsci403_assignment3::FitMinimalPlaneSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::compsci403_assignment3::FitMinimalPlaneSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "geometry_msgs/Point32 P1\n\
geometry_msgs/Point32 P2\n\
geometry_msgs/Point32 P3\n\
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

  static const char* value(const  ::compsci403_assignment3::FitMinimalPlaneSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::compsci403_assignment3::FitMinimalPlaneSrvRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::compsci403_assignment3::FitMinimalPlaneSrvResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::compsci403_assignment3::FitMinimalPlaneSrvResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::compsci403_assignment3::FitMinimalPlaneSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "185eea9c3fedfb05e4edf26c506b38ef";
  }

  static const char* value(const  ::compsci403_assignment3::FitMinimalPlaneSrvResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x185eea9c3fedfb05ULL;
  static const uint64_t static_value2 = 0xe4edf26c506b38efULL;
};

template<class ContainerAllocator>
struct DataType< ::compsci403_assignment3::FitMinimalPlaneSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "compsci403_assignment3/FitMinimalPlaneSrvResponse";
  }

  static const char* value(const  ::compsci403_assignment3::FitMinimalPlaneSrvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::compsci403_assignment3::FitMinimalPlaneSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "geometry_msgs/Point32 P0\n\
geometry_msgs/Point32 n\n\
\n\
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

  static const char* value(const  ::compsci403_assignment3::FitMinimalPlaneSrvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::compsci403_assignment3::FitMinimalPlaneSrvResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::compsci403_assignment3::FitMinimalPlaneSrvRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.P1);
    stream.next(m.P2);
    stream.next(m.P3);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct FitMinimalPlaneSrvRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::compsci403_assignment3::FitMinimalPlaneSrvResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.P0);
    stream.next(m.n);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct FitMinimalPlaneSrvResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<compsci403_assignment3::FitMinimalPlaneSrv> {
  static const char* value() 
  {
    return "0a7e6160a47703bcae7741304dcb479d";
  }

  static const char* value(const compsci403_assignment3::FitMinimalPlaneSrv&) { return value(); } 
};

template<>
struct DataType<compsci403_assignment3::FitMinimalPlaneSrv> {
  static const char* value() 
  {
    return "compsci403_assignment3/FitMinimalPlaneSrv";
  }

  static const char* value(const compsci403_assignment3::FitMinimalPlaneSrv&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<compsci403_assignment3::FitMinimalPlaneSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0a7e6160a47703bcae7741304dcb479d";
  }

  static const char* value(const compsci403_assignment3::FitMinimalPlaneSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<compsci403_assignment3::FitMinimalPlaneSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "compsci403_assignment3/FitMinimalPlaneSrv";
  }

  static const char* value(const compsci403_assignment3::FitMinimalPlaneSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<compsci403_assignment3::FitMinimalPlaneSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0a7e6160a47703bcae7741304dcb479d";
  }

  static const char* value(const compsci403_assignment3::FitMinimalPlaneSrvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<compsci403_assignment3::FitMinimalPlaneSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "compsci403_assignment3/FitMinimalPlaneSrv";
  }

  static const char* value(const compsci403_assignment3::FitMinimalPlaneSrvResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // COMPSCI403_ASSIGNMENT3_SERVICE_FITMINIMALPLANESRV_H
