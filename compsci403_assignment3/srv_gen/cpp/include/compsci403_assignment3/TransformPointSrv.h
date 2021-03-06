/* Auto-generated by genmsg_cpp for file /home/wyliao/Desktop/compsci403_assignment3/srv/TransformPointSrv.srv */
#ifndef COMPSCI403_ASSIGNMENT3_SERVICE_TRANSFORMPOINTSRV_H
#define COMPSCI403_ASSIGNMENT3_SERVICE_TRANSFORMPOINTSRV_H
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

namespace compsci403_assignment3
{
template <class ContainerAllocator>
struct TransformPointSrvRequest_ {
  typedef TransformPointSrvRequest_<ContainerAllocator> Type;

  TransformPointSrvRequest_()
  : P()
  , R()
  , T()
  {
    R.assign(0.0);
  }

  TransformPointSrvRequest_(const ContainerAllocator& _alloc)
  : P(_alloc)
  , R()
  , T(_alloc)
  {
    R.assign(0.0);
  }

  typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _P_type;
   ::geometry_msgs::Point32_<ContainerAllocator>  P;

  typedef boost::array<float, 9>  _R_type;
  boost::array<float, 9>  R;

  typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _T_type;
   ::geometry_msgs::Point32_<ContainerAllocator>  T;


  typedef boost::shared_ptr< ::compsci403_assignment3::TransformPointSrvRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::compsci403_assignment3::TransformPointSrvRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct TransformPointSrvRequest
typedef  ::compsci403_assignment3::TransformPointSrvRequest_<std::allocator<void> > TransformPointSrvRequest;

typedef boost::shared_ptr< ::compsci403_assignment3::TransformPointSrvRequest> TransformPointSrvRequestPtr;
typedef boost::shared_ptr< ::compsci403_assignment3::TransformPointSrvRequest const> TransformPointSrvRequestConstPtr;



template <class ContainerAllocator>
struct TransformPointSrvResponse_ {
  typedef TransformPointSrvResponse_<ContainerAllocator> Type;

  TransformPointSrvResponse_()
  : P_prime()
  {
  }

  TransformPointSrvResponse_(const ContainerAllocator& _alloc)
  : P_prime(_alloc)
  {
  }

  typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _P_prime_type;
   ::geometry_msgs::Point32_<ContainerAllocator>  P_prime;


  typedef boost::shared_ptr< ::compsci403_assignment3::TransformPointSrvResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::compsci403_assignment3::TransformPointSrvResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct TransformPointSrvResponse
typedef  ::compsci403_assignment3::TransformPointSrvResponse_<std::allocator<void> > TransformPointSrvResponse;

typedef boost::shared_ptr< ::compsci403_assignment3::TransformPointSrvResponse> TransformPointSrvResponsePtr;
typedef boost::shared_ptr< ::compsci403_assignment3::TransformPointSrvResponse const> TransformPointSrvResponseConstPtr;


struct TransformPointSrv
{

typedef TransformPointSrvRequest Request;
typedef TransformPointSrvResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct TransformPointSrv
} // namespace compsci403_assignment3

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::compsci403_assignment3::TransformPointSrvRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::compsci403_assignment3::TransformPointSrvRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::compsci403_assignment3::TransformPointSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "efddb4e09044f2e7a9a8e6cd60bc7de0";
  }

  static const char* value(const  ::compsci403_assignment3::TransformPointSrvRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xefddb4e09044f2e7ULL;
  static const uint64_t static_value2 = 0xa9a8e6cd60bc7de0ULL;
};

template<class ContainerAllocator>
struct DataType< ::compsci403_assignment3::TransformPointSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "compsci403_assignment3/TransformPointSrvRequest";
  }

  static const char* value(const  ::compsci403_assignment3::TransformPointSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::compsci403_assignment3::TransformPointSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "geometry_msgs/Point32 P\n\
float32[9] R\n\
geometry_msgs/Point32 T\n\
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

  static const char* value(const  ::compsci403_assignment3::TransformPointSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::compsci403_assignment3::TransformPointSrvRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::compsci403_assignment3::TransformPointSrvResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::compsci403_assignment3::TransformPointSrvResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::compsci403_assignment3::TransformPointSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6c657ebefd18d441c962728a7a40fd89";
  }

  static const char* value(const  ::compsci403_assignment3::TransformPointSrvResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x6c657ebefd18d441ULL;
  static const uint64_t static_value2 = 0xc962728a7a40fd89ULL;
};

template<class ContainerAllocator>
struct DataType< ::compsci403_assignment3::TransformPointSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "compsci403_assignment3/TransformPointSrvResponse";
  }

  static const char* value(const  ::compsci403_assignment3::TransformPointSrvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::compsci403_assignment3::TransformPointSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "geometry_msgs/Point32 P_prime\n\
\n\
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

  static const char* value(const  ::compsci403_assignment3::TransformPointSrvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::compsci403_assignment3::TransformPointSrvResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::compsci403_assignment3::TransformPointSrvRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.P);
    stream.next(m.R);
    stream.next(m.T);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct TransformPointSrvRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::compsci403_assignment3::TransformPointSrvResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.P_prime);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct TransformPointSrvResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<compsci403_assignment3::TransformPointSrv> {
  static const char* value() 
  {
    return "1e5903ff15280c4ec9bfe3c87782bdd3";
  }

  static const char* value(const compsci403_assignment3::TransformPointSrv&) { return value(); } 
};

template<>
struct DataType<compsci403_assignment3::TransformPointSrv> {
  static const char* value() 
  {
    return "compsci403_assignment3/TransformPointSrv";
  }

  static const char* value(const compsci403_assignment3::TransformPointSrv&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<compsci403_assignment3::TransformPointSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1e5903ff15280c4ec9bfe3c87782bdd3";
  }

  static const char* value(const compsci403_assignment3::TransformPointSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<compsci403_assignment3::TransformPointSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "compsci403_assignment3/TransformPointSrv";
  }

  static const char* value(const compsci403_assignment3::TransformPointSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<compsci403_assignment3::TransformPointSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1e5903ff15280c4ec9bfe3c87782bdd3";
  }

  static const char* value(const compsci403_assignment3::TransformPointSrvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<compsci403_assignment3::TransformPointSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "compsci403_assignment3/TransformPointSrv";
  }

  static const char* value(const compsci403_assignment3::TransformPointSrvResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // COMPSCI403_ASSIGNMENT3_SERVICE_TRANSFORMPOINTSRV_H

