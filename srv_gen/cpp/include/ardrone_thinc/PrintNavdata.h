/* Auto-generated by genmsg_cpp for file /home/caseyhetzler/fuerte_workspace/sandbox/ardrone_thinc/srv/PrintNavdata.srv */
#ifndef ARDRONE_THINC_SERVICE_PRINTNAVDATA_H
#define ARDRONE_THINC_SERVICE_PRINTNAVDATA_H
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




namespace ardrone_thinc
{
template <class ContainerAllocator>
struct PrintNavdataRequest_ {
  typedef PrintNavdataRequest_<ContainerAllocator> Type;

  PrintNavdataRequest_()
  : id(0)
  , option(0)
  {
  }

  PrintNavdataRequest_(const ContainerAllocator& _alloc)
  : id(0)
  , option(0)
  {
  }

  typedef int32_t _id_type;
  int32_t id;

  typedef int32_t _option_type;
  int32_t option;


  typedef boost::shared_ptr< ::ardrone_thinc::PrintNavdataRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ardrone_thinc::PrintNavdataRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PrintNavdataRequest
typedef  ::ardrone_thinc::PrintNavdataRequest_<std::allocator<void> > PrintNavdataRequest;

typedef boost::shared_ptr< ::ardrone_thinc::PrintNavdataRequest> PrintNavdataRequestPtr;
typedef boost::shared_ptr< ::ardrone_thinc::PrintNavdataRequest const> PrintNavdataRequestConstPtr;


template <class ContainerAllocator>
struct PrintNavdataResponse_ {
  typedef PrintNavdataResponse_<ContainerAllocator> Type;

  PrintNavdataResponse_()
  : id(0)
  , option(0)
  {
  }

  PrintNavdataResponse_(const ContainerAllocator& _alloc)
  : id(0)
  , option(0)
  {
  }

  typedef int32_t _id_type;
  int32_t id;

  typedef int32_t _option_type;
  int32_t option;


  typedef boost::shared_ptr< ::ardrone_thinc::PrintNavdataResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ardrone_thinc::PrintNavdataResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PrintNavdataResponse
typedef  ::ardrone_thinc::PrintNavdataResponse_<std::allocator<void> > PrintNavdataResponse;

typedef boost::shared_ptr< ::ardrone_thinc::PrintNavdataResponse> PrintNavdataResponsePtr;
typedef boost::shared_ptr< ::ardrone_thinc::PrintNavdataResponse const> PrintNavdataResponseConstPtr;

struct PrintNavdata
{

typedef PrintNavdataRequest Request;
typedef PrintNavdataResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct PrintNavdata
} // namespace ardrone_thinc

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::ardrone_thinc::PrintNavdataRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::ardrone_thinc::PrintNavdataRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::ardrone_thinc::PrintNavdataRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "2a91c9f5611f4e63a9725ecaf46fda18";
  }

  static const char* value(const  ::ardrone_thinc::PrintNavdataRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x2a91c9f5611f4e63ULL;
  static const uint64_t static_value2 = 0xa9725ecaf46fda18ULL;
};

template<class ContainerAllocator>
struct DataType< ::ardrone_thinc::PrintNavdataRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ardrone_thinc/PrintNavdataRequest";
  }

  static const char* value(const  ::ardrone_thinc::PrintNavdataRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::ardrone_thinc::PrintNavdataRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 id\n\
int32 option\n\
\n\
";
  }

  static const char* value(const  ::ardrone_thinc::PrintNavdataRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::ardrone_thinc::PrintNavdataRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::ardrone_thinc::PrintNavdataResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::ardrone_thinc::PrintNavdataResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::ardrone_thinc::PrintNavdataResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "2a91c9f5611f4e63a9725ecaf46fda18";
  }

  static const char* value(const  ::ardrone_thinc::PrintNavdataResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x2a91c9f5611f4e63ULL;
  static const uint64_t static_value2 = 0xa9725ecaf46fda18ULL;
};

template<class ContainerAllocator>
struct DataType< ::ardrone_thinc::PrintNavdataResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ardrone_thinc/PrintNavdataResponse";
  }

  static const char* value(const  ::ardrone_thinc::PrintNavdataResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::ardrone_thinc::PrintNavdataResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 id\n\
int32 option\n\
\n\
\n\
";
  }

  static const char* value(const  ::ardrone_thinc::PrintNavdataResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::ardrone_thinc::PrintNavdataResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::ardrone_thinc::PrintNavdataRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.id);
    stream.next(m.option);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PrintNavdataRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::ardrone_thinc::PrintNavdataResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.id);
    stream.next(m.option);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PrintNavdataResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<ardrone_thinc::PrintNavdata> {
  static const char* value() 
  {
    return "80f3a199d4e318f38888fb7a91c62e79";
  }

  static const char* value(const ardrone_thinc::PrintNavdata&) { return value(); } 
};

template<>
struct DataType<ardrone_thinc::PrintNavdata> {
  static const char* value() 
  {
    return "ardrone_thinc/PrintNavdata";
  }

  static const char* value(const ardrone_thinc::PrintNavdata&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<ardrone_thinc::PrintNavdataRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "80f3a199d4e318f38888fb7a91c62e79";
  }

  static const char* value(const ardrone_thinc::PrintNavdataRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<ardrone_thinc::PrintNavdataRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ardrone_thinc/PrintNavdata";
  }

  static const char* value(const ardrone_thinc::PrintNavdataRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<ardrone_thinc::PrintNavdataResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "80f3a199d4e318f38888fb7a91c62e79";
  }

  static const char* value(const ardrone_thinc::PrintNavdataResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<ardrone_thinc::PrintNavdataResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ardrone_thinc/PrintNavdata";
  }

  static const char* value(const ardrone_thinc::PrintNavdataResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // ARDRONE_THINC_SERVICE_PRINTNAVDATA_H

