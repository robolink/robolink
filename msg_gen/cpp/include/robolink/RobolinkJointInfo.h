/* Auto-generated by genmsg_cpp for file /home/ttremblay/groovy_workspace/sandbox/robolink/msg/RobolinkJointInfo.msg */
#ifndef ROBOLINK_MESSAGE_ROBOLINKJOINTINFO_H
#define ROBOLINK_MESSAGE_ROBOLINKJOINTINFO_H
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


namespace robolink
{
template <class ContainerAllocator>
struct RobolinkJointInfo_ {
  typedef RobolinkJointInfo_<ContainerAllocator> Type;

  RobolinkJointInfo_()
  : joint_name()
  , joint_num(0)
  , joint_angle(0)
  , joint_velocity(0)
  , stamp()
  {
  }

  RobolinkJointInfo_(const ContainerAllocator& _alloc)
  : joint_name(_alloc)
  , joint_num(0)
  , joint_angle(0)
  , joint_velocity(0)
  , stamp()
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _joint_name_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  joint_name;

  typedef uint16_t _joint_num_type;
  uint16_t joint_num;

  typedef int32_t _joint_angle_type;
  int32_t joint_angle;

  typedef int32_t _joint_velocity_type;
  int32_t joint_velocity;

  typedef ros::Time _stamp_type;
  ros::Time stamp;


  typedef boost::shared_ptr< ::robolink::RobolinkJointInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robolink::RobolinkJointInfo_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct RobolinkJointInfo
typedef  ::robolink::RobolinkJointInfo_<std::allocator<void> > RobolinkJointInfo;

typedef boost::shared_ptr< ::robolink::RobolinkJointInfo> RobolinkJointInfoPtr;
typedef boost::shared_ptr< ::robolink::RobolinkJointInfo const> RobolinkJointInfoConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::robolink::RobolinkJointInfo_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::robolink::RobolinkJointInfo_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace robolink

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::robolink::RobolinkJointInfo_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::robolink::RobolinkJointInfo_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::robolink::RobolinkJointInfo_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5c6db0f3a2e087a064b6118eefb17321";
  }

  static const char* value(const  ::robolink::RobolinkJointInfo_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5c6db0f3a2e087a0ULL;
  static const uint64_t static_value2 = 0x64b6118eefb17321ULL;
};

template<class ContainerAllocator>
struct DataType< ::robolink::RobolinkJointInfo_<ContainerAllocator> > {
  static const char* value() 
  {
    return "robolink/RobolinkJointInfo";
  }

  static const char* value(const  ::robolink::RobolinkJointInfo_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::robolink::RobolinkJointInfo_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# This message will be used monitor information specific to robolink joints\n\
# joint_name is a string representing the name of the joint\n\
# joint_num is the number of the joint\n\
# joint_angle is the angle of the joint in degrees\n\
# joint_velocity is the velocity of the joint in degrees per second\n\
\n\
\n\
string joint_name\n\
uint16 joint_num\n\
int32 joint_angle\n\
int32 joint_velocity\n\
time stamp\n\
\n\
";
  }

  static const char* value(const  ::robolink::RobolinkJointInfo_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::robolink::RobolinkJointInfo_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.joint_name);
    stream.next(m.joint_num);
    stream.next(m.joint_angle);
    stream.next(m.joint_velocity);
    stream.next(m.stamp);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct RobolinkJointInfo_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robolink::RobolinkJointInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::robolink::RobolinkJointInfo_<ContainerAllocator> & v) 
  {
    s << indent << "joint_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.joint_name);
    s << indent << "joint_num: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.joint_num);
    s << indent << "joint_angle: ";
    Printer<int32_t>::stream(s, indent + "  ", v.joint_angle);
    s << indent << "joint_velocity: ";
    Printer<int32_t>::stream(s, indent + "  ", v.joint_velocity);
    s << indent << "stamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.stamp);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ROBOLINK_MESSAGE_ROBOLINKJOINTINFO_H

