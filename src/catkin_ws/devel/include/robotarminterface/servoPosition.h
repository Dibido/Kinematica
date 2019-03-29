// Generated by gencpp from file robotarminterface/servoPosition.msg
// DO NOT EDIT!


#ifndef ROBOTARMINTERFACE_MESSAGE_SERVOPOSITION_H
#define ROBOTARMINTERFACE_MESSAGE_SERVOPOSITION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robotarminterface
{
template <class ContainerAllocator>
struct servoPosition_
{
  typedef servoPosition_<ContainerAllocator> Type;

  servoPosition_()
    : servoId(0)
    , position(0)  {
    }
  servoPosition_(const ContainerAllocator& _alloc)
    : servoId(0)
    , position(0)  {
  (void)_alloc;
    }



   typedef uint32_t _servoId_type;
  _servoId_type servoId;

   typedef int32_t _position_type;
  _position_type position;





  typedef boost::shared_ptr< ::robotarminterface::servoPosition_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robotarminterface::servoPosition_<ContainerAllocator> const> ConstPtr;

}; // struct servoPosition_

typedef ::robotarminterface::servoPosition_<std::allocator<void> > servoPosition;

typedef boost::shared_ptr< ::robotarminterface::servoPosition > servoPositionPtr;
typedef boost::shared_ptr< ::robotarminterface::servoPosition const> servoPositionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robotarminterface::servoPosition_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robotarminterface::servoPosition_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace robotarminterface

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'robotarminterface': ['/home/owner/git/Kinematica/src/catkin_ws/src/kinematics/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::robotarminterface::servoPosition_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotarminterface::servoPosition_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotarminterface::servoPosition_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotarminterface::servoPosition_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotarminterface::servoPosition_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotarminterface::servoPosition_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robotarminterface::servoPosition_<ContainerAllocator> >
{
  static const char* value()
  {
    return "21c24bd1e99f0c44d572dd36095ff06f";
  }

  static const char* value(const ::robotarminterface::servoPosition_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x21c24bd1e99f0c44ULL;
  static const uint64_t static_value2 = 0xd572dd36095ff06fULL;
};

template<class ContainerAllocator>
struct DataType< ::robotarminterface::servoPosition_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robotarminterface/servoPosition";
  }

  static const char* value(const ::robotarminterface::servoPosition_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robotarminterface::servoPosition_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 servoId\n\
int32 position\n\
";
  }

  static const char* value(const ::robotarminterface::servoPosition_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robotarminterface::servoPosition_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.servoId);
      stream.next(m.position);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct servoPosition_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robotarminterface::servoPosition_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robotarminterface::servoPosition_<ContainerAllocator>& v)
  {
    s << indent << "servoId: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.servoId);
    s << indent << "position: ";
    Printer<int32_t>::stream(s, indent + "  ", v.position);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTARMINTERFACE_MESSAGE_SERVOPOSITION_H