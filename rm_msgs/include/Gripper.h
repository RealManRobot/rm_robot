// Generated by gencpp from file rm_msgs/Gripper.msg
// DO NOT EDIT!


#ifndef RM_MSGS_MESSAGE_GRIPPER_H
#define RM_MSGS_MESSAGE_GRIPPER_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rm_msgs
{
template <class ContainerAllocator>
struct Gripper_
{
  typedef Gripper_<ContainerAllocator> Type;

  Gripper_()
    : state(false)
    , speed(0)
    , force(0)  {
    }
  Gripper_(const ContainerAllocator& _alloc)
    : state(false)
    , speed(0)
    , force(0)  {
  (void)_alloc;
    }



   typedef uint8_t _state_type;
  _state_type state;

   typedef int16_t _speed_type;
  _speed_type speed;

   typedef int16_t _force_type;
  _force_type force;





  typedef boost::shared_ptr< ::rm_msgs::Gripper_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rm_msgs::Gripper_<ContainerAllocator> const> ConstPtr;

}; // struct Gripper_

typedef ::rm_msgs::Gripper_<std::allocator<void> > Gripper;

typedef boost::shared_ptr< ::rm_msgs::Gripper > GripperPtr;
typedef boost::shared_ptr< ::rm_msgs::Gripper const> GripperConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rm_msgs::Gripper_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rm_msgs::Gripper_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rm_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'rm_msgs': ['/home/nvidia/catkin_ws/src/RM_Robot/rm_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rm_msgs::Gripper_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rm_msgs::Gripper_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rm_msgs::Gripper_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rm_msgs::Gripper_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rm_msgs::Gripper_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rm_msgs::Gripper_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rm_msgs::Gripper_<ContainerAllocator> >
{
  static const char* value()
  {
    return "58ed04654b29265448d8d6f3dc59540a";
  }

  static const char* value(const ::rm_msgs::Gripper_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x58ed04654b292654ULL;
  static const uint64_t static_value2 = 0x48d8d6f3dc59540aULL;
};

template<class ContainerAllocator>
struct DataType< ::rm_msgs::Gripper_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rm_msgs/Gripper";
  }

  static const char* value(const ::rm_msgs::Gripper_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rm_msgs::Gripper_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool state      #0-open gripper, 1-close gripper\n\
int16 speed\n\
int16 force\n\
";
  }

  static const char* value(const ::rm_msgs::Gripper_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rm_msgs::Gripper_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.state);
      stream.next(m.speed);
      stream.next(m.force);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Gripper_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rm_msgs::Gripper_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rm_msgs::Gripper_<ContainerAllocator>& v)
  {
    s << indent << "state: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.state);
    s << indent << "speed: ";
    Printer<int16_t>::stream(s, indent + "  ", v.speed);
    s << indent << "force: ";
    Printer<int16_t>::stream(s, indent + "  ", v.force);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RM_MSGS_MESSAGE_GRIPPER_H
