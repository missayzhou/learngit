// Generated by gencpp from file IK_fast_solver/graspRequest.msg
// DO NOT EDIT!


#ifndef IK_FAST_SOLVER_MESSAGE_GRASPREQUEST_H
#define IK_FAST_SOLVER_MESSAGE_GRASPREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace IK_fast_solver
{
template <class ContainerAllocator>
struct graspRequest_
{
  typedef graspRequest_<ContainerAllocator> Type;

  graspRequest_()
    : flag(0)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , q_w(0.0)
    , q_x(0.0)
    , q_y(0.0)
    , q_z(0.0)  {
    }
  graspRequest_(const ContainerAllocator& _alloc)
    : flag(0)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , q_w(0.0)
    , q_x(0.0)
    , q_y(0.0)
    , q_z(0.0)  {
  (void)_alloc;
    }



   typedef int32_t _flag_type;
  _flag_type flag;

   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;

   typedef double _q_w_type;
  _q_w_type q_w;

   typedef double _q_x_type;
  _q_x_type q_x;

   typedef double _q_y_type;
  _q_y_type q_y;

   typedef double _q_z_type;
  _q_z_type q_z;





  typedef boost::shared_ptr< ::IK_fast_solver::graspRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::IK_fast_solver::graspRequest_<ContainerAllocator> const> ConstPtr;

}; // struct graspRequest_

typedef ::IK_fast_solver::graspRequest_<std::allocator<void> > graspRequest;

typedef boost::shared_ptr< ::IK_fast_solver::graspRequest > graspRequestPtr;
typedef boost::shared_ptr< ::IK_fast_solver::graspRequest const> graspRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::IK_fast_solver::graspRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::IK_fast_solver::graspRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace IK_fast_solver

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::IK_fast_solver::graspRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::IK_fast_solver::graspRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::IK_fast_solver::graspRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::IK_fast_solver::graspRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::IK_fast_solver::graspRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::IK_fast_solver::graspRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::IK_fast_solver::graspRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2b10ed38fd3ffdc374345acdf1648792";
  }

  static const char* value(const ::IK_fast_solver::graspRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2b10ed38fd3ffdc3ULL;
  static const uint64_t static_value2 = 0x74345acdf1648792ULL;
};

template<class ContainerAllocator>
struct DataType< ::IK_fast_solver::graspRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "IK_fast_solver/graspRequest";
  }

  static const char* value(const ::IK_fast_solver::graspRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::IK_fast_solver::graspRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 flag\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 q_w\n\
float64 q_x\n\
float64 q_y\n\
float64 q_z\n\
";
  }

  static const char* value(const ::IK_fast_solver::graspRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::IK_fast_solver::graspRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.flag);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.q_w);
      stream.next(m.q_x);
      stream.next(m.q_y);
      stream.next(m.q_z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct graspRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::IK_fast_solver::graspRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::IK_fast_solver::graspRequest_<ContainerAllocator>& v)
  {
    s << indent << "flag: ";
    Printer<int32_t>::stream(s, indent + "  ", v.flag);
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "q_w: ";
    Printer<double>::stream(s, indent + "  ", v.q_w);
    s << indent << "q_x: ";
    Printer<double>::stream(s, indent + "  ", v.q_x);
    s << indent << "q_y: ";
    Printer<double>::stream(s, indent + "  ", v.q_y);
    s << indent << "q_z: ";
    Printer<double>::stream(s, indent + "  ", v.q_z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // IK_FAST_SOLVER_MESSAGE_GRASPREQUEST_H
