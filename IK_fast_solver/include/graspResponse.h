// Generated by gencpp from file IK_fast_solver/graspResponse.msg
// DO NOT EDIT!


#ifndef IK_FAST_SOLVER_MESSAGE_GRASPRESPONSE_H
#define IK_FAST_SOLVER_MESSAGE_GRASPRESPONSE_H


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
struct graspResponse_
{
  typedef graspResponse_<ContainerAllocator> Type;

  graspResponse_()
    : angle0(0.0)
    , angle1(0.0)
    , angle2(0.0)
    , angle3(0.0)
    , angle4(0.0)
    , angle5(0.0)
    , angle6(0.0)  {
    }
  graspResponse_(const ContainerAllocator& _alloc)
    : angle0(0.0)
    , angle1(0.0)
    , angle2(0.0)
    , angle3(0.0)
    , angle4(0.0)
    , angle5(0.0)
    , angle6(0.0)  {
  (void)_alloc;
    }



   typedef double _angle0_type;
  _angle0_type angle0;

   typedef double _angle1_type;
  _angle1_type angle1;

   typedef double _angle2_type;
  _angle2_type angle2;

   typedef double _angle3_type;
  _angle3_type angle3;

   typedef double _angle4_type;
  _angle4_type angle4;

   typedef double _angle5_type;
  _angle5_type angle5;

   typedef double _angle6_type;
  _angle6_type angle6;





  typedef boost::shared_ptr< ::IK_fast_solver::graspResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::IK_fast_solver::graspResponse_<ContainerAllocator> const> ConstPtr;

}; // struct graspResponse_

typedef ::IK_fast_solver::graspResponse_<std::allocator<void> > graspResponse;

typedef boost::shared_ptr< ::IK_fast_solver::graspResponse > graspResponsePtr;
typedef boost::shared_ptr< ::IK_fast_solver::graspResponse const> graspResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::IK_fast_solver::graspResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::IK_fast_solver::graspResponse_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::IK_fast_solver::graspResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::IK_fast_solver::graspResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::IK_fast_solver::graspResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::IK_fast_solver::graspResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::IK_fast_solver::graspResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::IK_fast_solver::graspResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::IK_fast_solver::graspResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "82574c7ca86f14f2643ddd253664b083";
  }

  static const char* value(const ::IK_fast_solver::graspResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x82574c7ca86f14f2ULL;
  static const uint64_t static_value2 = 0x643ddd253664b083ULL;
};

template<class ContainerAllocator>
struct DataType< ::IK_fast_solver::graspResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "IK_fast_solver/graspResponse";
  }

  static const char* value(const ::IK_fast_solver::graspResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::IK_fast_solver::graspResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 angle0\n\
float64 angle1\n\
float64 angle2\n\
float64 angle3\n\
float64 angle4\n\
float64 angle5\n\
float64 angle6\n\
\n\
";
  }

  static const char* value(const ::IK_fast_solver::graspResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::IK_fast_solver::graspResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.angle0);
      stream.next(m.angle1);
      stream.next(m.angle2);
      stream.next(m.angle3);
      stream.next(m.angle4);
      stream.next(m.angle5);
      stream.next(m.angle6);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct graspResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::IK_fast_solver::graspResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::IK_fast_solver::graspResponse_<ContainerAllocator>& v)
  {
    s << indent << "angle0: ";
    Printer<double>::stream(s, indent + "  ", v.angle0);
    s << indent << "angle1: ";
    Printer<double>::stream(s, indent + "  ", v.angle1);
    s << indent << "angle2: ";
    Printer<double>::stream(s, indent + "  ", v.angle2);
    s << indent << "angle3: ";
    Printer<double>::stream(s, indent + "  ", v.angle3);
    s << indent << "angle4: ";
    Printer<double>::stream(s, indent + "  ", v.angle4);
    s << indent << "angle5: ";
    Printer<double>::stream(s, indent + "  ", v.angle5);
    s << indent << "angle6: ";
    Printer<double>::stream(s, indent + "  ", v.angle6);
  }
};

} // namespace message_operations
} // namespace ros

#endif // IK_FAST_SOLVER_MESSAGE_GRASPRESPONSE_H
