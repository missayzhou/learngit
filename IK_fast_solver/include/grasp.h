// Generated by gencpp from file IK_fast_solver/grasp.msg
// DO NOT EDIT!


#ifndef IK_FAST_SOLVER_MESSAGE_GRASP_H
#define IK_FAST_SOLVER_MESSAGE_GRASP_H

#include <ros/service_traits.h>


#include <graspRequest.h>
#include <graspResponse.h>


namespace IK_fast_solver
{

struct grasp
{

typedef graspRequest Request;
typedef graspResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct grasp
} // namespace IK_fast_solver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::IK_fast_solver::grasp > {
  static const char* value()
  {
    return "7cb80d94d4cc2b157db7b4f1f4b8ce60";
  }

  static const char* value(const ::IK_fast_solver::grasp&) { return value(); }
};

template<>
struct DataType< ::IK_fast_solver::grasp > {
  static const char* value()
  {
    return "IK_fast_solver/grasp";
  }

  static const char* value(const ::IK_fast_solver::grasp&) { return value(); }
};


// service_traits::MD5Sum< ::IK_fast_solver::graspRequest> should match 
// service_traits::MD5Sum< ::IK_fast_solver::grasp > 
template<>
struct MD5Sum< ::IK_fast_solver::graspRequest>
{
  static const char* value()
  {
    return MD5Sum< ::IK_fast_solver::grasp >::value();
  }
  static const char* value(const ::IK_fast_solver::graspRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::IK_fast_solver::graspRequest> should match 
// service_traits::DataType< ::IK_fast_solver::grasp > 
template<>
struct DataType< ::IK_fast_solver::graspRequest>
{
  static const char* value()
  {
    return DataType< ::IK_fast_solver::grasp >::value();
  }
  static const char* value(const ::IK_fast_solver::graspRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::IK_fast_solver::graspResponse> should match 
// service_traits::MD5Sum< ::IK_fast_solver::grasp > 
template<>
struct MD5Sum< ::IK_fast_solver::graspResponse>
{
  static const char* value()
  {
    return MD5Sum< ::IK_fast_solver::grasp >::value();
  }
  static const char* value(const ::IK_fast_solver::graspResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::IK_fast_solver::graspResponse> should match 
// service_traits::DataType< ::IK_fast_solver::grasp > 
template<>
struct DataType< ::IK_fast_solver::graspResponse>
{
  static const char* value()
  {
    return DataType< ::IK_fast_solver::grasp >::value();
  }
  static const char* value(const ::IK_fast_solver::graspResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // IK_FAST_SOLVER_MESSAGE_GRASP_H
