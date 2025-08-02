// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ufactory_ellmer_msgs:srv/SetInt16.idl
// generated code does not contain a copyright notice

#ifndef UFACTORY_ELLMER_MSGS__SRV__DETAIL__SET_INT16__BUILDER_HPP_
#define UFACTORY_ELLMER_MSGS__SRV__DETAIL__SET_INT16__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ufactory_ellmer_msgs/srv/detail/set_int16__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ufactory_ellmer_msgs
{

namespace srv
{

namespace builder
{

class Init_SetInt16_Request_data
{
public:
  Init_SetInt16_Request_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ufactory_ellmer_msgs::srv::SetInt16_Request data(::ufactory_ellmer_msgs::srv::SetInt16_Request::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ufactory_ellmer_msgs::srv::SetInt16_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ufactory_ellmer_msgs::srv::SetInt16_Request>()
{
  return ufactory_ellmer_msgs::srv::builder::Init_SetInt16_Request_data();
}

}  // namespace ufactory_ellmer_msgs


namespace ufactory_ellmer_msgs
{

namespace srv
{

namespace builder
{

class Init_SetInt16_Response_message
{
public:
  explicit Init_SetInt16_Response_message(::ufactory_ellmer_msgs::srv::SetInt16_Response & msg)
  : msg_(msg)
  {}
  ::ufactory_ellmer_msgs::srv::SetInt16_Response message(::ufactory_ellmer_msgs::srv::SetInt16_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ufactory_ellmer_msgs::srv::SetInt16_Response msg_;
};

class Init_SetInt16_Response_ret
{
public:
  Init_SetInt16_Response_ret()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetInt16_Response_message ret(::ufactory_ellmer_msgs::srv::SetInt16_Response::_ret_type arg)
  {
    msg_.ret = std::move(arg);
    return Init_SetInt16_Response_message(msg_);
  }

private:
  ::ufactory_ellmer_msgs::srv::SetInt16_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ufactory_ellmer_msgs::srv::SetInt16_Response>()
{
  return ufactory_ellmer_msgs::srv::builder::Init_SetInt16_Response_ret();
}

}  // namespace ufactory_ellmer_msgs

#endif  // UFACTORY_ELLMER_MSGS__SRV__DETAIL__SET_INT16__BUILDER_HPP_
