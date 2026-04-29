// Copyright (c) 2019 Intel Corporation
// Copyright (c) 2023 Davide Faconti
// Copyright (c) 2026 Sergio Garcia
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <rclcpp/executors.hpp>
#include <rclcpp/allocator/allocator_common.hpp>
#include <rclcpp/version.h>
#include <rclcpp/qos.hpp>
#include "behaviortree_cpp/bt_factory.h"

#include "behaviortree_ros2/ros_node_params.hpp"

namespace BT
{

/**
 * @brief Abstract class use to wrap rclcpp::Service<>
 *
 * RosServiceServerNode will return RUNNING while waiting for a request.
 * Once a request is received, it will be processed immediately in the ROS callback,
 * and the next tick() will return the resulting SUCCESS or FAILURE.
 *
 * The derived class must reimplement the virtual method processRequest.
 *
 * The name of the service will be determined as follows:
 *
 * 1. If a value is passes in the InputPort "service_name", use that
 * 2. Otherwise, use the value in RosNodeParams::default_port_value
 */
template <class ServiceT>
class RosServiceServerNode : public BT::ActionNodeBase
{
public:
  // Type definitions
  using ServiceServer = typename rclcpp::Service<ServiceT>;
  using ServiceServerPtr = std::shared_ptr<ServiceServer>;
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;

  /** To register this class into the factory, use:
   *
   *    factory.registerNodeType<>(node_name, params);
   */
  explicit RosServiceServerNode(const std::string& instance_name,
                                const BT::NodeConfig& conf,
                                const BT::RosNodeParams& params);

  virtual ~RosServiceServerNode() = default;

  /**
   * @brief Any subclass of RosServiceServerNode that has ports must implement a
   * providedPorts method and call providedBasicPorts in it.
   *
   * @param addition Additional ports to add to BT port list
   * @return PortsList containing basic ports along with node-specific ports
   */
  static PortsList providedBasicPorts(PortsList addition)
  {
    PortsList basic = { InputPort<std::string>("service_name", "", "Service name") };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return PortsList Containing basic ports along with node-specific ports
   */
  static PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  NodeStatus tick() override;

  void halt() override;

  /**
   * @brief Callback invoked when a request is received.
   * It is executed in the ROS executor thread.
   *
   * @param request  the request received from the client.
   * @param response the response to be sent back to the client.
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus processRequest(const typename Request::SharedPtr request,
                                        typename Response::SharedPtr response) = 0;

protected:
  // method to set the service name programmatically
  void setServiceName(const std::string& service_name);

  rclcpp::Logger logger()
  {
    if(auto node = node_.lock())
    {
      return node->get_logger();
    }
    return rclcpp::get_logger("RosServiceServerNode");
  }

  std::weak_ptr<rclcpp::Node> node_;
  std::string service_name_;
  bool service_name_should_be_checked_ = false;

private:
  ServiceServerPtr service_server_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::mutex request_mutex_;
  bool request_processed_ = false;
  BT::NodeStatus process_status_ = BT::NodeStatus::IDLE;

  bool createServer(const std::string& service_name);

  void serviceCallback(const typename Request::SharedPtr request,
                       typename Response::SharedPtr response);
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template <class T>
inline RosServiceServerNode<T>::RosServiceServerNode(const std::string& instance_name,
                                                     const NodeConfig& conf,
                                                     const RosNodeParams& params)
  : BT::ActionNodeBase(instance_name, conf), node_(params.nh)
{
  // check port remapping
  auto portIt = config().input_ports.find("service_name");
  if(portIt != config().input_ports.end())
  {
    const std::string& bb_service_name = portIt->second;

    if(isBlackboardPointer(bb_service_name))
    {
      // unknown value at construction time. postpone to tick
      service_name_should_be_checked_ = true;
    }
    else if(!bb_service_name.empty())
    {
      // "hard-coded" name in the bb_service_name. Use it.
      createServer(bb_service_name);
    }
  }
  if(!service_server_ && !params.default_port_value.empty())
  {
    createServer(params.default_port_value);
  }
}

template <class T>
inline bool RosServiceServerNode<T>::createServer(const std::string& service_name)
{
  if(service_name.empty() || service_name == "__default__placeholder__")
  {
    throw RuntimeError("service_name is empty or invalid");
  }

  auto node = node_.lock();
  if(!node)
  {
    throw RuntimeError("The ROS node went out of scope.");
  }

  callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);

  service_server_ = node->create_service<T>(
      service_name,
      [this](const typename Request::SharedPtr request,
             typename Response::SharedPtr response) {
        this->serviceCallback(request, response);
      },
      rmw_qos_profile_services_default, callback_group_);

  service_name_ = service_name;
  RCLCPP_INFO(logger(), "Node [%s] created service server [%s]", name().c_str(),
              service_name.c_str());
  return true;
}

template <class T>
inline void RosServiceServerNode<T>::setServiceName(const std::string& service_name)
{
  service_name_ = service_name;
  createServer(service_name);
}

template <class T>
inline void RosServiceServerNode<T>::serviceCallback(
    const typename Request::SharedPtr request, typename Response::SharedPtr response)
{
  std::lock_guard<std::mutex> lock(request_mutex_);

  if(status() != NodeStatus::RUNNING)
  {
    RCLCPP_WARN(logger(), "Node [%s] received request but is not RUNNING. Ignoring.",
                name().c_str());
    return;
  }

  if(request_processed_)
  {
    RCLCPP_WARN(logger(), "Node [%s] already processed a request, waiting for tick.",
                name().c_str());
    return;
  }

  // Process the request immediately
  process_status_ = processRequest(request, response);

  if(!isStatusCompleted(process_status_))
  {
    RCLCPP_ERROR(logger(), "Node [%s] processRequest must return SUCCESS or FAILURE",
                 name().c_str());
    process_status_ = NodeStatus::FAILURE;
  }

  request_processed_ = true;
}

template <class T>
inline NodeStatus RosServiceServerNode<T>::tick()
{
  if(!rclcpp::ok())
  {
    halt();
    return NodeStatus::FAILURE;
  }

  // First, check if the service_server is valid and that the name of the
  // service_name in the port didn't change.
  // otherwise, create a new server
  if(!service_server_ ||
     (status() == NodeStatus::IDLE && service_name_should_be_checked_))
  {
    std::string service_name;
    getInput("service_name", service_name);
    if(service_name_ != service_name)
    {
      createServer(service_name);
    }
  }

  if(!service_server_)
  {
    throw BT::RuntimeError("RosServiceServerNode: no service server was specified");
  }

  // first step to be done only at the beginning of the Action
  if(status() == BT::NodeStatus::IDLE)
  {
    std::lock_guard<std::mutex> lock(request_mutex_);
    request_processed_ = false;
    process_status_ = NodeStatus::IDLE;
    setStatus(NodeStatus::RUNNING);
    return NodeStatus::RUNNING;
  }

  if(status() == NodeStatus::RUNNING)
  {
    std::lock_guard<std::mutex> lock(request_mutex_);
    if(request_processed_)
    {
      request_processed_ = false;
      return process_status_;
    }
    return NodeStatus::RUNNING;
  }

  return NodeStatus::FAILURE;
}

template <class T>
inline void RosServiceServerNode<T>::halt()
{
  if(status() == NodeStatus::RUNNING)
  {
    std::lock_guard<std::mutex> lock(request_mutex_);
    request_processed_ = false;
    process_status_ = NodeStatus::IDLE;
    resetStatus();
  }
}

}  // namespace BT
