// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries
// SPDX-License-Identifier: BSD-3-Clause-Clear
#include <rclcpp/rclcpp.hpp>
#include <bt_msgs/msg/bt_node.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/tree_node.h>
#include "tree_parser.hpp"

class BTMonitor {
    public:
        BTMonitor(BT::Tree& tree, rclcpp::Node::SharedPtr node, const std::unordered_map<std::string, NodeData>& node_map);
        void attachMonitoring();
    
    private:
        BT::Tree& tree_;
        rclcpp::Publisher<bt_msgs::msg::BTNode>::SharedPtr publisher_;
        std::unordered_map<std::string, BT::TreeNode::StatusChangeSubscriber> status_subscribers_;
    
        void onStatusChanged(const BT::TreeNode& node, BT::NodeStatus status);
        std::string statusToString(BT::NodeStatus status);
        std::unordered_map<std::string, NodeData> node_map_;
    };
    