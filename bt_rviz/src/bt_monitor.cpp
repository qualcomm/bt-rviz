// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries
// SPDX-License-Identifier: BSD-3-Clause-Clear
#include "bt_monitor.hpp"

BTMonitor::BTMonitor(BT::Tree& tree, rclcpp::Node::SharedPtr node, const std::unordered_map<std::string, NodeData>& node_map)
    : tree_(tree), node_map_(node_map){
    publisher_ = node->create_publisher<bt_msgs::msg::BTNode>("/behavior_tree_updates", 10);
}

void BTMonitor::attachMonitoring() {
    for (auto& node : tree_.nodes) {
        status_subscribers_[node->name()] = node->subscribeToStatusChange(
            [this](std::chrono::time_point<std::chrono::system_clock> timestamp, 
                   const BT::TreeNode& node, 
                   BT::NodeStatus prev_status, 
                   BT::NodeStatus new_status) {
                this->onStatusChanged(node, new_status);
            }
        );
        
    }
}
void BTMonitor::onStatusChanged(const BT::TreeNode& node, BT::NodeStatus status) {
    auto msg = std::make_shared<bt_msgs::msg::BTNode>();

    std::string unique_name = node.name();
    for (const auto& [name, data] : node_map_) {
        if (data.name == unique_name) {
            unique_name = name;
            break;
        }
    }

    msg->name.data = unique_name;
    msg->type.data = toStr(node.type());
    msg->status.data = statusToString(status);

    if (node_map_.count(unique_name)) {
        msg->parent.data = node_map_[unique_name].parent;
    } else {
        msg->parent.data = "root";
    }

    if (node_map_.count(unique_name)) {
        for (const auto& child_name : node_map_[unique_name].children) {
            std_msgs::msg::String child_msg;
            child_msg.data = child_name;
            msg->children.push_back(child_msg);
        }
    }

    publisher_->publish(*msg);
}



std::string toStr(BT::NodeType type) {
    switch (type) {
        case BT::NodeType::ACTION: return "Action";
        case BT::NodeType::CONDITION: return "Condition";
        case BT::NodeType::DECORATOR: return "Decorator";
        case BT::NodeType::CONTROL: return "Control";
        default: return "Unknown";
    }
}


std::string BTMonitor::statusToString(BT::NodeStatus status) {
    switch (status) {
        case BT::NodeStatus::SUCCESS: return "SUCCESS";
        case BT::NodeStatus::FAILURE: return "FAILURE";
        case BT::NodeStatus::RUNNING: return "RUNNING";
        default: return "IDLE";
    }
}
