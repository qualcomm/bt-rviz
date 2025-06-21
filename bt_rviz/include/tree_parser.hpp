// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries
// SPDX-License-Identifier: BSD-3-Clause-Clear
#ifndef TREE_PARSER_HPP
#define TREE_PARSER_HPP

#include <tinyxml2.h>
#include <unordered_map>
#include <vector>
#include <string>
#include <functional>
#include "bt_msgs/msg/bt.hpp"
#include "bt_msgs/msg/bt_node.hpp"
#include "std_msgs/msg/string.hpp"
#include <rclcpp/rclcpp.hpp>
#include <random>
#include <sstream>


struct NodeData {
    std::string name;
    std::string type;
    std::vector<std::string> children;
    std::string parent;
    std::vector<std::string> input_ports;
};

class TreeParser {
public:
    TreeParser() = default;

    std::unordered_map<std::string, NodeData> parseNodeTypes(const std::string& xml_string);

    void parseTreeHierarchy(const std::vector<std::string>& xml_paths, std::unordered_map<std::string, NodeData>& node_map);

    void printNodes(const std::unordered_map<std::string, NodeData>& node_map);
    void printConnectedNodes(const std::unordered_map<std::string, NodeData>& node_map);
    bt_msgs::msg::BT constructBTMessage(const std::unordered_map<std::string, NodeData>& node_map);
    void printBTMessage(const bt_msgs::msg::BT& bt_message);
    std::vector<std::string> getAllInputPorts(const std::unordered_map<std::string, NodeData>& node_map);
    std::string generateUUID();

};

#endif // TREE_PARSER_HPP
