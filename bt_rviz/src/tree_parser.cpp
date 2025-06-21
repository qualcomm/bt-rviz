// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries
// SPDX-License-Identifier: BSD-3-Clause-Clear
#include "tree_parser.hpp"
#include <iostream>

using namespace tinyxml2;

std::unordered_map<std::string, NodeData> TreeParser::parseNodeTypes(const std::string& xml_string) {
    XMLDocument doc;
    doc.Parse(xml_string.c_str());

    std::unordered_map<std::string, NodeData> node_map;
    XMLElement* model_root = doc.FirstChildElement("root")->FirstChildElement("TreeNodesModel");

    for (XMLElement* elem = model_root->FirstChildElement(); elem != nullptr; elem = elem->NextSiblingElement()) {
        NodeData node;
        node.name = elem->Attribute("ID");
        node.type = elem->Name(); 
        for (XMLElement* input_elem = elem->FirstChildElement("input_port"); input_elem != nullptr; input_elem = input_elem->NextSiblingElement("input_port")) {
            const char* port_name = input_elem->Attribute("name");
            if (port_name) {
                node.input_ports.push_back(port_name);
            }
        }
        node_map[node.name] = node;
    }
    
    return node_map;
}
void TreeParser::parseTreeHierarchy(const std::vector<std::string>& xml_paths, std::unordered_map<std::string, NodeData>& node_map) {
    XMLDocument doc;
    if (doc.LoadFile(xml_paths[0].c_str()) != XML_SUCCESS) {
        std::cerr << "Error loading main XML file!" << std::endl;
        return;
    }

    XMLElement* root_bt = doc.FirstChildElement("root")->FirstChildElement("BehaviorTree");
    if (!root_bt) {
        std::cerr << "Main BehaviorTree element not found!" << std::endl;
        return;
    }

    std::string tree_name = root_bt->Attribute("ID");
    std::unordered_map<std::string, int> instance_count;

    std::function<void(XMLElement*, const std::string&)> traverse;
    traverse = [&](XMLElement* elem, const std::string& parent) {
        if (!elem) return;

        std::string type = elem->Name();
        std::string base_name;

        if (const char* name_attr = elem->Attribute("name")) base_name = name_attr;
        else if (const char* id_attr = elem->Attribute("ID")) base_name = id_attr;
        else base_name = type;

        int count = instance_count[base_name]++;
        std::string instance_name = (count == 0) ? base_name : base_name + "_" + std::to_string(count);

        if (type == "SubTree") {
            bool found = false;
            for (size_t i = 1; i < xml_paths.size(); ++i) {
                XMLDocument sub_doc;
                if (sub_doc.LoadFile(xml_paths[i].c_str()) != XML_SUCCESS) continue;

                for (XMLElement* bt = sub_doc.FirstChildElement("root")->FirstChildElement("BehaviorTree"); 
                     bt != nullptr; 
                     bt = bt->NextSiblingElement("BehaviorTree")) {

                    const char* bt_id = bt->Attribute("ID");
                    if (bt_id && base_name == std::string(bt_id)) {
                        found = true;
                        for (XMLElement* child = bt->FirstChildElement(); child != nullptr; child = child->NextSiblingElement()) {
                            traverse(child, parent);
                        }
                        break;
                    }
                }
                if (found) break;
            }

            if (!found) {
                NodeData& node = node_map[instance_name];
                node.name = instance_name;
                node.type = "SubTree";
                node.parent = parent;
                if (!parent.empty()) node_map[parent].children.push_back(instance_name);
            }

            return;
        }

        NodeData& node = node_map[instance_name];
        node.name = instance_name;
        node.type = type;
        if (!parent.empty()) {
            node.parent = parent;
            node_map[parent].children.push_back(instance_name);
        }

        if (node_map.count(base_name)) {
            for (const auto& port : node_map[base_name].input_ports) {
                const char* val = elem->Attribute(port.c_str());
                if (val) node.name += "_" + std::string(val);
            }
        }

        for (XMLElement* child = elem->FirstChildElement(); child != nullptr; child = child->NextSiblingElement()) {
            traverse(child, instance_name);
        }
    };

    traverse(root_bt, ""); 

    for (auto it = node_map.begin(); it != node_map.end();) {
        if (it->second.parent.empty() && it->second.children.empty()) {
            it = node_map.erase(it);
        } else {
            ++it;
        }
    }
}




void TreeParser::printNodes(const std::unordered_map<std::string, NodeData>& node_map) {
    for (const auto& [name, data] : node_map) {
        std::cout << "Node: " << name 
                  << ", Type: " << data.type 
                  << ", Parent: " << data.parent 
                  << ", Children: ";
        
        for (const auto& child : data.children) {
            std::cout << child << " ";
        }

        std::cout << ", Input Ports: ";
        if (!data.input_ports.empty()) {
            for (const auto& port : data.input_ports) {
                std::cout << port << " ";
            }
        } else {
            std::cout << "None";
        }

        std::cout << std::endl;
    }
}

void TreeParser::printConnectedNodes(const std::unordered_map<std::string, NodeData>& node_map) {
    for (const auto& [name, data] : node_map) {
        if (data.parent.empty() && data.children.empty()) {
            continue;
        }

        std::cout << "Node: " << name << ", Type: " << data.type << ", Parent: " << data.parent << ", Children: ";
        for (const auto& child : data.children) {
            std::cout << child << " ";
        }
        std::cout << std::endl;
    }
}

bt_msgs::msg::BT TreeParser::constructBTMessage(const std::unordered_map<std::string, NodeData>& node_map) {
    bt_msgs::msg::BT bt_msg;
    bt_msg.header.frame_id = "map";
    bt_msg.header.stamp = rclcpp::Clock().now(); 

    for (const auto& [name, data] : node_map) {
        if (data.parent.empty()) {
            bt_msg.root.data = name;
            break;
        } else{
            bt_msg.root.data = "root_sequence";
        }
    }

    for (const auto& [name, data] : node_map) {
        bt_msgs::msg::BTNode node_msg;
        node_msg.name.data = data.name;
        node_msg.type.data = data.type;
        node_msg.status.data = "IDLE";
        node_msg.parent.data = data.parent;

        for (const auto& child : data.children) {
            std_msgs::msg::String child_msg;
            child_msg.data = child;
            node_msg.children.push_back(child_msg);
        }

        bt_msg.nodes.push_back(node_msg);
    }

    return bt_msg;
}

void TreeParser::printBTMessage(const bt_msgs::msg::BT& bt_message) {
    std::cout << "Behavior Tree Root: " << bt_message.root.data << std::endl;
    std::cout << "Nodes:" << std::endl;

    for (const auto& node : bt_message.nodes) {
        std::cout << "  Name: " << node.name.data 
                  << ", Type: " << node.type.data 
                  << ", Status: " << node.status.data 
                  << ", Parent: " << node.parent.data 
                  << ", Children: ";

        for (const auto& child : node.children) {
            std::cout << child.data << " ";
        }
        std::cout << std::endl;
    }
}

std::vector<std::string> TreeParser::getAllInputPorts(const std::unordered_map<std::string, NodeData>& node_map) {
    std::vector<std::string> input_ports;

    for (const auto& [name, data] : node_map) {
        for (const auto& port : data.input_ports) {
            input_ports.push_back(port);
        }
    }

    return input_ports;
}

std::string TreeParser::generateUUID() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 15);

    std::stringstream uuid;
    uuid << std::hex;
    for (int i = 0; i < 8; ++i) uuid << dis(gen);
    uuid << "-";
    for (int i = 0; i < 4; ++i) uuid << dis(gen);
    uuid << "-";
    for (int i = 4; i < 6; ++i) uuid << dis(gen);
    uuid << "-";
    for (int i = 6; i < 8; ++i) uuid << dis(gen);
    uuid << "-";
    for (int i = 8; i < 12; ++i) uuid << dis(gen);
    
    return uuid.str();
}


