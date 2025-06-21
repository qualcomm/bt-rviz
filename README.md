# BT-RVIZ: plugin for behavior tree monitoring

This project allows users to visualise their behavior tree (including subtrees!) using ROS2 messages and RVIZ. 
It is a helpful tool for debugging the tree structure and identify any flows in the execution itself.

## Branches

**main**: Primary development branch.

## Requirements

The following must be installed:
- ROS2 Humble
- BehaviorTree.CPP
- RVIZ

## Installation Instructions

Clone the repository to your ROS2 workspace.

To install dependencies, run rosdep from the workspace root.

```
rosdep install --from-paths src --ignore-src
```

Build the packages.

```
colcon build
```

## Usage

1. Create a parser
Include the tree parser in your file:
```cpp
#include <tree_parser.hpp>
```
Then, create a Tree Parser object with the name `parser`:
```
TreeParser parser;
```

2. Generate Tree Nodes Model
This functionality is already provided in a BehaviorTree.CPP package. To use it:
```cpp
std::string xml_models = BT::writeTreeNodesModelXML(factory_);
std::ofstream file("tree_nodes_model.xml");
if (file.is_open()) {
    file << xml_models; 
    file.close();
} else {
    std::cerr << "Error: Unable to open file for writing!" << std::endl;
}
```

3. Create node map with all types
Use the `parser` created earlier to get node types from the Tree Nodes Model string `xml_models`:
```cpp
auto nodes = parser.parseNodeTypes(xml_models);
```

4. Parse the tree from XMLs
Since now all the types are known, we can create a tree using the XML files where the hierarchy is defined.
The function accepts the `std::vector<std::string>`, a list of paths to XML files, which allows to parse the subtrees too.
The first element in the array must be the main tree.
```cpp
std::vector<std::string> xml_paths = {
        "main_tree.xml",
        "subtree1.xml",
        "subtree2.xml",
        "subtree3.xml",
    };
parser.parseTreeHierarchy(xml_paths, nodes);
```

5. Construct BT message
> Make sure to build bt_msgs provided in this repository, as the type is custom.
First, include the header:
```cpp
#include "bt_msgs/msg/bt.hpp"
```
Then, use parser along with the updated node map to generate a message and store it:
```cpp
bt_msgs::msg::BT bt_message = parser.constructBTMessage(nodes);
```
This message doesn't have to be published often, should be published at least once in the beginning and, if needed, be updated infrequently.

6. Monitor the tree
Include the BTMonitor in your file:
```cpp
#include <bt_monitor.hpp>
```
To create a monitor object, you will need tyo create a node in your main file, which will be passed to the constructor with the tree and nodes map.
The tree refers to the normal BehaviorTree.CPP setup tree: 
```cpp
auto tree = factory_.createTree("MainTree");
```
Then, create a monitor and attach it:
```cpp
rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("bt_monitor_node");
BTMonitor monitor(tree, node, nodes);
```

After all this is done, you can start the executor.

## Debugging

- Make sure to have a name for all nodes in XML
All tags, including sequences and fallbacks should hjave a `name=""` field. The tags that do not have this property should have an `ID=""`.
Other nodes will be considered custom types and may cause issues.

- Verify the paths to your XMLs are correct

- Print available nodes
TreeParser allows you to print all the nodes in the map, for this, use:
```cpp
void TreeParser::printNodes(const std::unordered_map<std::string, NodeData>& node_map);
```
To make sure you do not have any unconnected nodes, you can print only connected ones:
```cpp
void TreeParser::printConnectedNodes(const std::unordered_map<std::string, NodeData>& node_map);
```

- Print the constructed message
The parser alsdo provides a way to print a BT message from bt_msgs, allowing you to see the:
name, type, status, parent, children of all nodes in the tree and the root of the tree.
```cpp
void TreeParser::printBTMessage(const bt_msgs::msg::BT& bt_message);
```

## Development

If you wish to contribute, check:
[CONTRIBUTING.md file](CONTRIBUTING.md).

## Getting in Contact

* [Report an Issue on GitHub](../../issues)
* [E-mail us](mailto:ssarana@qti.qualcomm.com) for general questions

## License

*bt-rviz* is licensed under the [BSD-3-clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE.txt](LICENSE.txt) for the full license text.
