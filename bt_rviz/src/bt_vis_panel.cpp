// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries
// SPDX-License-Identifier: BSD-3-Clause-Clear
#include "bt_vis_panel.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace bt_rviz {

    BTVisPanel::BTVisPanel(QWidget* parent)
    : rviz_common::MessageFilterDisplay<bt_msgs::msg::BT>()
{
    RCLCPP_INFO(rclcpp::get_logger("bt_visualizer_node"), "BTVisPanel Constructor Called");
    QWidget* widget = new QWidget();
    widget->setMouseTracking(true);

    RCLCPP_INFO(rclcpp::get_logger("bt_visualizer_node"), "mouse tracking set");
    this->installEventFilter(this);
    RCLCPP_INFO(rclcpp::get_logger("bt_visualizer_node"), "event filter set");

    QVBoxLayout* layout = new QVBoxLayout(widget);
    label_ = new QLabel("Waiting for Behavior Tree messages...");
    layout->addWidget(label_);
    widget->setLayout(layout);
    node_ = rclcpp::Node::make_shared("bt_visualizer_node");
    nodeShapeMap = {
        {"ReactiveSequence", "Sphere"},
        {"SequenceWithMemory", "Sphere"},
        {"Sequence", "Sphere"},
        {"IfThenElse", "Sphere"},
        {"WhileDoElse", "Sphere"},
        {"Fallback", "Sphere"},
        {"ReactiveFallback", "Sphere"},
        {"Action", "Cube"},
        {"SubTree", "Cube"},
        {"Condition", "Cylinder"},
        {"Parallel", "Cone"},
        {"Inverter", "Cone"},
        {"Delay", "Cone"},
        {"Repeat", "Cone"},
        {"Retry", "Cone"}
    };
}

void BTVisPanel::onInitialize() {
    MFDClass::onInitialize();
    scene_manager_ = context_->getSceneManager();
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    manual_object_ = scene_manager_->createManualObject("bt_tree_visual");
    render_panel_ = context_->getViewManager()->getRenderPanel();
    //render_panel_->setFocusPolicy(Qt::StrongFocus);
    render_panel_->setMouseTracking(true);
    render_panel_->installEventFilter(this);
    rviz_rendering::RenderWindow* render_window_ = render_panel_->getRenderWindow();
    view_controller_ = context_->getViewManager()->getCurrent();
    
    camera_ = scene_manager_->getCameraIterator().getNext();
    if (camera_) {
        RCLCPP_INFO(node_->get_logger(), "Retrieved Camera: %s", camera_->getName().c_str());
    } else {
        RCLCPP_ERROR(node_->get_logger(), "No camera found in SceneManager!");
    }
    scene_node_->attachObject(manual_object_);
    topic_property_ = new rviz_common::properties::RosTopicProperty(
        "Topic", "/behavior_tree",
        "", "", this);
    setTopic(QString("/behavior_tree"), QString("bt_msgs/msg/BT"));
    bt_subscriber_  = node_->create_subscription<bt_msgs::msg::BTNode>("/behavior_tree_updates", 10, std::bind(&BTVisPanel::processUpdateMessage, this, std::placeholders::_1));
    spinner_thread_ = std::thread([this]() {
        rclcpp::spin(node_);
    });
}

Ogre::AxisAlignedBox BTVisPanel::get_active_bbox() {
    Ogre::AxisAlignedBox combined_bbox;
    bool first_node = true;

    for (auto& [node_name, shape] : node_map_) {
        for (const auto& node : latest_msg_->nodes) {
            if(node_name == node.name.data && node.status.data == "IDLE"){
                Ogre::AxisAlignedBox bbox = shape->getEntity()->getBoundingBox();
                bbox.transform(shape->getRootNode()->_getFullTransform());

                if (first_node) {
                    combined_bbox = bbox;
                    first_node = false;
                } else {
                    combined_bbox.merge(bbox);
                }
            }
        }
    }
    return combined_bbox;

}

void BTVisPanel::processMessage(const bt_msgs::msg::BT::ConstSharedPtr msg) {
    latest_msg_ = std::make_shared<bt_msgs::msg::BT>(*msg);
}

void BTVisPanel::update(float wall_dt, float ros_dt) {
    if (!latest_msg_) return;
    if (!manual_object_) {
        RCLCPP_ERROR(node_->get_logger(), "manual_object_ is NULL before calling begin()!");
        return;
    }
    if (!scene_node_->numAttachedObjects()) {
        manual_object_->clear();
        manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
    }

    std::unordered_map<std::string, Ogre::Vector3> node_positions;
    std::unordered_set<std::string> processed_nodes;

    float root_x = 0.0, root_y = 1.0;
    node_positions[latest_msg_->root.data] = Ogre::Vector3(root_x, root_y, 0.0);

    if (cached_tree_ != latest_msg_) {
        next_x_ = 0.0;
        recursivelyPlaceNodes(latest_msg_->root.data, 0, processed_nodes);
        cached_tree_ = latest_msg_;
    }

    if (cached_tree_ != latest_msg_) {
        for (auto* line : line_map_) {
            delete line;
        }
        line_map_.clear();

        for (const auto& node : latest_msg_->nodes) {
            if (!node_positions.count(node.parent.data)) continue;

            auto* line = new rviz_rendering::Line(scene_manager_, scene_node_);
            line->setPoints(node_positions[node.name.data], node_positions[node.parent.data]);
            line->setColor(Ogre::ColourValue::White);
            line->setVisible(true);
            line_map_.push_back(line); 
        }
    }

    if (manual_object_->getNumSections() > 0) {
        manual_object_->end();
    }
    
}

bool BTVisPanel::eventFilter(QObject* object, QEvent* event) {
    if (event->type() == QEvent::CursorChange) {
        QPoint localPos = render_panel_->mapFromGlobal(QCursor::pos());
        checkHover(localPos);
        
        return false;  
    }
    return QObject::eventFilter(object, event);
}


float BTVisPanel::recursivelyPlaceNodes(const std::string& parent_name, int level,
                                       std::unordered_set<std::string>& processed_nodes)
{
    float horizontal_spacing = 1.0;
    float vertical_spacing = 1.0;
    float child_y = -level * vertical_spacing;

    std::vector<std::string> children;
    for (const auto& node : latest_msg_->nodes) {
        if (node.parent.data == parent_name) {
            children.push_back(node.name.data);
        }
    }

    if (children.empty()) {
        float x = next_x_;
        node_positions[parent_name] = Ogre::Vector3(x, child_y, 0.0f);
        drawNode(x, child_y, parent_name, getColorByStatus(parent_name), nodeShapeMap[findNodeByName(parent_name, latest_msg_).type.data]);
        processed_nodes.insert(parent_name);
        next_x_ += horizontal_spacing;
        return x;
    }

    std::vector<float> child_x_positions;
    for (const auto& child : children) {
        float child_x = recursivelyPlaceNodes(child, level + 1, processed_nodes);
        child_x_positions.push_back(child_x);
    }

    float avg_x = std::accumulate(child_x_positions.begin(), child_x_positions.end(), 0.0f) / child_x_positions.size();
    node_positions[parent_name] = Ogre::Vector3(avg_x, child_y, 0.0f);

    drawNode(avg_x, child_y, parent_name, getColorByStatus(parent_name), nodeShapeMap[findNodeByName(parent_name, latest_msg_).type.data]);
    processed_nodes.insert(parent_name);

    int i = 0;
    for (const auto& child : children) {
        auto* line = new rviz_rendering::Line(scene_manager_, scene_node_);
        line->setPoints(node_positions[child], node_positions[parent_name]);  
        line->setColor(Ogre::ColourValue::White); 
        line->setVisible(true);
        line_map_.push_back(line); 
        ++i;
    }    

    return avg_x;
}


void BTVisPanel::drawNode(float x, float y, const std::string& name, const Ogre::ColourValue& color, const std::string& type) {
    Ogre::SceneNode* shape_node = scene_manager_->getRootSceneNode()->createChildSceneNode(name);
    rviz_rendering::Shape* cylinder = nullptr;
    if (type == "Sphere") 
        cylinder = new rviz_rendering::Shape(rviz_rendering::Shape::Sphere, scene_manager_, shape_node);
    else if (type == "Cube") 
        cylinder = new rviz_rendering::Shape(rviz_rendering::Shape::Cube, scene_manager_, shape_node);
    else if (type == "Cone") 
        cylinder = new rviz_rendering::Shape(rviz_rendering::Shape::Cone, scene_manager_, shape_node);
    else 
        cylinder = new rviz_rendering::Shape(rviz_rendering::Shape::Cylinder, scene_manager_, shape_node);
    cylinder->setColor(color.r, color.g, color.b, 1.0); 
    shape_node->setPosition(Ogre::Vector3(x, y, 0));
    cylinder->setScale(Ogre::Vector3(0.3, 0.3, 0.3)); 
    
    // Attach text label
    rviz_rendering::MovableText* text = new rviz_rendering::MovableText(name);
    text->setCharacterHeight(0.4);
    Ogre::ColourValue customColor(0.6, 0.2, 0.7, 1.0);
    text->setColor(customColor);


    Ogre::SceneNode* text_node = cylinder->getRootNode()->createChildSceneNode();
    text_node->attachObject(text);
    text_node->setPosition(Ogre::Vector3(-0.25, 0, 0.65));

    node_map_[name] = cylinder;
    text_map_[name] = text;

}

void BTVisPanel::redrawNode(const std::string& node_name) {
    auto pos_it = node_positions.find(node_name);
    if (pos_it == node_positions.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Node position not found for: %s", node_name.c_str());
        return;
    }

    float x = pos_it->second.x;
    float y = pos_it->second.y;

    auto shape_it = node_map_.find(node_name);
    if (shape_it == node_map_.end() || !shape_it->second) {
        RCLCPP_ERROR(node_->get_logger(), "Node shape not found for: %s", node_name.c_str());
        return;
    }

    Ogre::MaterialPtr material = shape_it->second->getMaterial();
    material->setDiffuse(getColorByStatus(node_name));
    material->load();
    material->compile();
    material->setLightingEnabled(true);

}


bt_msgs::msg::BTNode BTVisPanel::findNodeByName(const std::string& name, const bt_msgs::msg::BT::SharedPtr latest_msg_) {
    for (const auto& node : latest_msg_->nodes) {
        if (node.name.data == name) {
            return node;
        }
    }
    throw std::runtime_error("Node with name '" + name + "' not found.");
}

Ogre::ColourValue BTVisPanel::getColorByStatus(const std::string& name) {
    try {
        auto node = findNodeByName(name, latest_msg_);
        const std::string& status = node.status.data;

        if (status == "RUNNING") return Ogre::ColourValue::Blue;
        if (status == "SUCCESS") return Ogre::ColourValue::Green;
        if (status == "FAILURE") return Ogre::ColourValue::Red;
        return Ogre::ColourValue::White; 
    } catch (...) {
        return Ogre::ColourValue::White;
    }
}

void BTVisPanel::processUpdateMessage(const bt_msgs::msg::BTNode::SharedPtr msg) {
    if (!latest_msg_) {
        return;
    }
    
    if (!msg) {
        RCLCPP_ERROR(node_->get_logger(), "Received NULL BTNode message!");
        return;
    }

    bool node_found = false;
    std::string new_parallel_parent = findParallelParent(msg->name.data);
    std::string last_parallel_parent = findParallelParent(last_updated_node_);

    for (auto& node : latest_msg_->nodes) {
        if (node.name.data == msg->name.data && node.parent.data == msg->parent.data) {
            if (!last_updated_node_.empty() && last_updated_node_ != msg->name.data) {
                if (last_parallel_parent != new_parallel_parent) {  
                    for (const auto& node_name : non_idle_nodes_) {
                        for (auto& node : latest_msg_->nodes) {
                            if (node.name.data == node_name) {
                                node.status.data = "IDLE";
                                redrawNode(node_name);
                                break;
                            }
                        }
                    }
                }
            }

            node.status.data = msg->status.data;
            last_updated_node_ = msg->name.data;
            last_updated_parent_ = msg->parent.data;
            non_idle_nodes_.insert(msg->name.data);
            node_found = true;

            Ogre::AxisAlignedBox box = this->get_active_bbox();
            Ogre::Vector3 center = box.getCenter();
            view_controller_->lookAt(center);
            float distance = box.getSize().length() * 0.8;
            view_controller_->subProp("Distance")->setValue(distance);
            break;
        }
    }

    if (node_found) {
        redrawNode(msg->name.data);
    }
}



std::string BTVisPanel::findParallelParent(const std::string& node_name) {
    std::string current_parent = "";
    
    for (const auto& node : latest_msg_->nodes) {
        if (node.name.data == node_name) {
            current_parent = node.parent.data;
            break;
        }
    }

    while (!current_parent.empty()) {
        for (const auto& node : latest_msg_->nodes) {
            if (node.name.data == current_parent) {
                if (node.type.data == "Parallel") {
                    return current_parent; 
                }
                current_parent = node.parent.data; 
                break;
            }
        }
    }

    return "";

void BTVisPanel::checkHover(const QPoint& mousePosition) {
    float viewportX = static_cast<float>(mousePosition.x()) / render_panel_->width();
    float viewportY = static_cast<float>(mousePosition.y()) / render_panel_->height();
    Ogre::Ray mouseRay = camera_->getCameraToViewportRay(viewportX, viewportY);

    Ogre::RaySceneQuery* query = scene_manager_->createRayQuery(mouseRay);
    query->setQueryTypeMask(Ogre::SceneManager::ENTITY_TYPE_MASK);
    Ogre::RaySceneQueryResult& result = query->execute();
    for (auto& entry : result) {
        if (std::string(entry.movable->getName()) == "Grid0") {
            continue;
        }        
        Ogre::Vector3 hit_point = entry.movable->getWorldBoundingBox().getCenter();
        Ogre::Vector3 target_pos = entry.movable->getParentSceneNode()->_getDerivedPosition();
        Ogre::SceneNode* node = entry.movable->getParentSceneNode();

        std::string closest_node = "";
        float min_distance = std::numeric_limits<float>::max();

        for (auto& [node_name, shape] : node_map_) {
            Ogre::Vector3 node_pos = node_positions[node_name];
            float distance = (target_pos - node_pos).length();
            if (distance < min_distance) {
                min_distance = distance;
                closest_node = node_name;
            }
        }

        // if (closest_node != "") {
        //     RCLCPP_INFO(node_->get_logger(), "Nearest node is: %s", closest_node.c_str());
        // } else {
        //     RCLCPP_WARN(node_->get_logger(), "No close nodes found!");
        // }
}
    
}

void BTVisPanel::enlargeLabel(const std::string& node_name) {
    float newWidth = 1.0;
    float newHeight = 0.5;
    auto text = text_map_.find(node_name);
    if (text != text_map_.end()){
        text->second->setCharacterHeight(0.6);
        text->second->setColor(Ogre::ColourValue(1.0, 1.0, 0.0, 1.0)); 
    }

}

void BTVisPanel::resetLabel(const std::string& node_name) {
    float originalWidth = 0.6;
    float originalHeight = 0.2;
    auto text= text_map_.find(node_name);
    if (text != text_map_.end()){
        text->second->setCharacterHeight(0.4);
        text->second->setColor(Ogre::ColourValue::Black);
    }
}




void BTVisPanel::load(const rviz_common::Config& config) {}
void BTVisPanel::save(rviz_common::Config config) const {}
BTVisPanel::~BTVisPanel() {
    if (node_) {
        rclcpp::shutdown();
    }
    if (spinner_thread_.joinable()) {
        spinner_thread_.join();
    }
}


} // namespace bt_rviz

PLUGINLIB_EXPORT_CLASS(bt_rviz::BTVisPanel, rviz_common::Display)

