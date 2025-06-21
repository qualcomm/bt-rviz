// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries
// SPDX-License-Identifier: BSD-3-Clause-Clear
#ifndef BT_VIS_PANEL_HPP
#define BT_VIS_PANEL_HPP

#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/display.hpp>
#include <rclcpp/rclcpp.hpp>
#include <bt_msgs/msg/bt.hpp>
#include <bt_msgs/msg/bt_node.hpp>
#include <QLabel>
#include <QVBoxLayout>
#include <QPoint>
#include <QWindow>
#include <QVector3D>
#include <QEvent>
#include <rviz_rendering/objects/movable_text.hpp>
#include <OgreSceneManager.h>  
#include <OgreSceneNode.h> 
#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreManualObject.h> 
#include <OgreAxisAlignedBox.h> 
#include <rviz_common/message_filter_display.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/objects/line.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/render_window.hpp>
#include <rviz_common/render_panel.hpp>
#include <QMouseEvent>
#include <OgreRenderWindow.h>
#include <QCoreApplication>

namespace bt_rviz {

class BTVisPanel : public rviz_common::MessageFilterDisplay<bt_msgs::msg::BT>{
    Q_OBJECT

public:
    explicit BTVisPanel(QWidget* parent = nullptr);
    void load(const rviz_common::Config& config) override;
    void save(rviz_common::Config config) const override;
    ~BTVisPanel();
    
protected:
    void processMessage(const bt_msgs::msg::BT::ConstSharedPtr msg) override;
    void onInitialize() override;
    void update(float wall_dt, float ros_dt) override;
    void drawNode(float x, float y, const std::string& name, const Ogre::ColourValue& color, const std::string& type);
    void redrawNode(const std::string& node_name);
    bt_msgs::msg::BTNode findNodeByName(const std::string& name, const bt_msgs::msg::BT::SharedPtr latest_msg_);
    float recursivelyPlaceNodes(const std::string& parent_name, int level,
        std::unordered_set<std::string>& processed_nodes);
    Ogre::ColourValue getColorByStatus(const std::string& name);
    void processUpdateMessage(const bt_msgs::msg::BTNode::SharedPtr msg); 
    std::string findParallelParent(const std::string& node_name);
    void checkHover(const QPoint& mousePosition); 
    void enlargeLabel(const std::string& node_name);
    void resetLabel(const std::string& node_name);
    bool eventFilter(QObject* object, QEvent* event) override;
    Ogre::AxisAlignedBox get_active_bbox();

private:
    rclcpp::Node::SharedPtr node_;
    std::thread spinner_thread_;
    QLabel* label_;
    rviz_common::properties::RosTopicProperty* topic_property_;
    Ogre::SceneManager* scene_manager_;
    Ogre::SceneNode* scene_node_;
    Ogre::ManualObject* manual_object_;
    rviz_rendering::MovableText* node_label_;
    rclcpp::Subscription<bt_msgs::msg::BTNode>::SharedPtr bt_subscriber_;
    bt_msgs::msg::BT::SharedPtr latest_msg_;
    std::unordered_map<std::string, std::string> nodeShapeMap;
    float next_x_ = 0.0;
    std::vector<rviz_rendering::Line*> line_map_;
    std::vector<std::pair<std::string, std::string>> edges_to_draw_;
    std::string last_updated_node_;
    std::string last_updated_parent_;
    std::unordered_set<std::string> non_idle_nodes_;
    bt_msgs::msg::BT::SharedPtr cached_tree_;
    std::unordered_map<std::string, Ogre::Vector3> node_positions;
    std::unordered_map<std::string, rviz_rendering::Shape*> node_map_;
    Ogre::Camera* camera_;
    std::unordered_map<std::string, rviz_rendering::MovableText*> text_map_;
    rviz_common::RenderPanel* render_panel_;
    rviz_common::ViewController* view_controller_;
};

} // namespace bt_rviz_plugin

#endif // BT_VIS_PANEL_HPP
