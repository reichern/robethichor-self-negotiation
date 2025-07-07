/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Metro Robots
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Metro Robots nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: David V. Lu!! */

#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <rviz_goal_panel/goal_panel.hpp>

namespace rviz_goal_panel
{
GoalPanel::GoalPanel(QWidget * parent) : Panel(parent)
{
  // Create labels for different text, displayed vertically (the V in VBox means vertical)
  const auto layout = new QVBoxLayout(this);
  goal_label_ = new QLabel("No current Goal.");
  goal_label_->setStyleSheet("font: 14pt;");  
  interrupting_goal_label_ = new QLabel("");
  interrupting_goal_label_->setStyleSheet("font: 14pt;");  
  winner_label_ = new QLabel("");
  winner_label_->setStyleSheet("font: 14pt;");  
  layout->addWidget(goal_label_);
  layout->addWidget(interrupting_goal_label_);
  layout->addWidget(winner_label_);
}

GoalPanel::~GoalPanel() = default;

void GoalPanel::onInitialize()
{
  // Access the abstract ROS Node and
  // in the process lock it for exclusive use until the method is done.
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

  // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
  // (as per normal rclcpp code)
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  goal_subscription_ = node->create_subscription<std_msgs::msg::String>(
      "/goal", 10, std::bind(&GoalPanel::goalCallback, this, std::placeholders::_1));
  interrupting_goal_subscription_ = node->create_subscription<std_msgs::msg::String>(
      "/interrupting_user/goal", 10, std::bind(&GoalPanel::interruptingGoalCallback, this, std::placeholders::_1));
  winner_subscription_ = node->create_subscription<std_msgs::msg::String>(
      "/negotiation_result", 10, std::bind(&GoalPanel::winnerCallback, this, std::placeholders::_1));

}

// When the subscriber gets a message, this callback is triggered,
// and then we copy its data into the widget's label
void GoalPanel::goalCallback(const std_msgs::msg::String& msg)
{
  if (msg.data == "Reset robot..."){
    goal_label_->setText(QString(msg.data.c_str()));
    interrupting_goal_label_->setText(" ");
    winner_label_->setText(" ");
  } else {
    goal_label_->setText("Current Goal: " + QString(msg.data.c_str()));
  }
}

// When the subscriber gets a message, this callback is triggered,
// and then we copy its data into the widget's label
void GoalPanel::interruptingGoalCallback(const std_msgs::msg::String& msg)
{
  interrupting_goal_label_->setText("Interruption!\nInterrupting Goal: " + QString(msg.data.c_str()));
}

// When the subscriber gets a message, this callback is triggered,
// and then we copy its data into the widget's label
void GoalPanel::winnerCallback(const std_msgs::msg::String& msg)
{
  winner_label_->setText("Negotiation Result: " + QString(msg.data.c_str()));
}

}  // namespace rviz_goal_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_goal_panel::GoalPanel, rviz_common::Panel)
