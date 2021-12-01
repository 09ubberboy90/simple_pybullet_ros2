//BSD 3-Clause License
//
//Copyright (c) 2021, Florent Audonnet
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions are met:
//
//1. Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
//2. Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
//3. Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#pragma once
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <string>

#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "service_handler.hpp"
#include "simple_interface/srv/set_object_active.hpp"

enum gripper_state
{
    opened = 35,
    closed = 0
};

class SimpleMoveIt : public rclcpp::Node
{
public:
    SimpleMoveIt(std::string node_name);
    bool pick(std::string name, geometry_msgs::msg::Pose pose, double approach_distance = 0.1);
    bool place(std::string name, geometry_msgs::msg::Pose pose, double approach_distance = 0.1);
    bool goto_pose(geometry_msgs::msg::Pose pose);
    bool goto_pose(moveit::planning_interface::MoveGroupInterface *move_group, geometry_msgs::msg::Pose pose);
    bool change_gripper(gripper_state state);
    moveit::planning_interface::MoveGroupInterface *get_move_group() { return &move_group; }
    moveit::planning_interface::MoveGroupInterface *get_hand_move_group() { return &hand_move_group; }
    moveit::planning_interface::PlanningSceneInterface *get_planning_scene_interface() { return &planning_scene_interface; }
    bool set_obj_active(std::string name, bool set_active);

private:
    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::MoveGroupInterface hand_move_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::shared_ptr<ServiceClient<simple_interface::srv::SetObjectActive>> client;

    bool wait_for_exec(moveit::planning_interface::MoveGroupInterface *move_group);
};
