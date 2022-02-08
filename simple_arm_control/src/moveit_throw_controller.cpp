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

#include <moveit/planning_scene/planning_scene.h>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <thread>
#include <chrono>

#include "service_handler.hpp"
#include "shared.hpp"
#include "simple_moveit.hpp"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "simple_interface/srv/set_object_active.hpp"

bool check_object_pose(geometry_msgs::msg::Pose * current_pose, geometry_msgs::msg::Pose * target_pose)
{

    if ((current_pose->position.x < target_pose->position.x - 0.05) || (target_pose->position.x + 0.05 < current_pose->position.x) ||
        (current_pose->position.y < target_pose->position.y - 0.05) || (target_pose->position.y + 0.05 < current_pose->position.y)) 
        // No need to check height since if its in position then it can only be on top of the other cube
    {
        RCLCPP_ERROR(rclcpp::get_logger("panda_moveit_controller"), "Cube is not in bound");
        return false;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("panda_moveit_controller"), "Cube in bound");
        return true;
    }

}

std::vector<std::string> extract_keys(std::map<std::string, geometry_msgs::msg::Pose> const& input_map) {
  std::vector<std::string> retval;
  for (auto const& element : input_map) {
    retval.push_back(element.first);
  }
  return retval;
}


int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    
    auto simple_moveit = std::make_shared<SimpleMoveIt>("panda_moveit_controller");
    // For current state monitor

    std::thread([&simple_moveit]() { 
        rclcpp::spin(simple_moveit);
        rclcpp::shutdown();}
    ).detach();
    int moved = 0;
    int failure = 0;
    
    std::set<std::string> processed{banned};
    processed.emplace("table");
    processed.emplace("target");

    std::map<std::string, moveit_msgs::msg::CollisionObject> poses;
    do
    {
        poses = simple_moveit->get_planning_scene_interface()->getObjects();
    } while (poses.size() <= 0);

    RCLCPP_INFO(rclcpp::get_logger("panda_moveit_controller"), "Starting timer");
    auto start = std::chrono::steady_clock::now();

    auto start_pose = simple_moveit->get_move_group()->getCurrentPose().pose;
    auto obj_name = "target";
    auto collision_object = simple_moveit->get_planning_scene_interface()->getObjects({obj_name})[obj_name];

    auto pose = collision_object.primitive_poses[0];
    bool success = true;
    // set_service(service_node, client, true, obj_name); // advertise to collision
    Eigen::Quaternionf q = Eigen::AngleAxisf(3.14, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(0.785, Eigen::Vector3f::UnitZ());
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.position.z += 0.1;


    success = simple_moveit->pick(obj_name, pose);

    if (!success)
    {
        failure += 1;//Handle failure
    }
    
    success = simple_moveit->throw_obj(obj_name);

    
    if (!success)
    {
        failure += 1; //Handle failure
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // wait for it to settle

    RCLCPP_INFO(rclcpp::get_logger("panda_moveit_controller"), "Going to start pose");
    simple_moveit->goto_pose(start_pose);
    auto end = std::chrono::steady_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
    RCLCPP_INFO(rclcpp::get_logger("panda_moveit_controller"), "Task finished executing in %s ms", std::to_string(diff.count()).c_str());
    std::this_thread::sleep_for(std::chrono::seconds(60));
    auto collision_objects = simple_moveit->get_planning_scene_interface()->getObjects();

    for (const auto& imap : collision_objects)
    {
        if (!check_object_pose(&collision_objects[imap.first].primitive_poses[0], &poses[imap.first].primitive_poses[0]))
        {
            moved += 1;
        }
    }
    auto new_pose = simple_moveit->get_planning_scene_interface()->getObjects({obj_name})[obj_name].primitive_poses[0];

    RCLCPP_INFO(rclcpp::get_logger("panda_moveit_controller"), 
    "%d cubes moved out of 6. Original cube %s. %d moveit failure",moved, check_object_pose(&new_pose, &pose) ? "still in place" : "moved", failure);
}