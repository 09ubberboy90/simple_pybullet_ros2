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



std::pair<const std::string, moveit_msgs::msg::CollisionObject> choose_target(moveit::planning_interface::PlanningSceneInterface *ps, std::set<std::string> * processed)
{   
    std::vector<std::string> keys;


    srand ( time(NULL) ); //initialize the random seed
    std::map<std::string, moveit_msgs::msg::CollisionObject> collision_objects;
    do
    {
        collision_objects = ps->getObjects();
        for (const auto& imap : collision_objects) {
            if (!processed->count(imap.first))
            {
                keys.push_back(imap.first);
            }
        }
    } while (collision_objects.size() <= 0);
        
    int rand_index = rand() % (int) keys.size();
    RCLCPP_INFO(rclcpp::get_logger("panda_moveit_controller"), "Chosen index is %d", rand_index);
    auto name = keys[rand_index];
    auto chosen = * new std::pair<const std::string, moveit_msgs::msg::CollisionObject>(name, collision_objects.at(name));
    processed->emplace(name);
    RCLCPP_INFO(rclcpp::get_logger("panda_moveit_controller"), "Chosen target is %s", chosen.first.c_str());
    return chosen;

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
    int acc = 0;
    int iter = 2;
    std::set<std::string> processed{banned};
    processed.emplace("table");
    auto discard = choose_target(simple_moveit->get_planning_scene_interface(), &processed);

    RCLCPP_INFO(rclcpp::get_logger("panda_moveit_controller"), "Starting timer");
    auto start = std::chrono::steady_clock::now();

    auto start_pose = simple_moveit->get_move_group()->getCurrentPose().pose;
    for (int i = 1; i <= iter; i++)
    {
        auto object = choose_target(simple_moveit->get_planning_scene_interface(), &processed);

        auto obj_name = object.first;
        auto collision_object = object.second;
        bool success = true;
        // set_service(service_node, client, true, obj_name); // advertise to collision
        auto pose = collision_object.primitive_poses[0];
        Eigen::Quaternionf q = Eigen::AngleAxisf(3.14, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(0.785, Eigen::Vector3f::UnitZ());
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.position.z += 0.1;


        success = simple_moveit->pick(obj_name, pose);

        if (!success)
        {
            //Handle failure
        }
        

        pose.position.x = 0.5;
        pose.position.y = 0.0;
        pose.position.z = (0.4) + i*0.05; 
        

        success = simple_moveit->place(obj_name, pose, 0.15);

        
        if (!success)
        {
            //Handle failure
        }
        collision_object = simple_moveit->get_planning_scene_interface()->getObjects({obj_name})[obj_name];
        auto new_pose = collision_object.primitive_poses[0];

        if ((new_pose.position.x < pose.position.x - 0.05) || (pose.position.x + 0.05 < new_pose.position.x) ||
            (new_pose.position.y < pose.position.y - 0.05) || (pose.position.y + 0.05 < new_pose.position.y)) 
            // No need to check height since if its in position then it can only be on top of the other cube
        {
            RCLCPP_ERROR(rclcpp::get_logger("panda_moveit_controller"), "Cube is not in bound");
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("panda_moveit_controller"), "Cube in bound");
            acc += 1;
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("panda_moveit_controller"), "Going to start pose");
    simple_moveit->goto_pose(start_pose);
    auto end = std::chrono::steady_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
    if (acc == iter)
    {
        RCLCPP_INFO(rclcpp::get_logger("panda_moveit_controller"), "Task executed successfully in %s ms", std::to_string(diff.count()).c_str());
    
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("panda_moveit_controller"), "Task failed with %d cube stacked in %s ms", acc, std::to_string(diff.count()).c_str());
    }
    

}