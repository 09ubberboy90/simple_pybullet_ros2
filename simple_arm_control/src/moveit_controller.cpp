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

#include "service_handler.hpp"
#include "shared.hpp"
#include "simple_moveit.hpp"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "simple_interface/srv/set_object_active.hpp"



std::pair<const std::string, moveit_msgs::msg::CollisionObject> choose_target(moveit::planning_interface::PlanningSceneInterface *ps, std::set<std::string> * processed)
{    
    srand ( time(NULL) ); //initialize the random seed
    auto collision_objects = ps->getObjects();

    for (std::set<std::string>::iterator it = processed->begin(); it != processed->end(); it++) 
    {
        // Known as the erase remove idiom
        collision_objects.erase(*it);
    }

    int rand_index = rand() % (int) collision_objects.size();
    auto chosen = *std::next(std::begin(collision_objects),rand_index-1);
    
    processed->emplace(chosen.first);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Chosen target is %s", chosen.first.c_str());
    return chosen;

}



int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto simple_moveit = std::make_shared<SimpleMoveIt>("panda_group_interface");
    // For current state monitor
    rclcpp::executors::SingleThreadedExecutor executor;
    // executor.add_node(service_node);
    executor.add_node(simple_moveit);
    // executor.add_node(parameter_server);
    std::thread([&executor]() { executor.spin(); }).detach();

    auto start_pose = simple_moveit->get_move_group()->getCurrentPose().pose;
    std::set<std::string> processed{banned};
    for (int i = 1; i <= 5; i++)
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
        

        pose.position.x = 0.6;
        pose.position.y = 0.0;
        pose.position.z = (0.4) + i*0.05; 
        

        success = simple_moveit->place(obj_name, pose);

        
        if (!success)
        {
            //Handle failure
        }
        collision_object = simple_moveit->get_planning_scene_interface()->getObjects({obj_name})[obj_name];
        auto new_pose = collision_object.primitive_poses[0];

        if ((new_pose.position.x < pose.position.x - 0.05) || (pose.position.x + 0.05 < new_pose.position.x))
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cube is not in bound");
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Task completed Succesfully");
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going to start pose");
    simple_moveit->goto_pose(start_pose);
    rclcpp::shutdown();
    return 0;
}