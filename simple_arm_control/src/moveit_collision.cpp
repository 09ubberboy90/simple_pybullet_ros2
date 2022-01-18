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

#include "gazebo_msgs/msg/model_state.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "gazebo_msgs/srv/get_model_list.hpp"
#include "service_handler.hpp"
#include "shared.hpp"

#include "simple_interface/srv/set_object_active.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/collision_object.hpp>

#include <chrono>
#include <map>
#include <string>
#include <vector>

void get_model_state_handler(std::shared_ptr<gazebo_msgs::srv::GetEntityState_Response> result, gazebo_msgs::msg::ModelStates *states)
{
    states->name.push_back(result->state.name);
    states->pose.push_back(result->state.pose);
    states->twist.push_back(result->state.twist);
}

void get_model_list_handler(std::shared_ptr<ServiceClient<gazebo_msgs::srv::GetModelList>> model_client, std::shared_ptr<ServiceClient<gazebo_msgs::srv::GetEntityState>> state_client, gazebo_msgs::msg::ModelStates *states)
{
    auto model_request = model_client->create_request_message();
    auto state_request = state_client->create_request_message();
    auto model_response = model_client->service_caller(model_request);
    if (model_response->success)
    {
        for (auto name : model_response->model_names)
        {
            if (banned.count(name) == 0)
            {
                state_request->name = name;
                auto state_response = state_client->service_caller(state_request);
                if (state_response->success)
                {
                    get_model_state_handler(state_response, states);
                }
            }
        }
    }
}

void set_bool(const std::shared_ptr<simple_interface::srv::SetObjectActive::Request> request,
              std::shared_ptr<simple_interface::srv::SetObjectActive::Response> response, bool *param, std::string *obj_name)
{
    *param = request->data;
    *obj_name = request->name;
    response->success = true;
    response->message = "200";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Object %s has been set %s", obj_name->c_str(), request->data ? "active" : "inactive");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::map<std::string, double> obj_height = {};
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto service_node = rclcpp::Node::make_shared("collision_service_handler");
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("set_target_active_server");
    bool param = true;
    std::string obj_name = "";

    std::function<void(const std::shared_ptr<simple_interface::srv::SetObjectActive::Request>,
                       std::shared_ptr<simple_interface::srv::SetObjectActive::Response>)>
        fcn2 = std::bind(set_bool, std::placeholders::_1, std::placeholders::_2, &param, &obj_name);

    rclcpp::Service<simple_interface::srv::SetObjectActive>::SharedPtr service =
        node->create_service<simple_interface::srv::SetObjectActive>("set_target_active", fcn2);

    auto model_client = std::make_shared<ServiceClient<gazebo_msgs::srv::GetModelList>>("get_model_list");
    auto state_client = std::make_shared<ServiceClient<gazebo_msgs::srv::GetEntityState>>("get_entity_state");

    // For current state monitor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    while (true)
    {
        gazebo_msgs::msg::ModelStates states;
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        get_model_list_handler(model_client, state_client, &states);
        for (int i = 0; i < (int)states.name.size(); i++)
        {
            moveit_msgs::msg::CollisionObject obj;
            auto pose = states.pose[i];
            obj.header.frame_id = "world";
            obj.id = states.name[i];
            shape_msgs::msg::SolidPrimitive primitive;
            double height = 0.0;
            if (obj_height.count(obj.id) > 0)
            {
                obj.operation = obj.MOVE;
                height = obj_height[obj.id];
            }
            else // New object
            {
                obj.operation = obj.ADD;
                primitive.type = primitive.BOX;
                primitive.dimensions.resize(3);
                if (obj.id == "table") // table
                {
                    primitive.dimensions[0] = 0.92;
                    primitive.dimensions[1] = 0.5;
                    primitive.dimensions[2] = 0.914;
                    height = primitive.dimensions[1]; // reason is that rviz uses center of mass while webots table uses bottom position
                }
                else // cube
                {
                    primitive.dimensions[0] = 0.05;
                    primitive.dimensions[1] = 0.05;
                    primitive.dimensions[2] = 0.05;
                    height = 0;
                }
                obj.primitives.push_back(primitive);
                obj_height[obj.id] = height;
            }
            pose.position.z += height / 2;

            if (!param && obj.id == obj_name)
            {
                obj_height.erase(obj_name);
                continue;
            }

            obj.primitive_poses.push_back(pose);
            collision_objects.push_back(obj);
        }
        planning_scene_interface.applyCollisionObjects(collision_objects);
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // otherwise too many calls 
    }

    rclcpp::shutdown();
    return 0;
}