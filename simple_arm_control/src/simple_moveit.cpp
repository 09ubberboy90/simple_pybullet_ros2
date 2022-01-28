#include "simple_moveit.hpp"

SimpleMoveIt::SimpleMoveIt(std::string node_name) : 
    Node(node_name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    move_group(std::shared_ptr<rclcpp::Node>(std::move(this)), "panda_arm"),
    hand_move_group(std::shared_ptr<rclcpp::Node>(std::move(this)), "hand")
{
    move_group.allowReplanning(true);
    move_group.setNumPlanningAttempts(5);
    hand_move_group.allowReplanning(true);
    hand_move_group.setNumPlanningAttempts(5);
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    hand_move_group.setMaxVelocityScalingFactor(1.0);
    hand_move_group.setMaxAccelerationScalingFactor(1.0);
    this->client = std::make_shared<ServiceClient<simple_interface::srv::SetObjectActive>>("set_target_active");
}

bool SimpleMoveIt::wait_for_exec(moveit::planning_interface::MoveGroupInterface *move_group)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    for (int i = 0; i < 5; i++)
    {
        // 10 tries to plan otherwise give up
        bool success = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(rclcpp::get_logger("simple_moveit"), "Plan %d %s", i, success ? "SUCCEEDED" : "FAILED");
        auto pose = move_group->getPoseTarget().pose.position;
        RCLCPP_INFO(rclcpp::get_logger("simple_moveit"), "Finding a path to %f, %f, %f", pose.x, pose.y, pose.z);

        if (success)
        {
            move_group->execute(plan);
            return true;
        }
    }
    auto pose = move_group->getPoseTarget().pose.position;
    RCLCPP_ERROR(rclcpp::get_logger("simple_moveit"), "Failed to find a valid path to %f, %f, %f", pose.x, pose.y, pose.z);
    return false;
}
bool SimpleMoveIt::pick(std::string name, geometry_msgs::msg::Pose pose, double approach_distance)
{
    bool success = true;
    //open gripper
    RCLCPP_INFO(rclcpp::get_logger("simple_moveit"), "Opening hand");
    success = this->change_gripper(gripper_state::opened);
    // aproach
    RCLCPP_INFO(rclcpp::get_logger("simple_moveit"), "Approaching");
    pose.position.z += approach_distance;
    success = this->goto_pose(pose);
    //remove collision object
    success = this->set_obj_active(name, false);
    auto collision_object = this->planning_scene_interface.getObjects({name})[name];
    collision_object.operation = collision_object.REMOVE;
    this->planning_scene_interface.applyCollisionObject(collision_object);
    //go in position
    pose.position.z -= approach_distance;
    success = this->goto_pose(pose);
    //grip
    RCLCPP_INFO(rclcpp::get_logger("simple_moveit"), "Closing Hand");
    success = this->change_gripper(gripper_state::closed);
    //lift
    RCLCPP_INFO(rclcpp::get_logger("simple_moveit"), "Retracting");
    pose.position.z += approach_distance;
    success = this->goto_pose(pose);

    return success;
}

bool SimpleMoveIt::place(std::string name, geometry_msgs::msg::Pose pose, double approach_distance)
{
    bool success = true;
    // aproach
    RCLCPP_INFO(rclcpp::get_logger("simple_moveit"), "Approaching");
    pose.position.z += approach_distance;
    success = this->goto_pose(pose);
    //go in position
    pose.position.z -= approach_distance;
    success = this->goto_pose(pose);
    //grip
    RCLCPP_INFO(rclcpp::get_logger("simple_moveit"), "Opening Hand");
    success = this->change_gripper(gripper_state::opened);
    //remove collision object
    success = this->set_obj_active(name, true);
    //lift
    RCLCPP_INFO(rclcpp::get_logger("simple_moveit"), "Retracting");
    pose.position.z += approach_distance;
    success = this->goto_pose(pose);

    return success;
}


bool SimpleMoveIt::change_gripper(gripper_state state)
{
    auto current_state = this->hand_move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(current_state->getJointModelGroup("hand"), joint_group_positions);
    float pose = (float)state / 1000;
    joint_group_positions[0] = pose;
    joint_group_positions[1] = pose;
    this->hand_move_group.setGoalPositionTolerance(0.04);
    this->hand_move_group.setJointValueTarget(joint_group_positions);
    return this->wait_for_exec(&this->hand_move_group);
}

bool SimpleMoveIt::goto_pose(geometry_msgs::msg::Pose pose)
{
    // by default assume it's the robot we want to move
    return this->goto_pose(&this->move_group, pose);
}

bool SimpleMoveIt::goto_pose(moveit::planning_interface::MoveGroupInterface *move_group, geometry_msgs::msg::Pose pose)
{
    move_group->setPoseTarget(pose);
    return this->wait_for_exec(move_group);
}

bool SimpleMoveIt::set_obj_active(std::string name, bool set_active)
{
    auto request = this->client->create_request_message();
    request->data = set_active;
    request->name = name;
    auto response = this->client->service_caller(request);
    RCLCPP_INFO(rclcpp::get_logger("simple_moveit"), "Obj %s set %s %s", name.c_str(), set_active ? "active" : "inactive", response->success ? "successfully" : "unsuccessfully");
    return response->success;
}
