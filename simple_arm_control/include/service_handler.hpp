#pragma once

#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <stdlib.h>

// using CallbackType = std::function<
// void (
//     const std::shared_ptr<typename ServiceT::Request>,
//     std::shared_ptr<typename ServiceT::Response>)>;

template <class T>
class ServiceClient
{
public:
    ServiceClient(std::string srv_name);
    std::shared_ptr<typename T::Response> service_caller(std::shared_ptr<typename T::Request> request);
    std::shared_ptr<typename T::Request> create_request_message();

private:
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<rclcpp::Client<T>> client;
};

// template <> class ServiceClient <>{};

template <class T>
ServiceClient<T>::ServiceClient(std::string srv_name)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Created service with name %s", srv_name.c_str());
    node = rclcpp::Node::make_shared(srv_name);
    client = node->create_client<T>(srv_name);
}

template <class T>
std::shared_ptr<typename T::Response> ServiceClient<T>::service_caller(std::shared_ptr<typename T::Request> request)
{
    std::chrono::seconds timeout(1);
    while (!client->wait_for_service(timeout))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return nullptr;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        return result.get(); // get the future result
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
        return nullptr;
    }
}

template <class T>
std::shared_ptr<typename T::Request> ServiceClient<T>::create_request_message()
{
    return std::make_shared<typename T::Request>();
}
