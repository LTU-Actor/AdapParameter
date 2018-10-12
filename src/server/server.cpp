#include "server.h"

#include <adap_parameter/Feedback.h>
#include <adap_parameter/Register.h>
#include <adap_parameter/Tune.h>

#include <string>

Server::Server(const ros::NodeHandle &nh) : nh(nh) {}

void
Server::run()
{
    registration_server =
        nh.advertiseService("register", &Server::registrationCB, this);
    feedback_server =
        nh.advertiseService("feedback", &Server::feedbackCB, this);
    ros::spin();
    registration_server.shutdown();
}

bool
Server::registrationCB(
    ros::ServiceEvent<adap_parameter::Register::Request,
                      adap_parameter::Register::Response> &event)
{
    std::string caller_name = event.getCallerName();

    ROS_INFO_STREAM("Getting connection from: " << event.getCallerName());

    std::shared_ptr<Client> c =
        std::make_shared<Client>(nh, caller_name, event.getRequest());

    if (!c)
    {
        ROS_ERROR_STREAM("Failed to connect to Adaptable Parameter client: "
                         << event.getCallerName());
        return false;
    }

    for (auto &client : clients)
    {
        // if client is already registered, replace it
        if (*client == caller_name)
        {
            ROS_INFO_STREAM("  Re-registering client");
            client = c;
            return true;
        }
        else if (!(*client))
        {
            client = c;
        }
    }

    clients.push_back(c);

    return true;
}

bool
Server::feedbackCB(ros::ServiceEvent<adap_parameter::Feedback::Request,
                                     adap_parameter::Feedback::Response> &event)
{
    for (auto &c : clients)
        if (*c == event.getCallerName())
            return c->processFeedback(event.getRequest());

    ROS_ERROR_STREAM(
        "Failed find registered client for feedback feedback from: "
        << event.getCallerName());

    return false;
}
