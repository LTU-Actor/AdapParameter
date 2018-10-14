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

    clients[event.getCallerName()] = c;
    pruneDeadClients();

    return true;
}

bool
Server::feedbackCB(ros::ServiceEvent<adap_parameter::Feedback::Request,
                                     adap_parameter::Feedback::Response> &event)
{
    return clients[event.getCallerName()]->processFeedback(event.getRequest());

    ROS_ERROR_STREAM(
        "Failed find registered client for feedback feedback from: "
        << event.getCallerName());

    return false;
}

void
Server::pruneDeadClients()
{
    for(auto it = clients.begin(); it != clients.end(); )
    {
        if(!(*it->second))
        {
            ROS_INFO_STREAM("Removing dead client: " << it->first);
            clients.erase(it++);
        } else
        {
            ROS_INFO_STREAM(it->first << " is alive");
            ++it;
        }
    }
}
