#include "server.h"

#include <adap_parameter/Feedback.h>
#include <adap_parameter/Register.h>
#include <adap_parameter/Tune.h>
#include <string>

Server::Server(const ros::NodeHandle &nh) : nh(nh) {}

void
Server::run()
{
    // Create the only two services under the node handle's namespace
    registration_server =
        nh.advertiseService("register", &Server::registrationCB, this);
    feedback_server =
        nh.advertiseService("feedback", &Server::feedbackCB, this);

    // The rest of everything happens in callbacks
    ros::spin();

    registration_server.shutdown();
}

bool
Server::registrationCB(
    ros::ServiceEvent<adap_parameter::Register::Request,
                      adap_parameter::Register::Response> &event)
{
    std::string caller_name = event.getCallerName();

    ROS_INFO_STREAM("Adap Parameter: Getting connection from: " << caller_name);

    std::shared_ptr<Client> c =
        std::make_shared<Client>(nh, caller_name, event.getRequest());

    if (!c)
    {
        ROS_ERROR_STREAM(
            "Adap Parameter: Failed to connect to Adaptable Parameter client: "
            << caller_name);

        // Exit fail
        return false;
    }

    // This is a good place for this, since it will prevent the map from growing
    // unnecessarily.
    pruneDeadClients();

    // Add the client to the map, kicking the existing one out if it exists.
    clients[caller_name] = c;

    // Exit success
    return true;
}

bool
Server::feedbackCB(ros::ServiceEvent<adap_parameter::Feedback::Request,
                                     adap_parameter::Feedback::Response> &event)
{
    auto c = clients[event.getCallerName()];

    //  Check if node has not registered (bad feedback call)
    if (!c)
    {
        ROS_ERROR_STREAM(
            "Adap Parameter: Failed find registered client for feedback feedback from: "
            << event.getCallerName());

        return false;
    }
    else
    {
        c->processFeedback(event.getRequest());
        return true;
    }
}

void
Server::pruneDeadClients()
{
    // Funky for loop because of iterator invalidation
    for (auto it = clients.begin(); it != clients.end();)
    {
        if (!(*it->second))
        {
            ROS_INFO_STREAM("Adap Parameter: " << it->first << " is dead. Unregistering...");
            clients.erase(it++);
        }
        else
        {
            ++it;
        }
    }
}
