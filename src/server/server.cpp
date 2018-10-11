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

    Client c =
        Client::createClient(nh, caller_name, event.getRequest());

    if (!c)
    {
        ROS_ERROR_STREAM("Failed to connect to Adaptable Parameter client: "
                         << event.getCallerName());
        return false;
    } else {
        ROS_INFO_STREAM("Connected from: " << event.getCallerName());
    }

    Client *first_dead = NULL;
    for (Client client : clients)
    {
        // if client is already registered, replace it
        if(client == caller_name)
        {
            c.close();
            c = client;
            return true;
        } else if(first_dead == NULL) {
            if(!client) first_dead = &client;
        }
    }

    if(first_dead)
    {
        *first_dead = c;
    } else {
        clients.push_back(c);
    }

    return true;
}

bool
Server::feedbackCB(ros::ServiceEvent<adap_parameter::Feedback::Request,
                                     adap_parameter::Feedback::Response> &event)
{
    for (Client c : clients)
        if (c == event.getCallerName())
            return c.processFeedback(event.getRequest());

    ROS_ERROR_STREAM(
        "Failed find registered client for feedback feedback from: "
        << event.getCallerName());

    return false;
}
