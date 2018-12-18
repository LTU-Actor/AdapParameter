#pragma once

#include "client.h"

#include <adap_parameter/Feedback.h>
#include <adap_parameter/Register.h>
#include <memory>
#include <ros/ros.h>
#include <string>
#include <unordered_map>

/*
 * Main class for server. It has a few jobs:
 *  - Handle registrations
 *  - Keep a map of client names to Client objects
 *  - Forward feedback to the correct Client object
 *
 *  We take a node handle in the constructor so make it easier to change the
 * node's namespace (in main.cpp)
 */
class Server {
public:
    Server(const ros::NodeHandle &);

    /*
     * Does not exit until ros::spin() would exit
     */
    void run();

private:
    bool
    registrationCB(ros::ServiceEvent<adap_parameter::Register::Request,
                                     adap_parameter::Register::Response> &);
    bool feedbackCB(ros::ServiceEvent<adap_parameter::Feedback::Request,
                                      adap_parameter::Feedback::Response> &);

    /*
     * When called, checks every Client in the map and deletes it if the
     * corisponding node has died.
     */
    void pruneDeadClients();

    ros::NodeHandle nh;
    ros::ServiceServer registration_server;
    ros::ServiceServer feedback_server;

    std::unordered_map<std::string, std::shared_ptr<Client>> clients;
};
