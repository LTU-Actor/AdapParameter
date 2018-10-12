#pragma once

#include <vector>

#include <ros/ros.h>

#include <adap_parameter/Feedback.h>
#include <adap_parameter/Register.h>
#include <memory>

#include "client.h"

class Server {
public:
    Server(const ros::NodeHandle &);
    void run();

private:
    bool registrationCB(ros::ServiceEvent<adap_parameter::Register::Request,
                                     adap_parameter::Register::Response> &);
    bool feedbackCB(ros::ServiceEvent<adap_parameter::Feedback::Request,
                                      adap_parameter::Feedback::Response> &);

    ros::NodeHandle    nh;
    ros::ServiceServer registration_server;
    ros::ServiceServer feedback_server;

    std::vector<std::shared_ptr<Client>> clients;
};
