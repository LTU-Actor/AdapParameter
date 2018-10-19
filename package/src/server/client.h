#pragma once

#include <adap_parameter/Feedback.h>
#include <adap_parameter/Register.h>
#include <memory>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "tuner.h"

class Client {
public:
    Client(ros::NodeHandle &, const std::string &name,
           const adap_parameter::Register::Request &);

    ~Client() { close(); }
    void close();

    bool processFeedback(const adap_parameter::FeedbackRequest &);

    explicit operator bool() { return tune.exists(); }

    bool operator==(const std::string &n) { return node_name == n; }

private:
    Client(const Client &) = delete; // breaks callbacks on copy?

    void sendTune(const Tuner::parameters &params);

    std::shared_ptr<Tuner> tuner = NULL;

    void timerCB(const ros::TimerEvent &);
    ros::Timer timer;
    Tuner::parameters params_to_send;

    std::string node_name;
    ros::ServiceClient tune;
    std::vector<std::string> parameter_names;
    std::vector<std::string> feedback_names;
    Tuner::feedback feedback_goals;
};
