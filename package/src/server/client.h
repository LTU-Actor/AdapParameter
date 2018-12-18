#pragma once

#include <adap_parameter/Feedback.h>
#include <adap_parameter/Register.h>
#include <memory>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "tuner.h"

/*
 * Client class keeps track of all data related to a particular client
 * connection
 *
 * Accepts feedback from server, and handles running the tuning algorithm
 * selected, then calls the client's tune service.
 */
class Client {
public:
    Client(ros::NodeHandle &, const std::string &name,
           const adap_parameter::Register::Request &);

    ~Client() { close(); }

    void close();

    // Accept feedback from server
    bool processFeedback(const adap_parameter::FeedbackRequest &);

    // bool operator returns is alive
    explicit operator bool() { return tune.exists(); }

    // compare true of clients' node names are same
    bool operator==(const std::string &n) { return node_name == n; }

private:
    Client(const Client &) = delete; // breaks callbacks on copy?

    void sendTune(const Tuner::parameters &params);

    // The client's tune service cannot be called recursivly while processing
    // the client's feedback, since it would not ever give ros::spin execution
    // control. Instead, schedule calling the service with a 0 length timer.
    void timerCB(const ros::TimerEvent &);
    ros::Timer timer;

    std::string node_name;
    Tuner::parameters params_to_send;
    Tuner::feedback feedback_goals;

    ros::ServiceClient tune;

    // Type of tuner. Ex: GradientDecent
    std::shared_ptr<Tuner> tuner = NULL;

    // Nothing uses the name yet. They are intended for a graphical front-end
    // that allows for manual tuning similar to dynamic_reconfigure.
    std::vector<std::string> parameter_names;
    std::vector<std::string> feedback_names;
};
