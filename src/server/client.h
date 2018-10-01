#pragma once

#include <adap_parameter/Feedback.h>
#include <adap_parameter/Register.h>
#include <ros/ros.h>
#include <string>
#include <vector>

class Client {
public:
    static Client createClient(ros::NodeHandle &, const std::string &name,
                               const adap_parameter::Register::Request &);

    void close();

    bool processFeedback(const adap_parameter::FeedbackRequest &);

    explicit operator bool()
    {
        if (!tune.isValid()) state = DEAD;
        return state != DEAD;
    }
    bool operator==(const std::string &n) { return node_name == n; }

private:
    enum State {
        DEAD,
        WAITING, // waiting on feedback
        SLEEPING // sleeping to send next tune
    } state;

    Client() {}

    std::string              node_name;
    ros::ServiceClient       tune;
    std::vector<std::string> parameter_names;
    std::vector<std::string> feedback_names;
    std::vector<float>       feedback_goals;
};
