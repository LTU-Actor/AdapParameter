#pragma once

#include <adap_parameter/Feedback.h>
#include <adap_parameter/Tune.h>
#include <ros/ros.h>

namespace adap_parameter {
class Server {
public:
    typedef int (*CallbackType)();

    struct Parameter {
        std::string name;
    };

    struct Feedback {
        std::string name;
        float goal;
    };

    struct Tunables {
        std::vector<Parameter> parameters;
        std::vector<Feedback> feedbacks;
    };

    Server(bool (*tuneCB)(adap_parameter::Tune::Request &,
                          adap_parameter::Tune::Response &));

    bool connect(Tunables n);
    void setCallback(CallbackType cb) { this->cb = cb; }
    bool sendFeedback(adap_parameter::Feedback::Request &req);

private:
    void timerCB(const ros::TimerEvent &e);

    CallbackType cb = NULL;
    bool initalized = false;
    ros::NodeHandle nh;
    ros::ServiceServer tune;
    ros::ServiceClient feedback;
    ros::Timer timer;
    adap_parameter::Feedback timer_args;
};
} // namespace adap_parameter
