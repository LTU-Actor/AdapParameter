#pragma once

#include <adap_parameter/Feedback.h>
#include <adap_parameter/Register.h>
#include <adap_parameter/Tune.h>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace adap_parameter {
class Server {
public:
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

    template <typename... Args> Server(Args... args) : nh("~")
    {
        tune = nh.advertiseService("tune", args...);
        feedback = nh.serviceClient<adap_parameter::Feedback>(
            "/adap_parameter/feedback", true); // persistant
        timer = nh.createTimer(ros::Duration(0), &Server::timerCB, this, true,
                               false);
    }

    bool connect(Server::Tunables n)
    {
        ros::ServiceClient registration =
            nh.serviceClient<adap_parameter::Register>(
                "/adap_parameter/register");

        adap_parameter::Register register_data;
        int num_parameters = n.parameters.size();
        int num_feedbacks = n.feedbacks.size();
        register_data.request.parameter_names.resize(num_parameters);
        register_data.request.feedback_names.resize(num_feedbacks);
        register_data.request.feedback_goals.resize(num_feedbacks);

        for (int i = 0; i < num_parameters; i++)
            register_data.request.parameter_names[i].data =
                n.parameters[i].name;

        for (int i = 0; i < num_feedbacks; i++)
        {
            register_data.request.feedback_names[i].data = n.feedbacks[i].name;
            register_data.request.feedback_goals[i].data = n.feedbacks[i].goal;
        }

        return (initalized = registration.call(register_data));
    }

    bool sendFeedback(adap_parameter::Feedback::Request &req)
    {
        timer_args.request = req;
        timer.stop();
        timer.setPeriod(ros::Duration(0));
        timer.start();
        return true;
    }

private:
    void timerCB(const ros::TimerEvent &e) { feedback.call(timer_args); }

    bool initalized = false;
    ros::NodeHandle nh;
    ros::ServiceServer tune;
    ros::ServiceClient feedback;
    ros::Timer timer;
    adap_parameter::Feedback timer_args;
};
} // namespace adap_parameter
