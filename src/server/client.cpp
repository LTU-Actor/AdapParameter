#include "client.h"

#include <adap_parameter/Tune.h>

Client::Client(ros::NodeHandle &nh, const std::string &name,
               const adap_parameter::Register::Request &req)
{
    node_name = name;

    std::string tune_path = name + "/tune";
    tune = nh.serviceClient<adap_parameter::Tune>(tune_path, true);

    parameter_names.reserve(req.parameter_names.size());
    for (auto s : req.parameter_names)
    {
        parameter_names.push_back(s.data);
        ROS_INFO_STREAM("  ParamName: " << s.data);
    }

    feedback_names.reserve(req.feedback_names.size());
    for (auto s : req.feedback_names)
    {
        feedback_names.push_back(s.data);
        ROS_INFO_STREAM("  FeedbackName: " << s.data);
    }

    feedback_goals.reserve(req.feedback_goals.size());
    for (auto f : req.feedback_goals)
    {
        feedback_goals.push_back(f.data);
        ROS_INFO_STREAM("  FeedbackGoal: " << f.data);
    }

    state = tune.waitForExistence(ros::Duration(.1)) ? WAITING : DEAD;

    if (*this)
        updater = nh.createTimer(ros::Duration(2), &Client::updateCB, this);
}

void
Client::updateCB(const ros::TimerEvent &e)
{
    if (*this)
    {
        ROS_INFO_STREAM(node_name << ": Update ("
                                  << (state == WAITING ? "waiting" : "sending")
                                  << ")");
    }
    else
    {
        updater.stop();
        ROS_INFO_STREAM(node_name << ": Removing dead node");
    }
}

void
Client::close()
{
    updater.stop();
    tune.shutdown();
    state = DEAD;
}

bool
Client::processFeedback(const adap_parameter::Feedback::Request &req)
{
    return true;
}
