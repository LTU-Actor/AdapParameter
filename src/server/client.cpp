#include "client.h"

#include <adap_parameter/Tune.h>

Client
Client::createClient(ros::NodeHandle &nh, const std::string &name,
                     const adap_parameter::Register::Request &req)
{
    Client c;

    c.node_name = name;

    std::string tune_path = name + "/tune";
    c.tune = nh.serviceClient<adap_parameter::Tune>(tune_path, true);

    c.parameter_names.reserve(req.parameter_names.size());
    for (auto s : req.parameter_names) c.parameter_names.push_back(s.data);

    c.feedback_names.reserve(req.feedback_names.size());
    for (auto s : req.parameter_names) c.feedback_names.push_back(s.data);

    c.feedback_goals.reserve(req.feedback_goals.size());
    for (auto f : req.feedback_goals) c.feedback_goals.push_back(f.data);

    c.state = c.tune.waitForExistence(ros::Duration(.1)) ? WAITING : DEAD;
    return c;
}

void
Client::close()
{
    tune.shutdown();
    state = DEAD;
}

bool
Client::processFeedback(const adap_parameter::Feedback::Request &req)
{
    return true;
}
