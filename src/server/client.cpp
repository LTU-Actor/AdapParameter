#include "client.h"

#include <cmath>

#include <adap_parameter/Tune.h>

#include "tuner.h"
#include "tuner_gradients.h"

Client::Client(ros::NodeHandle &nh, const std::string &name,
               const adap_parameter::Register::Request &req)
    :node_name(name),
     tuner(std::make_shared<TunerGradients>())
{
    std::string tune_path = name + "/tune";
    tune = nh.serviceClient<adap_parameter::Tune>(tune_path, true);

    parameter_names.reserve(req.parameter_names.size());
    for (auto s : req.parameter_names)
        parameter_names.push_back(s.data);

    feedback_names.reserve(req.feedback_names.size());
    for (auto s : req.feedback_names)
        feedback_names.push_back(s.data);

    feedback_goals.reserve(req.feedback_goals.size());
    for (auto f : req.feedback_goals)
        feedback_goals.push_back(f.data);

    tuner->init(parameter_names.size(), feedback_goals);
    timer = nh.createTimer(ros::Duration(0), &Client::timerCB, this, true, false);
    sendTune(tuner->get_inital());
}

void
Client::timerCB(const ros::TimerEvent &e)
{
    adap_parameter::Tune args;
    args.request.parameters.resize(params_to_send.size());
    for(int i=0; i<params_to_send.size(); i++)
    {
        args.request.parameters[i].data = params_to_send[i];
    }

    tune.call(args);
}

void
Client::sendTune(const Tuner::parameters &params)
{
    params_to_send = params;
    timer.stop();
    timer.setPeriod(ros::Duration(0));
    timer.start();
}

void
Client::close()
{
    tune.shutdown();
}

bool
Client::processFeedback(const adap_parameter::Feedback::Request &req)
{
    if(*this)
    {
        std::vector<double> feedback;
        feedback.resize(req.feedback.size());
        for(int i=0; i<feedback.size(); i++)
            feedback[i] = req.feedback[i].data;
        sendTune(tuner->iterate(feedback));
        return true;
    }

    return false;
}
