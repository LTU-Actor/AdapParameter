#include "client.h"

#include <adap_parameter/Tune.h>
#include <cmath>

#include "tuner.h"
#include "tuner_gradients.h"

Client::Client(ros::NodeHandle &nh, const std::string &name,
               const adap_parameter::Register::Request &req)
  : node_name(name),
    tuner(std::make_shared<TunerGradients>()) // For now, always use gradients
{
    // Client nodes are required to have the tune callback directly under their
    // namespace
    std::string tune_path = name + "/tune";
    tune = nh.serviceClient<adap_parameter::Tune>(tune_path, true);

    // Wrangle the names and goals out of their .data vector into something more
    // sensible...
    parameter_names.reserve(req.parameter_names.size());
    for (auto s : req.parameter_names) parameter_names.push_back(s.data);

    feedback_names.reserve(req.feedback_names.size());
    for (auto s : req.feedback_names) feedback_names.push_back(s.data);

    feedback_goals.reserve(req.feedback_goals.size());
    for (auto f : req.feedback_goals) feedback_goals.push_back(f.data);

    // Initialize the chosen tuning algorithm
    tuner->init(parameter_names.size(), feedback_goals);

    // Create the timer used to queue up tunes
    timer =
        nh.createTimer(ros::Duration(0), &Client::timerCB, this, true, false);

    // Send what the tuner wants to start at to the client
    sendTune(tuner->get_inital());
}

void
Client::timerCB(const ros::TimerEvent &e)
{
    adap_parameter::Tune args; // service message to send

    // Put our nicely set up vector into the message format
    args.request.parameters.resize(params_to_send.size());
    for (int i = 0; i < params_to_send.size(); i++)
        args.request.parameters[i].data = params_to_send[i];

    // Send it to the client
    tune.call(args);
}

void
Client::sendTune(const Tuner::parameters &params)
{
    params_to_send = params;

    // Reset the timer so it is put next in the ROS queue
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
    if (*this)
    {
        // More wrangling vectors into our nice vectors
        std::vector<double> feedback;
        feedback.resize(req.feedback.size());
        for (int i = 0; i < feedback.size(); i++)
            feedback[i] = req.feedback[i].data;

        // Run the chosen tuning algorithm and queue sending the feedback. We
        // can't directly call the service here. ros::spin() will not continue
        // processing events until we return from this function, but a call to a
        // service would not return until it is processed.
        //
        // TLDR: calling service from within service callback means deadlock
        sendTune(tuner->iterate(feedback));
        return true;
    }

    return false;
}
