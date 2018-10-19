#include <adap_parameter/Feedback.h>
#include <adap_parameter/Register.h>
#include <adap_parameter/Tune.h>
#include <ros/ros.h>
#include <string>
#include <vector>

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

    Server() : nh("~")
    {
        tune = nh.advertiseService("tune", &Server::tuneCB, this);
        feedback = nh.serviceClient<adap_parameter::Feedback>(
            "/adap_parameter/feedback", true); // persistant
        timer = nh.createTimer(ros::Duration(0), &Server::timerCB, this, true, false);
    }

    bool connect(Tunables n)
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
        {
            register_data.request.parameter_names[i].data =
                n.parameters[i].name;
        }

        for (int i = 0; i < num_feedbacks; i++)
        {
            register_data.request.feedback_names[i].data = n.feedbacks[i].name;
            register_data.request.feedback_goals[i].data = n.feedbacks[i].goal;
        }

        return (initalized = registration.call(register_data));
    }

    void setCallback(CallbackType cb) { this->cb = cb; }

private:
    bool tuneCB(adap_parameter::Tune::Request &req,
                adap_parameter::Tune::Response &res)
    {
        std::cout << req.parameters[0].data << ", " << req.parameters[1].data << ", " << req.parameters[2].data << std::endl;

        timer_args.request.feedback.resize(3);
        timer_args.request.feedback[0].data = req.parameters[0].data/3;
        timer_args.request.feedback[1].data = req.parameters[1].data*2;
        timer_args.request.feedback[2].data = req.parameters[2].data;
        timer.stop();
        timer.setPeriod(ros::Duration(0));
        timer.start();
        return true;
    }

    void timerCB(const ros::TimerEvent &e)
    {
        feedback.call(timer_args);
    }

    CallbackType cb = NULL;

    bool initalized = false;

    ros::NodeHandle nh;
    ros::ServiceServer tune;
    ros::ServiceClient feedback;
    ros::Timer timer;
    adap_parameter::Feedback timer_args;
};
} // namespace adap_parameter

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "test_client");

    adap_parameter::Server::Tunables t = {{{"p1"}, {"p2"}, {"p3"}},
                                          {{"f1", 0.13}, {"f2", 0.77}, {"f3", 0.02}}};
    adap_parameter::Server srv;
    srv.connect(t);

    std::cout << "A, B, C" << std::endl;

    ros::spin();

    return EXIT_SUCCESS;
}
