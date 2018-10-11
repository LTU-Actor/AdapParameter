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

private:
    bool tuneCB(adap_parameter::Tune::Request &req,
                adap_parameter::Tune::Response &res)
    {
    }

    CallbackType cb = NULL;
    void setCallback(CallbackType cb) { this->cb = cb; }

    bool initalized = false;

    ros::NodeHandle nh;

    ros::ServiceServer tune;
    ros::ServiceClient feedback;
};
} // namespace adap_parameter

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "test_client");

    adap_parameter::Server::Tunables t = {{{"a"}, {"p2"}, {"p3"}},
                                          {{"f1", 1}, {"f2", 2}, {"f3", 3}}};
    adap_parameter::Server srv;
    srv.connect(t);

    ros::spin();

    return EXIT_SUCCESS;
}
