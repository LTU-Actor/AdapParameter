#include <adap_parameter/server.h>
#include <cmath>
#include <cstdlib>
#include <ros/ros.h>

adap_parameter::Server *srv = NULL;

bool
tuneCB(adap_parameter::Tune::Request &req, adap_parameter::Tune::Response &res)
{
    static long count = 0;
    if (count++ > 100) exit(EXIT_SUCCESS);
    adap_parameter::Feedback::Request fb;
    fb.feedback.resize(1);
    fb.feedback[0].data = std::sqrt(req.parameters[0].data);
    srv->sendFeedback(fb);

    std::cout << req.parameters[0].data << "," << fb.feedback[0].data << ","
              << "0.23" << std::endl;

    return true;
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "synth_single");
    srv = new adap_parameter::Server(&tuneCB);

    adap_parameter::Server::Tunables t = {{{"p1"}}, {{"f1", 0.23}}};

    srv->connect(t);
    std::cout << "Synthetic - Single Parameter & Feedback\nParameter "
                 "1,Feedback 1,Target"
              << std::endl;
    ros::spin();
    return EXIT_SUCCESS;
}
