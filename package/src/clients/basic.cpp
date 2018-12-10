#include <adap_parameter/server.h>
#include <ros/ros.h>
#include <cstdlib>

adap_parameter::Server *srv = NULL;

bool
tuneCB(adap_parameter::Tune::Request &req, adap_parameter::Tune::Response &res)
{
    static long count = 0;
    if(count++ > 1000) exit(EXIT_SUCCESS);
    std::cout << req.parameters[0].data << "," << req.parameters[1].data
              << "," << req.parameters[2].data << std::endl;

    adap_parameter::Feedback::Request fb;
    fb.feedback.resize(3);
    fb.feedback[0].data = req.parameters[0].data / 3;
    fb.feedback[1].data = req.parameters[1].data * 2;
    fb.feedback[2].data = req.parameters[2].data;
    srv->sendFeedback(fb);

    return true;
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "test_client");
    srv = new adap_parameter::Server(&tuneCB);

    adap_parameter::Server::Tunables t = {
        {{"p1"}, {"p2"}, {"p3"}}, {{"f1", 0.13}, {"f2", 0.77}, {"f3", 0.02}}};

    srv->connect(t);
    std::cout << "A,B,C" << std::endl;
    ros::spin();
    return EXIT_SUCCESS;
}
