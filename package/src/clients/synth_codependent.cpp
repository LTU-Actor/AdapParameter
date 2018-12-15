#include <adap_parameter/server.h>
#include <cstdlib>
#include <ros/ros.h>

adap_parameter::Server *srv = NULL;

bool
tuneCB(adap_parameter::Tune::Request &req, adap_parameter::Tune::Response &res)
{
    static long count = 0;
    if (count++ > 100) exit(EXIT_SUCCESS);

    adap_parameter::Feedback::Request fb;
    fb.feedback.resize(3);

    float a = req.parameters[0].data;
    float b = req.parameters[1].data;
    float c = req.parameters[2].data;

    fb.feedback[0].data = a + b;
    fb.feedback[1].data = b + c;
    fb.feedback[2].data = c + a;
    srv->sendFeedback(fb);

    std::cout << fb.feedback[0].data << "," << fb.feedback[1].data << ","
              << fb.feedback[2].data << "," << 0.13 << "," << 0.77 << "," << 0.5
              << std::endl;

    return true;
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "synth_codependent");
    srv = new adap_parameter::Server(&tuneCB);

    adap_parameter::Server::Tunables t = {
        {{"p1"}, {"p2"}, {"p3"}}, {{"f1", 0.13}, {"f2", 0.77}, {"f3", 0.5}}};

    srv->connect(t);
    std::cout << "Synthetic - Codependent Parameters\nFeedback 1,Feedback "
                 "2,Feedback 3,Target 1,Target 2,Target 3"
              << std::endl;
    ros::spin();
    return EXIT_SUCCESS;
}
