#include <ros/ros.h>
#include "server.h"

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "adap_parameter_server");
    ros::NodeHandle nh("/adap_parameter");

    Server(nh).run();
}
