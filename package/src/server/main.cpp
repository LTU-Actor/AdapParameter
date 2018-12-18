#include "server.h"
#include <ros/ros.h>

/*
 * Short and sweet...
 * Init ros. Create a node handle here so that there is one point to change the
 * namespace of all topics if if ever needs to be changed. Create a server
 * object and transfer control to it thrugh the run function.
 */
int
main(int argc, char **argv)
{
    ros::init(argc, argv, "adap_parameter_server");
    ros::NodeHandle nh("/adap_parameter");

    Server(nh).run();
}
