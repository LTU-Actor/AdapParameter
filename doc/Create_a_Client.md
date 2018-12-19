# Creating a Client

#### Example

```c++
#include <adap_parameter/server.h>
#include <ros/ros.h>

adap_parameter::Server *srv = NULL;

bool
tuneCB(adap_parameter::Tune::Request &req, adap_parameter::Tune::Response &res)
{
    static long count = 0;
    if(count++ > 100) exit(EXIT_SUCCESS);

    adap_parameter::Feedback::Request fb;
    fb.feedback.resize(1);
    fb.feedback[0].data = req.parameters[0].data;
    srv->sendFeedback(fb);

    return true;
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "test_client");
    srv = new adap_parameter::Server(&tuneCB);

    adap_parameter::Server::Tunables t = {
        {{"param_name"}}, {{"goal_name", 0.2}}
    };

    srv->connect(t);
    ros::spin();
    return 0;
}
```


[basic.cpp](../package/src/clients/basic.cpp) is a similar file with three
parameters that can be used with the graphing script. To run it, `./graph.py -r
client_basic`
