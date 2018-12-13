#include <cstdlib>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sstream> // for converting the command line parameter to integer

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "cam_pub");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("video", 1);
    cv::VideoCapture cap;

    std::string source;
    if (nh.getParam("source", source))
        cap.open(source);
    else
        cap.open(0);

    if (!cap.isOpened())
    {
        ROS_ERROR_STREAM("video device cannot be opened");
        return EXIT_FAILURE;
    }

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(30);
    while (nh.ok())
    {
        cap >> frame;
        // Check if grabbed frame is actually full with some content
        if (!frame.empty())
        {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame)
                      .toImageMsg();
            pub.publish(msg);
            cv::waitKey(1);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
