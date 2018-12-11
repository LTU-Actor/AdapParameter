#include <adap_parameter/server.h>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>

adap_parameter::Server *srv = NULL;
cv::VideoCapture *camera = NULL;
double next_thresh = 100;
int have_next_thresh = 0;

bool
tuneCB(adap_parameter::Tune::Request &req, adap_parameter::Tune::Response &res)
{
    ROS_ERROR_STREAM("GOT TUNE: " << req.parameters[0].data);
    next_thresh = req.parameters[0].data * 120.0f;
    have_next_thresh = 1;
    return false;
}

double
calculateRatio(const cv::Mat &img)
{
    static const constexpr int THRESH_TYPE = 0; // Binary
    static const constexpr int MAX_VAL = 255;

    cv::Mat img_gray;
    cv::cvtColor(img, img_gray, CV_BGR2GRAY );

    cv::threshold(
            img_gray,
            img_gray,
            next_thresh,
            MAX_VAL,
            THRESH_TYPE);

    cv::imshow("binary", img_gray);

    const int white = cv::countNonZero(img_gray);
    const int total = img_gray.rows * img_gray.cols;

    // Return the ratio of white pixels
    ROS_INFO_STREAM("white/total: " << white << "/" << total);
    return (double)white / (double)total;
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "thresh_client");
    srv = new adap_parameter::Server(&tuneCB);
    camera = new cv::VideoCapture(0);
    if (!camera->isOpened())
    {
        ROS_ERROR_STREAM("Failed to open default video stream");
        return EXIT_FAILURE;
    }

    adap_parameter::Server::Tunables t = {{{"thresh"}}, {{"ratio", 0.5}}};
    srv->connect(t);

    std::cout << "Thresh,Ratio" << std::endl;

    long loop = 0;
    while (ros::ok())
    {
        cv::Mat frame;
        *camera >> frame;

        if (frame.empty()) return EXIT_SUCCESS;

        cv::imshow("video", frame);

        if (have_next_thresh == -1)
        {
            double ratio = calculateRatio(frame);
            adap_parameter::Feedback::Request fb;
            fb.feedback.resize(1);
            fb.feedback[0].data = ratio;
            srv->sendFeedback(fb);
            // ROS_INFO_STREAM("Sent fb: " << ratio);
            //std::cout << neratio << std::endl;
            ROS_INFO_STREAM("Thresh: " << next_thresh << ", Ratio:" << ratio);
            have_next_thresh = 0;

        }
        else if (have_next_thresh == 1)
        {
            have_next_thresh = -5;
            std::cout << next_thresh << ",";
        }
        else
        {
            have_next_thresh++;
        }

        cv::waitKey(5);
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
