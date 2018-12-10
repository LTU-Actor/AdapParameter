#include <adap_parameter/server.h>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

adap_parameter::Server *srv = NULL;
cv::VideoCapture *camera = NULL;
double next_exposure;
int have_next_exposure = 0;

bool
tuneCB(adap_parameter::Tune::Request &req, adap_parameter::Tune::Response &res)
{
    ROS_ERROR_STREAM("GOT TUNE: " << req.parameters[0].data);
    next_exposure = req.parameters[0].data;
    have_next_exposure = 1;
    return false;
}

double
calculateDetail(const cv::Mat &image)
{
    cv::Mat tmp;
    cv::GaussianBlur(image, tmp, cv::Size(3, 3), 0);
    cv::Laplacian(tmp, tmp, tmp.depth(), 3);

    double sum = 0;

    cv::MatIterator_<uchar> it, end;
    for (it = tmp.begin<uchar>(), end = tmp.end<uchar>(); it != end; ++it)
    {
        double n = (double)*it / 255.0;
        double activate = .01;
        double alpha = .5;
        double s =
            log(alpha * (n - activate) + 1) / log(alpha * (1 - activate) + 1);
        if (s > 1) s = 1;
        if (s < 0) s = 0;

        sum += s;
    }

    sum /= (tmp.rows * tmp.cols * .1);

    ROS_ERROR_STREAM(sum);

    return sum;
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "v4l_client");
    srv = new adap_parameter::Server(&tuneCB);
    camera = new cv::VideoCapture(0);
    if (!camera->isOpened())
    {
        ROS_ERROR_STREAM("Failed to open default video stream");
        return EXIT_FAILURE;
    }
    adap_parameter::Server::Tunables t = {{{"exposure"}}, {{"detail", 1}}};
    srv->connect(t);

    camera->set(CV_CAP_PROP_AUTO_EXPOSURE, 0.25);
    camera->set(CV_CAP_PROP_EXPOSURE, 0);

    std::cout << "Exposure,Detail" << std::endl;

    long loop = 0;
    while (ros::ok())
    {
        cv::Mat frame;
        *camera >> frame;

        if (frame.empty()) return EXIT_SUCCESS;

        cv::imshow("video", frame);

        if (have_next_exposure == -1)
        {
            double detail = calculateDetail(frame);
            adap_parameter::Feedback::Request fb;
            fb.feedback.resize(1);
            fb.feedback[0].data = detail;
            srv->sendFeedback(fb);
            // ROS_INFO_STREAM("Sent fb: " << detail);
            std::cout << detail << std::endl;
            have_next_exposure = 0;

            if (loop++ == 40) break;
        }
        else if (have_next_exposure == 1)
        {
            camera->set(CV_CAP_PROP_EXPOSURE, next_exposure);
            have_next_exposure = -5;
            std::cout << next_exposure << ",";
        }
        else
        {
            have_next_exposure++;
        }

        cv::waitKey(5);
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
