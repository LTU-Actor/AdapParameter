#include <adap_parameter/server.h>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <ros/ros.h>




class ThreshTest
{
public:
    ThreshTest();

    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    bool tuneCB(adap_parameter::Tune::Request &req, adap_parameter::Tune::Response &res);
    double calculateRatio(const cv::Mat &img);

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber steer_sub_;

    std::unique_ptr<adap_parameter::Server> srv;
    std::unique_ptr<cv::VideoCapture> camera;

    double next_thresh_;
    int have_next_thresh_;

    int framecount_;
};

ThreshTest* thresh_test;


static bool callback(adap_parameter::Tune::Request &req, adap_parameter::Tune::Response &res)
{
    return thresh_test->tuneCB(req, res);
}

ThreshTest::ThreshTest() :
    nh_{"~"},
    it_{nh_}
{
    std::string camera_source_topic = "";
    if (!nh_.getParam("source", camera_source_topic))
    {
        ROS_ERROR_STREAM("Param 'source' is required");
    }
    else
    {
        ROS_INFO_STREAM("Using source '" << camera_source_topic << "'");
    }

    image_sub_ = it_.subscribe(camera_source_topic, 1, &ThreshTest::imageCb, this);

    framecount_ = 0;
    next_thresh_ = 100;
    have_next_thresh_ = 0;

    srv = std::unique_ptr<adap_parameter::Server>(new adap_parameter::Server(&callback));

    adap_parameter::Server::Tunables t = {{{"thresh"}}, {{"ratio", 0.5}}};
    srv->connect(t);

    std::cout << "Thresh,Ratio" << std::endl;
}


bool ThreshTest::tuneCB(adap_parameter::Tune::Request &req, adap_parameter::Tune::Response &res)
{
    ROS_ERROR_STREAM("GOT TUNE: " << req.parameters[0].data);
    next_thresh_ = req.parameters[0].data * 120.0f;
    have_next_thresh_ = 1;
    return false;
}

double ThreshTest::calculateRatio(const cv::Mat &img)
{
    static const constexpr int THRESH_TYPE = 0; // Binary
    static const constexpr int MAX_VAL = 255;

    cv::Mat img_gray;

    cv::cvtColor(img, img_gray, CV_BGR2GRAY );

    cv::threshold(
            img_gray,
            img_gray,
            next_thresh_,
            MAX_VAL,
            THRESH_TYPE);

    cv::imshow("binary", img_gray);

    const int white = cv::countNonZero(img_gray);
    const int total = img_gray.rows * img_gray.cols;

    // Return the ratio of white pixels
    ROS_INFO_STREAM("white/total: " << white << "/" << total);
    return (double)white / (double)total;
}


void ThreshTest::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    using std::string;

    //Convert to cv image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (cv_ptr->image.empty()) 
    {
        ROS_ERROR_STREAM("EMPTY IMAGE");
        return;
    }

    cv::imshow("orig", cv_ptr->image);

    if (have_next_thresh_ == 1)
    {
        double ratio = calculateRatio(cv_ptr->image);
        adap_parameter::Feedback::Request fb;
        fb.feedback.resize(1);
        fb.feedback[0].data = ratio;
        srv->sendFeedback(fb);
        ROS_INFO_STREAM("Thresh: " << next_thresh_ << ", Ratio:" << ratio);
        have_next_thresh_ = 0;
    }

    cv::waitKey(1);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thresh_client");


    ThreshTest t{};
    thresh_test = &t;

    ros::spin();
    return 0;
}
