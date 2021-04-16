#include "ros/ros.h"
#include <ros/console.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/Image.h>

#include "opencv2/imgcodecs.hpp"
#include <sensor_msgs/image_encodings.h>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


class RSDetector
{
private:
  ros::NodeHandle node;

  image_transport::ImageTransport it;
  image_transport::Subscriber sub;
  image_transport::Publisher debug_pub;

  bool debug;

public:
  RSDetector()
    : it(node)
  {
    // subscribe to camera image stream
    sub = it.subscribe("/camera/image", 20, &RSDetector::processImg, this);
    debug_pub = it.advertise("rs_detector/debug", 10, true);

    node.param<bool>("rs_detector/debug", debug, true);

    ROS_INFO_STREAM("Roadsign Detector Initialised!");
      
  }

  void processImg(const sensor_msgs::ImageConstPtr& msg)
  {
    ROS_INFO_THROTTLE(60,"Processing...");
    cv_bridge::CvImagePtr cvPtr;
    try
    {
      cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat imgOriginal = cvPtr->image;
    auto rows = imgOriginal.rows;
    auto cols = imgOriginal.cols;
    // crop image to restrain excessive processing
    cv::Mat croppedImage = cv::Mat(imgOriginal, cv::Rect(cols / 2, 0, cols / 2, rows / 2));

    processRed(croppedImage);
    // processBlue(imgOriginal);
  }

  void processRed(const cv::Mat imgOriginal)
  {
    // convert image to yuv
    cv::Mat imgYUV;
    cv::cvtColor(imgOriginal, imgYUV, CV_RGB2YUV);

    // equalize Y channel
    std::vector<cv::Mat> channels;
    cv::split(imgYUV, channels);
    cv::equalizeHist(channels[0], channels[0]);
    cv::merge(channels, imgYUV);

    // convert yuv to bgr
    cv::Mat imgBGR;
    cv::cvtColor(imgYUV, imgBGR, CV_YUV2BGR);

    // convert bgr to hsv
    cv::Mat imgHSV;
    cv::cvtColor(imgOriginal, imgHSV, CV_RGB2HSV);

    // mask red
    cv::Mat maskLow;
    cv::inRange(imgHSV, cv::Scalar(0, 70, 60), cv::Scalar(10, 255, 255), maskLow);
    cv::Mat maskHigh;
    cv::inRange(imgHSV, cv::Scalar(170, 70, 60), cv::Scalar(180, 255, 255), maskHigh);
    cv::Mat redMask;
    // cv::Mat mask;
    cv::bitwise_or(maskLow, maskHigh, redMask);
    // cv::bitwise_and(imgBGR, imgBGR, redMask, mask);

    // blur mask
    cv::Mat blurred;
    cv::medianBlur(redMask, blurred, 5);

    // detect circles
    std::vector<cv::Vec3f> circles;
    HoughCircles(blurred, circles, cv::HOUGH_GRADIENT, 1,
                 blurred.rows/16,  // change this value to detect circles with different distances to each other
                 100, 30, 10, 300 // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
    );

    // draw circles
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);
        // circle center
        cv::circle(imgOriginal, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
        // circle outline
        int radius = c[2];
        cv::circle(imgOriginal, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
    }
    
    // Output modified video stream
    auto msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", imgOriginal).toImageMsg();
    if(debug)
      debug_pub.publish(msg);

  }

  void processBlue(const cv::Mat imgOriginal)
  {
    // convert image to yuv
    cv::Mat imgYUV;
    cv::cvtColor(imgOriginal, imgYUV, CV_RGB2YUV);

    // equalize Y channel
    std::vector<cv::Mat> channels;
    cv::split(imgYUV, channels);
    cv::equalizeHist(channels[0], channels[0]);
    cv::merge(channels, imgYUV);

    // convert yuv to bgr
    cv::Mat imgBGR;
    cv::cvtColor(imgYUV, imgBGR, CV_YUV2BGR);

    // convert bgr to hsv
    cv::Mat imgHSV;
    cv::cvtColor(imgOriginal, imgHSV, CV_RGB2HSV);

    // mask blue
    cv::Mat mask;
    cv::inRange(imgHSV, cv::Scalar(94, 127, 20), cv::Scalar(126, 255, 200), mask);
    cv::Mat blueMask;
    cv::bitwise_and(imgBGR, imgBGR, blueMask, mask);
    
    // median filter
    cv::medianBlur(blueMask, blueMask, 5);
    std::vector<cv::Mat> blueMaskChannels;
    cv::split(blueMask, blueMaskChannels);
    // create blue gray space
    cv::Mat filteredB = -0.5 * blueMaskChannels[2] * blueMaskChannels[0] - 2 * blueMaskChannels[1];

    // // apply mser to detect regions of interest
    // auto mser_blue = cv::MSER::create(8, 400, 4000);
    // std::vector<std::vector<cv::Point>> regions;
    // std::vector<cv::Rect> mser_bbox;
    // mser_blue->detectRegions(filteredB, regions, mser_bbox);
    
    // std::vector<std::vector<cv::Point>> hulls(regions.size());
    // for(auto i = 0; i < regions.size(); i++)
    // {
    //   cv::convexHull(regions[i], hulls[i]);
    // }
    // cv::Mat blank = cv::Mat::zeros(cv::Size(blueMask.cols, blueMask.rows),CV_8UC1);
    // cv::fillPoly(blank, hulls, cv::Scalar(255, 0, 0));


    // Output modified video stream
    // cvPtr->encoding = sensor_msgs::image_encodings::RGB8;
    cv_bridge::CvImagePtr cvPtr;
    cvPtr->image = blueMask;
    if(debug)
      debug_pub.publish(cvPtr->toImageMsg());
  }
};





int main(int argc, char **argv)
{
  ros::init(argc, argv, "rs_detector");
  RSDetector rs_detector;

  ros::spin();

  return 0;
}