#include "ros/ros.h"
#include <ros/console.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/Image.h>

#include "opencv2/imgcodecs.hpp"
#include <sensor_msgs/image_encodings.h>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

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
    sub = it.subscribe("/camera/image", 20, &RSDetector::process, this);
    debug_pub = it.advertise("rs_detector/debug", 10, true);

    node.param<bool>("rs_detector/debug", debug, true);

    ROS_INFO_STREAM("Roadsign Detector Initialised!");
      
  }

  void process(const sensor_msgs::ImageConstPtr& msg)
  {
    ROS_INFO_THROTTLE(60,"Processing...");
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

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Output modified video stream
    if(debug)
      debug_pub.publish(cv_ptr->toImageMsg());
  }
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "rs_detector");
  RSDetector rs_detector;

  ros::spin();

  return 0;
}