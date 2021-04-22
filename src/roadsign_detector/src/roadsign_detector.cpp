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

#include <algorithm>

class RSDetector
{
private:
  ros::NodeHandle node;

  image_transport::ImageTransport it;
  image_transport::Subscriber sub;
  image_transport::Publisher debugRed, debugBlue;
  image_transport::Publisher signRed, signBlue;

  bool debug;
  // tuning variables
  float minArea, maxArea;
  float aspectRatioError;
  int signSize;

public:
  RSDetector()
    : it(node)
  {
    // subscribe and publish topics
    sub = it.subscribe("/camera/image", 20, &RSDetector::processImg, this);
    debugRed = it.advertise("rs_detector/debug_red", 10, false);
    debugBlue = it.advertise("rs_detector/debug_blue", 10, false);
    signRed = it.advertise("rs_detector/sign_red", 10, true);
    signBlue = it.advertise("rs_detector/sign_blue", 10, true);

    // load launch parameters
    node.param<bool>("debug", debug, true);
    node.param<float>("min_area", minArea, 5000);
    node.param<float>("max_area", maxArea, 60000);
    node.param<float>("aspect_ratio_error", aspectRatioError, 0.15);
    node.param<int>("sign_size", signSize, 60);

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
    // crop image's right top corner to restrain excessive processing
    cv::Mat croppedImage = cv::Mat(imgOriginal, cv::Rect(cols / 2, 0, cols / 2, rows / 2));

    // TODO thredding
    processRed(croppedImage.clone());
    processBlue(croppedImage.clone());
  }

  /**
   * Check area criterion for bounding rectangle.
   *
   * Area is compared with min_area and max_area parameters given to node.
   *
   * @param r Rectangle whose area is to be checked.
   * @return 'true' if area is within range, 'false' otherwise. 
   */
  bool isAreaOK(cv::Rect r)
  {
    return r.area() > minArea && r.area() < maxArea;
  }

  /**
   * Check aspect ratio criterion for bounding rectangle.
   *
   * Aspect ratio is calculated by division of width by height.
   * Absolute value of error is compared with aspect_ratio_error parameter given to node. 
   *
   * @param r Rectangle whose aspect ratio is to be checked.
   * @return 'true' if aspect ratio is within range, 'false' otherwise. 
   */
  bool isAspectRatioOK(cv::Rect r)
  {
    return fabs((float)r.width / (float)r.height - 1.0) < aspectRatioError;
  }

  /**
   * Clusterize and draw blobs in image.
   *
   * White pixels are found in the image. Then they are partitioned into clusters.
   * Then colored cluster points are drawn on a new image along with their centroids.
   *
   * @param img 1-channel image where blobs will be detected. Output 3-channel image with drawn clusters.
   * @return Vector with clusters.
   */
  std::vector<std::vector<cv::Point>> clusterize(cv::Mat &img)
  {
    // Get all non black points
    std::vector<cv::Point> pts;
    cv::findNonZero(img, pts);
    // Define the distance between clusters
    int euclidean_distance = 50;

    // Apply partition 
    // All pixels within the the given distance will belong to the same cluster
    std::vector<int> labels;
    int th2 = euclidean_distance * euclidean_distance;
    int n_labels = cv::partition(pts, labels, [th2](const cv::Point& lhs, const cv::Point& rhs) {
        return ((lhs.x - rhs.x)*(lhs.x - rhs.x) + (lhs.y - rhs.y)*(lhs.y - rhs.y)) < th2;
    });

    // Store all points in same cluster, and compute centroids
    std::vector<std::vector<cv::Point>> clusters(n_labels);
    std::vector<cv::Point> centroids(n_labels, cv::Point(0,0));

    for (int i = 0; i < pts.size(); ++i)
    {
        clusters[labels[i]].push_back(pts[i]);
        centroids[labels[i]] += pts[i];
    }
    for (int i = 0; i < n_labels; ++i)
    {
        centroids[i].x /= clusters[i].size();
        centroids[i].y /= clusters[i].size();
    }

    // Draw results

    // Build a vector of random color, one for each class (label)
    std::vector<cv::Vec3b> colors;
    for (int i = 0; i < n_labels; ++i)
    {
        colors.push_back(cv::Vec3b(rand() & 255, rand() & 255, rand() & 255));
    }

    // Draw the points
    cv::Mat3b res(img.rows, img.cols, cv::Vec3b(0, 0, 0));
    for (int i = 0; i < pts.size(); ++i)
    {
        res(pts[i]) = colors[labels[i]];
    }

    // Draw centroids
    for (int i = 0; i < n_labels; ++i)
    {
        circle(res, centroids[i], 3, cv::Scalar(colors[i][0], colors[i][1], colors[i][2]), CV_FILLED);
        circle(res, centroids[i], 6, cv::Scalar(255 - colors[i][0], 255 - colors[i][1], 255 - colors[i][2]));
    }

    return clusters;
  }

  /**
   * Detect circles on given image
   *
   * Circles are detected by HoughCircles. Min radius is 10, max is 300
   *
   * @param img Image on which circles will be detected.
   * @return Vector of detected circles.
   */
  std::vector<cv::Vec3f> detectCircles(cv::Mat img)
  {
    // detect circles
    std::vector<cv::Vec3f> circles;
    HoughCircles(img, circles, cv::HOUGH_GRADIENT, 1,
                 img.rows/16,  // change this value to detect circles with different distances to each other
                 100, 30, 10, 300 // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
    );

    return circles;
  }

  /**
   * Draw given circles on given image.
   *
   * Circles outlines are drawn in purple color, centers in green
   *
   * @param img 3-channel image, returned image
   * @param circles vector of circles detected by Hough transform
   */
  void drawCircles(cv::Mat &img, std::vector<cv::Vec3f> circles)
  {
    // draw circles
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);
        // circle center
        cv::circle(img, center, 1, cv::Scalar(0,255,0), 3, cv::LINE_AA);
        // circle outline
        int radius = c[2];
        cv::circle(img, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
    }
  }


  /**
   * Filter contours by size and area
   *
   * Contours are filtered by their bounding box area and aspect ratio.
   *
   * @param contours Vector of contours to be filtered.
   * @return Vector of rectangles fitting the criteria.
   */
  std::vector<cv::Rect> filterContours(std::vector<std::vector<cv::Point>> contours)
  {
    // create bounding rectangles
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::Rect> rects;

    for( size_t i = 0; i < contours.size(); i++ )
    {
      approxPolyDP( contours[i], contours_poly[i], 3, true);
      auto rect = boundingRect(contours_poly[i]);
      // filter rectangles by area and aspect ratio
      if(isAreaOK(rect) && isAspectRatioOK(rect))
      {
        rects.push_back(rect);
      }
    }

    return rects;
  }


  /**
   * Crop given rectangles from the image
   *
   * @param img Image from which rectangles will be cropped.
   * @param img Vector of rectangles. Rectangle coords must be inside the image.
   * @return Vector of cropped images.
   */
  std::vector<cv::Mat> cropRects(const cv::Mat img, std::vector<cv::Rect> rects)
  {
    std::vector<cv::Mat> croppedRects;
    for(auto r : rects)
    {
      cv::Mat croppedImage(img, r);
      croppedRects.push_back(croppedImage);
    }
    return croppedRects;
  }

  /**
   * Detect red signs and publish them.
   *
   * Detect signs based on color segmentation and contours' properties. Signs will be scaled
   * accordingly to given sign_size node's parameter.
   *
   * @param imgOriginal RGB image from which signs will be extracted. 
   */
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

    // TODO enhance equalization so it's usable

    // convert orginal to hsv
    cv::Mat imgHSV;
    cv::cvtColor(imgOriginal, imgHSV, CV_RGB2HSV);

    // mask red
    cv::Mat maskLow;
    cv::inRange(imgHSV, cv::Scalar(0, 70, 60), cv::Scalar(10, 255, 255), maskLow);
    cv::Mat maskHigh;
    cv::inRange(imgHSV, cv::Scalar(170, 70, 60), cv::Scalar(180, 255, 255), maskHigh);
    cv::Mat redMask;
    cv::bitwise_or(maskLow, maskHigh, redMask);

    // blur mask 5x5
    cv::Mat blurred;
    cv::medianBlur(redMask, blurred, 5);

    // erode mask 3x3
    cv::erode(redMask, redMask, cv::getStructuringElement(cv::MORPH_ERODE,
                        cv::Size(3, 3), cv::Point(1, 1)));

    // dilate mask 5x5
    // cv::dilate(redMask, redMask, cv::getStructuringElement( cv::MORPH_OPEN,
    //                     cv::Size(5, 5), cv::Point(2, 2));

    // open mask 5x5
    // cv::morphologyEx(redMask, redMask, cv::MORPH_OPEN, cv::getStructuringElement( cv::MORPH_OPEN,
    //                     cv::Size(5, 5), cv::Point(2, 2));

    // detect contours
    cv::Mat canny_output;
    cv::Canny(redMask, canny_output, 250, 255); // thershold doesn't matter cos mask is binary
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    // find only external contours (only supreme parents to prevent having another contour inside)
    cv::findContours(canny_output, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // get the moments
    // std::vector<cv::Moments> mu(contours.size() );
    // for( int i = 0; i < contours.size(); i++ )
    //   { mu[i] = moments( contours[i], false ); }

    //  get the mass centers - can be useful for tracking
    // std::vector<cv::Point2f> mc( contours.size() );
    // for( int i = 0; i < contours.size(); i++ )
    //   { mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }
      
      
    // filter contours to get rects of potential signs
    auto rects = filterContours(contours);

    // crop and return signs
    auto signs = cropRects(imgOriginal, rects);

    // scale and output detected signs
    for(auto sign : signs)
    {
      cv::Mat resizedSign(cv::Size(signSize,signSize), CV_8UC3);
      cv::resize(sign, resizedSign, resizedSign.size());  
      auto msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", resizedSign).toImageMsg();

      signRed.publish(msg);
    }

    if(debug)
    {
      // draw debug image
      for( size_t i = 0; i < contours.size(); i++ )
      {
        cv::drawContours(imgOriginal, contours, (int)i, cv::Scalar(0, 255, 0), 1/*cv::FILLED*/, cv::LINE_8, hierarchy, 0 );
      }
      for( size_t i = 0; i < rects.size(); i++ )
      {
        rectangle(imgOriginal, rects[i].tl(), rects[i].br(), cv::Scalar(255, 0, 255), 2 );
      }

      // Output modified video stream
      auto msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", imgOriginal).toImageMsg();
  
      debugRed.publish(msg);
    }

  }

  /**
   * Detect blue signs and publish them.
   *
   * Detect signs based on color segmentation and contours' properties. Signs will be scaled
   * accordingly to given sign_size node's parameter.
   *
   * @param imgOriginal RGB image from which signs will be extracted. 
   */
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

    // TODO enhance equalization so it's usable

    // convert orginal to hsv
    cv::Mat imgHSV;
    cv::cvtColor(imgOriginal, imgHSV, CV_RGB2HSV);

    // mask blue
    cv::Mat blueMask;
    cv::inRange(imgHSV, cv::Scalar(94, 127, 20), cv::Scalar(126, 255, 200), blueMask);
    
        // blur mask 5x5
    cv::Mat blurred;
    cv::medianBlur(blueMask, blurred, 5);

    // erode mask 3x3
    cv::erode(blueMask, blueMask, cv::getStructuringElement(cv::MORPH_ERODE,
                        cv::Size(3, 3), cv::Point(1, 1)));

    // dilate mask 5x5
    // cv::dilate(blueMask, blueMask, cv::getStructuringElement( cv::MORPH_OPEN,
    //                     cv::Size(5, 5), cv::Point(2, 2));

    // open mask 5x5
    // cv::morphologyEx(blueMask, blueMask, cv::MORPH_OPEN, cv::getStructuringElement( cv::MORPH_OPEN,
    //                     cv::Size(5, 5), cv::Point(2, 2));

    // detect contours
    cv::Mat canny_output;
    cv::Canny(blueMask, canny_output, 250, 255); // thershold doesn't matter cos mask is binary
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    // find only external contours (only supreme parents to prevent having another contour inside)
    cv::findContours(canny_output, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // get the moments
    // std::vector<cv::Moments> mu(contours.size() );
    // for( int i = 0; i < contours.size(); i++ )
    //   { mu[i] = moments( contours[i], false ); }

    //  get the mass centers - can be useful for tracking
    // std::vector<cv::Point2f> mc( contours.size() );
    // for( int i = 0; i < contours.size(); i++ )
    //   { mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }
      
      
    // filter contours to get rects of potential signs
    auto rects = filterContours(contours);

    // crop and return signs
    auto signs = cropRects(imgOriginal, rects);

    // scale and output detected signs
    for(auto sign : signs)
    {
      cv::Mat resizedSign(cv::Size(signSize,signSize), CV_8UC3);
      cv::resize(sign, resizedSign, resizedSign.size());  
      auto msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", resizedSign).toImageMsg();

      signBlue.publish(msg);
    }

    if(debug)
    {
      // draw debug image
      for( size_t i = 0; i < contours.size(); i++ )
      {
        cv::drawContours(imgOriginal, contours, (int)i, cv::Scalar(0, 255, 0), 1/*cv::FILLED*/, cv::LINE_8, hierarchy, 0 );
      }
      for( size_t i = 0; i < rects.size(); i++ )
      {
        rectangle(imgOriginal, rects[i].tl(), rects[i].br(), cv::Scalar(255, 0, 255), 2 );
      }

      // Output modified video stream
      auto msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", imgOriginal).toImageMsg();
  
      debugBlue.publish(msg);
    }
  }
};





int main(int argc, char **argv)
{
  ros::init(argc, argv, "rs_detector");
  RSDetector rs_detector;

  ros::spin();

  return 0;
}