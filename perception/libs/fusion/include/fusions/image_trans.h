#ifndef IMAGE_TRANS_H_
#define IMAGE_TRANS_H_

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
using namespace std;
using namespace cv;

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

class Image_trans
{
private:
    bool cache_ = false;
    cv::Mat_<double> D_;
    cv::Matx33d R_;
    cv::Matx33d K_;
    cv::Matx34d P_;
    cv::Matx44d E_;
    cv::Size dst_size_;

    cv::Mat invRt, invTt;
    cv::Point3d Point;
    bool initialized() { return cache_; }

public:
    Image_trans(int width, int height, cv::Mat K, cv::Mat D, cv::Mat E);
    ~Image_trans();

    std::vector<cv::Point3d> projectLidar3dToPixel(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
    std::vector<cv::Point3d> projectLidar3dToRoiPixel(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, const cv::Rect bbox);
    bool projectLidar3dToPixel(const pcl::PointXYZ cloud_point);
    bool projectLidar3dToRoiPixel(const pcl::PointXYZ cloud_point, const cv::Rect bbox);

    cv::Point2d project3dToPixel(const cv::Point3d &xyz);
    cv::Point3d projectPixelTo3dRay(const cv::Point2d &uv_rect);
    void rectifyImage(const sensor_msgs::Image &msg, cv::Mat &raw, cv::Mat &undistort);
    cv::Point2d rectifyPoint(const cv::Point2d &uv_raw);
    cv::Point2d unrectifyPoint(const cv::Point2d &uv_rect);
    cv::Rect rectifyRoi(const cv::Rect &roi_raw);
    cv::Rect unrectifyRoi(const cv::Rect &roi_rect);

    inline double fx();
    inline double fy();
    inline double cx();
    inline double cy();
    inline double Tx();
    inline double Ty();

    cv::Mat_<double> get_D();
    cv::Matx33d get_R();
    cv::Matx33d get_K();
    cv::Matx34d get_P();
    cv::Matx44d get_E();
    cv::Size get_dst_size();
    cv::Point3d get_Point();
};

#endif // IMAGE_TRANS_H_