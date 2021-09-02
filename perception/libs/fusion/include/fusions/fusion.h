#ifndef FUSION_H_
#define FUSION_H_

#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <map>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <me120_msgs/PcSegConfig.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <me120_msgs/CloudClusterArray.h>
#include <me120_msgs/CloudCluster.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>

#include <pcl/common/common.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/version.hpp>

#include "point_types.h"
#include "fusions/image_trans.h"

struct GroupType
{
    UVPointCloud cloud;
    float distance;
};

class fusion
{
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    cv::Mat image_;
    std::vector<cv::Rect> bboxes_;
    Image_trans *trans_;

    std::vector<GroupType> groups;

    float getShortest(const std::vector<float> &candidates);
    float getMedian(const std::vector<float> &candidates);
    float getMode(const std::vector<float> &candidates);

public:
    fusion(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Mat image, Image_trans *trans, std::vector<cv::Rect> bboxes);
    ~fusion();
    void process();
    std::vector<cv::Point3d> get_PointsImage();
    std::vector<GroupType> get_Groups();
    std::vector<float> get_Distances();
};

#endif // FUSION_H_