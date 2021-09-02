#include <stdlib.h>
#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <me120_msgs/PcSegConfig.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <me120_msgs/CloudClusterArray.h>
#include <me120_msgs/CloudCluster.h>
#include <me120_msgs/gpsPacket.h>
#include <me120_msgs/img_objs.h>
#include <me120_msgs/img_obj.h>

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
#include "gencolors.cpp"
#include "utils.h"

#include "fusions/image_trans.h"
#include "fusions/yolov5_d.h"
#include "fusions/fusion.h"

const std::string param_ns_prefix_ = "/fusion_node"; // NOLINT
ros::Publisher bboxes_pub;
ros::Publisher bboxes_world_pub;
ros::Publisher CloudCluster_pub;
ros::Publisher cloudRGB_pub;
ros::Publisher img_objs_pub;

Yolov5DET *detector = NULL;
Image_trans *transformer = NULL;

std::vector<cv::Scalar> _colors;
static cv::Mat colormap;

pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr;
std_msgs::Header current_header;

void Syncallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud, const sensor_msgs::ImageConstPtr &ori_image)
{
  // ---------------Image------------------
  cv::Mat cap_frame;

  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(ori_image, sensor_msgs::image_encodings::BGR8);
  cap_frame = cv_ptr->image;
  std::vector<cv::Mat> cap_frames;
  cap_frames.push_back(cap_frame);

  // ros::Time begin = ros::Time::now();
  detector->process_func(cap_frames);
  // double time_cost = (ros::Time::now() - begin).toSec();

  std::vector<cv::Rect> bboxes = detector->get_bboxes();
  std::vector<std::string> class_names = detector->get_class_names();
  std::vector<std::string> labels = detector->get_labels();
  std::vector<int> class_ids = detector->get_class_ids();

  // ----------------LiDAR-------------------
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPPoint(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*in_sensor_cloud, *cloudPPoint);

  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  pcl::copyPointCloud(*cloudPPoint, cloud_xyz);

  pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_cloud_ptr = cloud_xyz.makeShared();
  current_sensor_cloud_ptr = sensor_cloud_ptr;
  current_header = in_sensor_cloud->header;
  // process_func();

  // ------------------fusion-------------------
  fusion ff(current_sensor_cloud_ptr, cap_frame, transformer, bboxes);
  ff.process();

  std::vector<GroupType> groups = ff.get_Groups();

  // ---------DISPLAY-Image-Points---------------------
  // std::vector<cv::Point3d> Points_image;
  // Points_image = ff.get_PointsImage();

  // cv::Mat dis_frame1 = cap_frame.clone();
  // for (int i = 0; i < Points_image.size(); i++)
  // {
  //   cv::Point point(Points_image[i].x, Points_image[i].y);
  //   int colorid = (Points_image[i].z - 0) * 255 / 100;
  //   cv::Vec3b color = colormap.at<cv::Vec3b>(colorid);
  //   int g = color[1];
  //   int b = color[2];
  //   int r = color[0];
  //   cv::circle(dis_frame1, point, 2, cv::Scalar(b, g, r));
  // }
  // cv::imshow("display window1", dis_frame1);
  // cv::waitKey(1);

  // ---------DISPLAY-Roi-Points---------------------
  cv::Mat dis_frame2 = cap_frame.clone();
  for (int i = 0; i < groups.size(); i++)
  {
    for (int j = 0; j < groups[i].cloud.size(); j++)
    {
      cv::Point point(groups[i].cloud[j].u, groups[i].cloud[j].v);
      int colorid = (groups[i].cloud[j].d - 0) * 255 / 100;
      cv::Vec3b color = colormap.at<cv::Vec3b>(colorid);
      int g = color[1];
      int b = color[2];
      int r = color[0];
      cv::circle(dis_frame2, point, 2, cv::Scalar(b, g, r));
    }
  }
  cv::imshow("display window2", dis_frame2);
  cv::waitKey(1);

  // ---------DISPLAY-Image-Results---------------------
  std::vector<float> distances = ff.get_Distances();

  cv::Mat dis_frame3 = cap_frame.clone();
  if (labels.size() == distances.size())
  {
    for (int i = 0; i < labels.size(); i++)
    {
      char str[10];
      sprintf(str, "%.2f", distances[i]);
      std::string dis_text(str);
      labels[i] = labels[i] + " dis: " + dis_text + "m";
    }
  }
  plot_one_image(dis_frame3, bboxes, labels, class_ids);
  cv::imshow("display window3", dis_frame3);
  cv::waitKey(1);

  // ------------------Publish-ROS-Message-----------------
  me120_msgs::img_objs objs_msg;
  objs_msg.header = ori_image->header;
  for (int i = 0; i < bboxes.size(); i++)
  {
    me120_msgs::img_obj obj_msg;
    obj_msg.obj_name = class_names[i];
    obj_msg.x = bboxes[i].x;
    obj_msg.y = bboxes[i].y;
    obj_msg.w = bboxes[i].width;
    obj_msg.h = bboxes[i].height;
    std::cout << class_names[i] << ": ";
    std::cout << bboxes[i].x << " " << bboxes[i].y << " " << bboxes[i].width << " " << bboxes[i].height << std::endl;
    objs_msg.obj.push_back(obj_msg);
  }
  img_objs_pub.publish(objs_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fusion_node");
  ros::NodeHandle nh;
  img_objs_pub = nh.advertise<me120_msgs::img_objs>("/objects", 1);

  // 建立同步订阅器
  message_filters::Subscriber<sensor_msgs::PointCloud2> PointCloudInfo_sub(nh, "/rslidar_points", 1);
  message_filters::Subscriber<sensor_msgs::Image> ImageInfo_sub(nh, "/usb_cam/image_raw", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), PointCloudInfo_sub, ImageInfo_sub); //queue size=10
  sync.registerCallback(boost::bind(&Syncallback, _1, _2));

  // initializing config file reading.
  ros::NodeHandle private_nh("~");

  generateColors(_colors, 255, 30);
  cv::Mat grayscale(256, 1, CV_8UC1);
  for (int i = 0; i < 256; i++)
  {
    grayscale.at<uchar>(i) = i;
  }
  cv::applyColorMap(grayscale, colormap, cv::COLORMAP_JET);
  std::cout << "Color pigment loading finished!" << std::endl;

  // Image Detect
  std::string names_file;
  std::string engine_file;
  if (!(private_nh.getParam("names_file", names_file)))
  {
    ROS_INFO("Lack objects name file!");
    return 0;
  }
  if (!(private_nh.getParam("engine_file", engine_file)))
  {
    ROS_INFO("Lack engine file!");
    return 0;
  }
  detector = new Yolov5DET(names_file, engine_file);
  std::cout << "Succeed load yolov5 engine!" << std::endl;

  // Image Transform
  int width;
  private_nh.getParam(param_ns_prefix_ + "/image_width", width);
  int height;
  private_nh.getParam(param_ns_prefix_ + "/image_height", height);
  std::vector<double> data;
  private_nh.getParam(param_ns_prefix_ + "/CameraMat/data", data);
  cv::Mat DataK = cv::Mat(3, 3, CV_64F, data.data()).clone();
  std::vector<double>().swap(data);
  private_nh.getParam(param_ns_prefix_ + "/DistCoeff/data", data);
  cv::Mat DataD = cv::Mat(5, 1, CV_64F, data.data()).clone();
  std::vector<double>().swap(data);
  private_nh.getParam(param_ns_prefix_ + "/CameraExtrinsicMat/data", data);
  cv::Mat DataE = cv::Mat(4, 4, CV_64F, data.data()).clone();
  std::vector<double>().swap(data);

  transformer = new Image_trans(width, height, DataK, DataD, DataE);
  cv::Mat_<double> D = transformer->get_D();
  cv::Matx33d K = transformer->get_K();
  cv::Matx44d E = transformer->get_E();
  std::cout << "Distort coefficient: " << D << std::endl
            << "Camera Intrisic: " << K << std::endl
            << "Extrinsic: " << E << std::endl;

  ros::spin();
  return 0;
}
