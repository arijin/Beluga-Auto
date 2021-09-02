#include "math.h"
#include <iostream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

#include "std_msgs/Header.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "me120_msgs/gpsPacket.h"

// #define M_PI 3.14159265359

const double WORLD_LATITUDE = 31.5141625396;  // 22.0179797910
const double WORLD_LONGITUDE = 121.132606168; // 113.7049594031

double world_latitude, world_longitude, map_latitude, map_longitude;
bool the_first_flag = false;

double radians(double degree)
{
  double rad = degree / 180 * M_PI;
  return rad;
}

double square(double x)
{
  return pow(x, 2);
}

/*
* @see https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames for explanation of these calculations.
*/
int long2tilex(double lon, int z)
{
  return (int)(floor((lon + 180.0) / 360.0 * (1 << z)));
}

// 这里为了和rviz_satellite选取的tile坐标对上，最后的tiley加上了1
int lat2tiley(double lat, int z)
{
  double latrad = lat * M_PI / 180.0;
  return (int)(floor((1.0 - asinh(tan(latrad)) / M_PI) / 2.0 * (1 << z)) + 1);
}

double tilex2long(int x, int z)
{
  return x / (double)(1 << z) * 360.0 - 180;
}

double tiley2lat(int y, int z)
{
  double n = M_PI - 2.0 * M_PI * y / (double)(1 << z);
  return 180.0 / M_PI * atan(0.5 * (exp(n) - exp(-n)));
}

/* static reference:  https://github.com/mikalhart/TinyGPSPlus*/
double distanceBetween(double lat1, double long1, double lat2, double long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1 - long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = square(delta);
  delta += square(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

double courseTo(double lat1, double long1, double lat2, double long2)
{
  // returns course in radians (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  double dlon = radians(long2 - long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += M_PI * 2;
  }
  return a2;
}

void toXYCoordinate(double lat1, double long1, double lat2, double long2, double &x, double &y)
{
  double radian = courseTo(lat1, long1, lat2, long2);
  double distance = distanceBetween(lat1, long1, lat2, long2);
  y = cos(radian) * distance;
  x = sin(radian) * distance;
}

void MapPoseCb(const sensor_msgs::NavSatFixConstPtr &SatCoordinate_msg)
{

  map_latitude = SatCoordinate_msg->latitude;
  map_longitude = SatCoordinate_msg->longitude;
  int map_tilex, map_tiley;
  map_tilex = long2tilex(SatCoordinate_msg->longitude, 18);
  map_tiley = lat2tiley(SatCoordinate_msg->latitude, 18);
  map_longitude = tilex2long(map_tilex, 18);
  map_latitude = tiley2lat(map_tiley, 18);

  if (the_first_flag == false)
  {
    std::cout << "start!" << std::endl;
    world_latitude = WORLD_LATITUDE;   // map_latitude;
    world_longitude = WORLD_LONGITUDE; // map_longitude;
    the_first_flag = true;
  }

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  double poseX, poseY;
  toXYCoordinate(world_latitude, world_longitude, map_latitude, map_longitude, poseX, poseY);
  transform.setOrigin(tf::Vector3(poseX, poseY, 0));

  geometry_msgs::Quaternion quat_msg;
  quat_msg.x = 0;
  quat_msg.y = 0;
  quat_msg.z = 0;
  quat_msg.w = 1;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(quat_msg, quat);
  transform.setRotation(quat);

  br.sendTransform(tf::StampedTransform(transform, SatCoordinate_msg->header.stamp, "world", "map")); //world-->map,
  return;
}

void BlinkPoseCb(const me120_msgs::gpsPacketConstPtr &data_msg)
{
  if (the_first_flag == false)
    return;

  static tf::TransformBroadcaster br;
  tf::Transform transform;

  double poseX, poseY;
  toXYCoordinate(world_latitude, world_longitude, data_msg->SatCoordinate.latitude, data_msg->SatCoordinate.longitude, poseX, poseY); // map_latitude, map_longitude
  transform.setOrigin(tf::Vector3(poseX, poseY, 0));
  // std::cout << poseX << " " << poseY << std::endl;
  geometry_msgs::Quaternion quat_msg;
  quat_msg = data_msg->pose.orientation;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(quat_msg, quat);
  transform.setRotation(quat);

  br.sendTransform(tf::StampedTransform(transform, data_msg->header.stamp, "world", "base_link")); // ros::Time::now()
  br.sendTransform(tf::StampedTransform(transform, data_msg->header.stamp, "world", "rslidar"));
  return;
}

int main(int argc, char **argv)
{
  the_first_flag = false;
  ros::init(argc, argv, "tf_broadcaster_node");
  ros::NodeHandle node;

  ros::Subscriber SatFix_data_sub = node.subscribe("/gps/fix_rectified", 1, MapPoseCb);
  ros::Subscriber gps_full_data_sub = node.subscribe("/gps_data_rectified", 1, BlinkPoseCb);

  ros::spin();
  return 0;
};
