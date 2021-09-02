#include <cstdio>
#include "tf/transform_listener.h"
#include "ros/ros.h"

#define _USE_MATH_DEFINES
class echoListener
{
public:
  tf::TransformListener tf;

  //constructor with name
  echoListener()
  {
  }

  ~echoListener()
  {
  }

private:
};

int main(int argc, char **argv)
{
  //Initialize ROS
  ros::init(argc, argv, "tf_echo", ros::init_options::AnonymousName);

  // Allow 2 or 3 command line arguments

  ros::NodeHandle nh("~");
  double rate_hz;
  nh.param("rate", rate_hz, 1.0);
  ros::Rate rate(rate_hz);

  int precision(3);
  if (nh.getParam("precision", precision))
  {
    if (precision < 1)
    {
      std::cerr << "Precision must be > 0\n";
      return -1;
    }
    printf("Precision default value was overriden, new value: %d\n", precision);
  }

  //Instantiate a local listener
  echoListener echoListener;

  std::string source_frameid = "world";
  std::string target_frameid = "rslidar";

  // Wait for up to one second for the first transforms to become avaiable.
  echoListener.tf.waitForTransform(source_frameid, target_frameid, ros::Time(), ros::Duration(1.0));

  //Nothing needs to be done except wait for a quit
  //The callbacks withing the listener class
  //will take care of everything
  while (nh.ok())
  {
    try
    {
      tf::StampedTransform echo_transform;
      echoListener.tf.lookupTransform(source_frameid, target_frameid, ros::Time(), echo_transform);
      std::cout.precision(precision);
      std::cout.setf(std::ios::fixed, std::ios::floatfield);
      std::cout << "At time " << echo_transform.stamp_.toSec() << std::endl;
      double yaw, pitch, roll;
      echo_transform.getBasis().getRPY(roll, pitch, yaw);
      tf::Quaternion q = echo_transform.getRotation();
      tf::Vector3 v = echo_transform.getOrigin();
      std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
      std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
                << q.getZ() << ", " << q.getW() << "]" << std::endl
                << "            in RPY (radian) [" << roll << ", " << pitch << ", " << yaw << "]" << std::endl
                << "            in RPY (degree) [" << roll * 180.0 / M_PI << ", " << pitch * 180.0 / M_PI << ", " << yaw * 180.0 / M_PI << "]" << std::endl;

      //print transform
    }
    catch (tf::TransformException &ex)
    {
      std::cout << "Failure at " << ros::Time::now() << std::endl;
      std::cout << "Exception thrown:" << ex.what() << std::endl;
      std::cout << "The current list of frames is:" << std::endl;
      std::cout << echoListener.tf.allFramesAsString() << std::endl;
    }
    rate.sleep();
  }

  return 0;
}
