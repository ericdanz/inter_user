#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "sensor_msgs/Image.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"

image_transport::Subscriber sub;
image_transport::Publisher pub;

void imageCallback(const sensor_msgs::Image::ConstPtr& img_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
    
    pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_compressor");
  ros::NodeHandle nh;
  ROS_INFO("Image compressor node started!");
  image_transport::ImageTransport it(nh);
  sub = it.subscribe("camera/rgb/image_raw", 1, imageCallback);
  pub = it.advertise("camera/image_compressed", 1);
  ros::Rate loop_rate(30);
  while (nh.ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
