//From turtlebot teleop 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

//D key
#define DRIVE_R 0x64
//A key
#define DRIVE_L 0x61
//W key
#define DRIVE_U 0x77
//S key
#define DRIVE_D 0x73
//ESC key
#define KEYCODE_Q 0x1B


//Right key
#define PAN_R 0x43
//Left key
#define PAN_L 0x44
//Up key
#define TILT_U 0x41
//Down key
#define TILT_D 0x42


class OddbotTeleop
{
public:
  OddbotTeleop();
  void keyLoop();
  void watchdog();

private:

  
  ros::NodeHandle nh_,ph_;
  double linear_, angular_;
  ros::Time first_publish_;
  ros::Time last_publish_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  void publish(double, double);
  boost::mutex publish_mutex_;

};

OddbotTeleop::OddbotTeleop():
  ph_("~"),
  linear_(0),
  angular_(0),
  l_scale_(1.0),
  a_scale_(1.0)
{
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "oddbot_teleop");
  OddbotTeleop oddbot_teleop;
  ros::NodeHandle n;

  signal(SIGINT,quit);

  boost::thread my_thread(boost::bind(&OddbotTeleop::keyLoop, &oddbot_teleop));
  
  
  ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&OddbotTeleop::watchdog, &oddbot_teleop));

  ros::spin();

  my_thread.interrupt() ;
  my_thread.join() ;
      
  return(0);
}


void OddbotTeleop::watchdog()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  if ((ros::Time::now() > last_publish_ + ros::Duration(0.15)) && 
      (ros::Time::now() > first_publish_ + ros::Duration(0.50)))
    publish(0, 0);
}

void OddbotTeleop::keyLoop()
{
  char c;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtlebot.");


  while (ros::ok())
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }


    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case DRIVE_L:
        ROS_DEBUG("DRIVE LEFT");
        angular_ = 1.0;
        break;
      case DRIVE_R:
        ROS_DEBUG("DRIVE RIGHT");
        angular_ = -1.0;
        break;
      case DRIVE_U:
        ROS_DEBUG("DRIVE UP");
        linear_ = 1.0;
        break;
      case DRIVE_D:
        ROS_DEBUG("DRIVE DOWN");
        linear_ = -1.0;
        break;
      case PAN_L:
        ROS_DEBUG("PAN LEFT");
        //DO SOMETHING
        break;
      case PAN_R:
        ROS_DEBUG("PAN RIGHT");
        //DO SOMETHING
        break;
      case TILT_U:
        ROS_DEBUG("TILT UP");
        //DO SOMETHING
        break;
      case TILT_D:
        ROS_DEBUG("TILT DOWN");
        //DO SOMETHING
        break;
    }
    boost::mutex::scoped_lock lock(publish_mutex_);
    if (ros::Time::now() > last_publish_ + ros::Duration(1.0)) { 
      first_publish_ = ros::Time::now();
    }
    last_publish_ = ros::Time::now();
    publish(angular_, linear_);
  }

  return;
}

void OddbotTeleop::publish(double angular, double linear)  
{
    geometry_msgs::Twist vel;
    vel.angular.z = a_scale_*angular;
    vel.linear.x = l_scale_*linear;

    vel_pub_.publish(vel);    


  return;
}



