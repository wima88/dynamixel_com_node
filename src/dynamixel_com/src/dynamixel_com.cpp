#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "mx240.h"
#include <unistd.h>


MX240 mx240;

/*void nodeCallBack (const geometry_msgs::Twist& msg)
{
  float lin_x = msg.linear.x;
  float ang_z;

  ROS_INFO("I heard you /cmd_vel");
  mx240.set_speed(lin_x*10,lin_x*10);
}

int main(int argc, char *argv[]) {

  ros::init(argc,argv,"dynamixel_com_Node");
  ros::NodeHandle nh;

  ros::Subscriber sub =nh.subscribe("/cmd_vel",1000, nodeCallBack);

  ros::spin();

  return 0;

}*/

uint16_t IDs[] = {0x01,0x02};
//int * rh;

int main(int argc, char *argv[]) {
  usleep(5000);
mx240.set_speed(10,10); 
  usleep(5000000);
mx240.syncRead(&PRESENT_VELOCITY,servo_ID,2);
  
  usleep(5000000);
}
