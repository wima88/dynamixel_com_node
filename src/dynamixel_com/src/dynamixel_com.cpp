#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include <nav_msgs/Odometry.h>

#include "mx240.h"

#include <unistd.h>




class OdemControll
{
  public :
    OdemControll()
    {
      n_.getParam("base_width",_roverBase_width);
      n_.getParam("wheel_radius",_wheelRadius);
      //topics want to publis
      odem_pub_ = n_.advertise<nav_msgs::Odometry>("odom", 50);

      //Topics wants to subscribe
      sub_ =n_.subscribe("/cmd_vel",1, &OdemControll::nodeCallBack, this);

    }
    

    void nodeCallBack (const geometry_msgs::Twist& msg)
    {
      float lin_x = msg.linear.x;
      float ang_z = msg.angular.z;

      float leftWheelSpeed = lin_x - ((_roverBase_width/2)*ang_z); // linear vel
      float rightWheelSpeed = lin_x + ((_roverBase_width/2)*ang_z);// linear vel

       int leftRpm_reg_val = ((leftWheelSpeed)/(0.10472*_wheelRadius))/0.23;
       int righttRpm_reg_val = ((rightWheelSpeed)/(0.10472*_wheelRadius))/0.23;

       ROS_INFO("left rpm = %d  right rpm = %d",leftRpm_reg_val ,righttRpm_reg_val);
       mx240.set_speed(leftRpm_reg_val,righttRpm_reg_val);
     }

    void get_param(double *base_width, double *wheel_R)
    {
      *base_width = _roverBase_width;
      *wheel_R = _wheelRadius;
    }

    void get_wheelSpeed()
    {
      int L_,R_;
      mx240.read_speed(&L_,&R_);
      ROS_INFO("left speed = %f  right rpm = %f",(L_*0.23) ,(R_*0.23));

    }

    void genOdemInfo()
    {
      //calculate linear x and anguler z 
      mx240.read_speed(&left_encdr_,&right_encdr_); // update the wheel speed factor
      float l_wheelSpeed_ = ((left_encdr_ *0.23)/60)*(_wheelRadius*6.2831); 
      float r_wheelSpeed_ = ((right_encdr_ *0.23)/60)*(_wheelRadius*6.2831); 

      vth= (r_wheelSpeed_ - l_wheelSpeed_)/_roverBase_width;
      vx = (l_wheelSpeed_ + r_wheelSpeed_)/2;
      ROS_INFO("linear speed  = %f",vx);
      ROS_INFO("angular speed  = %f",vth);
      ROS_INFO("====================");

      //current_time = ros::Time::now();
    }



  private:
    ros::NodeHandle n_  ;
    ros::Publisher odem_pub_ ;
    ros::Subscriber sub_;

    tf::TransformBroadcaster odom_broadcaster;
    
    double _roverBase_width =0.1;
    double _wheelRadius = 0.001;

    int left_encdr_, right_encdr_; // wheel speed factor


    /**************************/
    double x = 0.0;
  	double y = 0.0;
  	double th = 0.0;

  	double vx = 0.0;
  	double vy = 0.0;
  	double vth =0.0;

    //ros::Time current_time, last_time;
    //current_time = ros::Time::now();
    //last_time = ros::Time::now();
    /**************************/

    MX240 mx240;

};



int main(int argc, char *argv[]) {

    

  double roverBase_width =0.1;
  double wheelRadius = 0.001;


  ros::init(argc,argv,"dynamixel_com_Node");
  
  OdemControll odm_ctrl;

  odm_ctrl.get_param(&roverBase_width,&wheelRadius);
 
  ROS_INFO ("roverBase_width = %f" ,roverBase_width);
  ROS_INFO ("wheel Radius= %f" ,wheelRadius);

  ros::Rate loopRate(10);
 while(ros::ok())
  {
    //odm_ctrl.get_wheelSpeed();
    odm_ctrl.genOdemInfo();

		//call odem set here

    ros::spinOnce();
    loopRate.sleep();
    
  }
  return 0;

}

