#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
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
      

      //Topics wants to subscribe
      sub_ =n_.subscribe("/cmd_vel",1, &OdemControll::nodeCallBack, this);

    }
    

    void nodeCallBack (const geometry_msgs::Twist& msg)
    {
      float lin_x = msg.linear.x;
      float ang_z = msg.angular.z;

      float leftWheelSpeed = lin_x - ((_roverBase_width/2)*ang_z);
      float rightWheelSpeed = lin_x + ((_roverBase_width/2)*ang_z);

       int leftRpm_reg_val = ((leftWheelSpeed)/(0.10472*_wheelRadius))/0.23;
       int righttRpm_reg_val = ((rightWheelSpeed)/(0.10472*_wheelRadius))/0.23;

       ROS_INFO("left rpm = %f  right rpm = %f",(leftRpm_reg_val*0.23) ,(righttRpm_reg_val*0.23));
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


  private:
    ros::NodeHandle n_  ;
    ros::Publisher pub_ ;
    ros::Subscriber sub_;
    
    double _roverBase_width =0.1;
    double _wheelRadius = 0.001;

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

  ros::Rate loopRate(30);
 while(ros::ok())
  {
    odm_ctrl.get_wheelSpeed();

    ros::spinOnce();
    loopRate.sleep();
    
  }
  return 0;

}

