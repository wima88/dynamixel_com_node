#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_broadcaster.h>
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
      odom_pub_ = n_.advertise<nav_msgs::Odometry>("odom", 50);

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

    void broadcast_OdemInfo()
    {
      //calculate linear x and anguler z 
      if(mx240.read_speed(&left_encdr_,&right_encdr_))
      {
        float l_wheelSpeed_ = ((left_encdr_ *0.23)/60)*(_wheelRadius*6.2831); 
        float r_wheelSpeed_ = ((right_encdr_ *0.23)/60)*(_wheelRadius*6.2831); 

        vth= (r_wheelSpeed_ - l_wheelSpeed_)/_roverBase_width;
        vx = (l_wheelSpeed_ + r_wheelSpeed_)/2;
      
        ROS_INFO("linear speed  = %f",vx);
        ROS_INFO("angular speed  = %f",vth);
        ROS_INFO("====================");

        current_time = ros::Time::now();

        double dt = (current_time - last_time).toSec();       // time diferance
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;  //x distance
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;  //y distance
        double delta_th = vth * dt;                           //anguler distance

        //update the total travel distance in x,y and anguler
			  x += delta_x;
    	  y += delta_y;
    	  th += delta_th;
        ROS_INFO(" travel x =%f",x);

        //since all odometry is 6DOF we'll need a quaternion created from yaw
	      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

			  //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

			  //next, we'll publish the odometry message over ROS
    	  nav_msgs::Odometry odom;
    	  odom.header.stamp = current_time;
    	  odom.header.frame_id = "odom";

   		  //set the position
    	  odom.pose.pose.position.x = x;
    	  odom.pose.pose.position.y = y;
    	  odom.pose.pose.position.z = 0.0;
    	  odom.pose.pose.orientation = odom_quat;

    	  //set the velocity
    	  odom.child_frame_id = "base_link";
    	  odom.twist.twist.linear.x = vx;
    	  odom.twist.twist.linear.y = vy;
    	  odom.twist.twist.angular.z = vth;

    	  //publish the message
    	  odom_pub_.publish(odom);

        last_time = current_time;
      }//end if for wheel speed check
      else
      {
        ROS_WARN("Error in wheel Speed");
      }
    }



  private:
    ros::NodeHandle n_  ;
    ros::Publisher odom_pub_ ;
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

    ros::Time current_time= ros::Time::now();
    ros::Time last_time = ros::Time::now();
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

    ros::spinOnce();
    odm_ctrl.broadcast_OdemInfo();
    loopRate.sleep();
    
  }
  return 0;

}

