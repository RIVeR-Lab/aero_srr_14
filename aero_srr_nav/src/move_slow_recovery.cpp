#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/recovery_behavior.h>
#include <pluginlib/class_list_macros.h>


namespace aero_srr_nav {
class MoveSlowRecovery : public nav_core::RecoveryBehavior{
private:
  std::string name_;
  bool initialized_;
  double forward_vel_;
  double forward_time_;
public:
  MoveSlowRecovery(): initialized_(false) {} 

  void initialize(std::string name, tf::TransformListener* tf,
				  costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
    if(!initialized_){
      name_ = name;

      ros::NodeHandle private_nh("~/" + name_);

      private_nh.param("forward_vel", forward_vel_, 0.1);
      private_nh.param("forward_time", forward_time_, 2.0);

      initialized_ = true;
    }
    else{
      ROS_ERROR("You should not call initialize twice on this object, doing nothing");
    }
  }

  ~MoveSlowRecovery(){}

  void runBehavior(){
    if(!initialized_){
      ROS_ERROR("This object must be initialized before runBehavior is called");
      return;
    }
    ROS_WARN("MoveSlow recovery behavior started.");

    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ros::Duration time(forward_time_);

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = forward_vel_;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub.publish(cmd_vel);

    time.sleep();

    cmd_vel.linear.x = 0.0;
    vel_pub.publish(cmd_vel);
   
  }
};

}

PLUGINLIB_DECLARE_CLASS(aero_srr_nav, MoveSlowRecovery, aero_srr_nav::MoveSlowRecovery, nav_core::RecoveryBehavior)
