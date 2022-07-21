// Roland Arsenault
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright 2021, All rights reserved.

#include "ros/ros.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"

ros::Publisher mru_position_pub;
ros::Publisher mru_orientation_pub;
ros::Publisher mru_velocity_pub;

template <typename T> void sensorCallback(const sensor_msgs::NavSatFix::ConstPtr & position, const sensor_msgs::Imu::ConstPtr & orientation, const typename T::ConstPtr & velocity);


template <> void sensorCallback<geometry_msgs::TwistWithCovarianceStamped>(const sensor_msgs::NavSatFix::ConstPtr & position, const sensor_msgs::Imu::ConstPtr & orientation, const geometry_msgs::TwistWithCovarianceStamped::ConstPtr & velocity)
{
  mru_position_pub.publish(position);

  // hack to fake synchronized data, copy timestamps from position to others
  sensor_msgs::Imu imu = *orientation;
  imu.header.stamp = position->header.stamp;
  mru_orientation_pub.publish(imu);

  geometry_msgs::TwistWithCovarianceStamped twcs = *velocity;
  twcs.header.stamp = position->header.stamp;
  mru_velocity_pub.publish(twcs);
}

template <> void sensorCallback<geometry_msgs::TwistStamped>(const sensor_msgs::NavSatFix::ConstPtr & position, const sensor_msgs::Imu::ConstPtr & orientation, const geometry_msgs::TwistStamped::ConstPtr & velocity)
{
  mru_position_pub.publish(position);

  sensor_msgs::Imu imu = *orientation;
  imu.header.stamp = position->header.stamp;
  mru_orientation_pub.publish(imu);
  
  geometry_msgs::TwistWithCovarianceStamped twcs;
  twcs.header = velocity->header;
  twcs.header.stamp = position->header.stamp;
  twcs.twist.twist = velocity->twist;
  mru_velocity_pub.publish(twcs);
}

template<typename T> void run(ros::NodeHandle& n)
{
  std::string in_topic_prefix;
  ros::param::param<std::string>("~inTopicPrefix", in_topic_prefix, "in/");
  message_filters::Subscriber<sensor_msgs::NavSatFix> position_sub(n, in_topic_prefix+"position", 1);
  message_filters::Subscriber<sensor_msgs::Imu> orientation_sub(n, in_topic_prefix+"orientation", 1);
  message_filters::Subscriber<T> velocity_sub(n, in_topic_prefix+"velocity", 1);
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, sensor_msgs::Imu, T> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), position_sub, orientation_sub, velocity_sub);
  sync.registerCallback(std::bind(&sensorCallback<T>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_sync");
    ros::NodeHandle n;

    std::string out_topic_prefix;
    ros::param::param<std::string>("~outTopicPrefix", out_topic_prefix, "out/");
    
    mru_position_pub = n.advertise<sensor_msgs::NavSatFix>(out_topic_prefix+"position", 1);
    mru_orientation_pub = n.advertise<sensor_msgs::Imu>(out_topic_prefix+"orientation",1);
    mru_velocity_pub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>(out_topic_prefix+"velocity",1);
    
    bool twist_has_covariance = false;
    ros::param::param<bool>("~twistHasCovariance", twist_has_covariance, false);
    
    if(twist_has_covariance)
      run<geometry_msgs::TwistWithCovarianceStamped>(n);
    else
      run<geometry_msgs::TwistStamped>(n);
}
