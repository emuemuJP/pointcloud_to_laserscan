#ifndef DETECT_AREA_DISTANCE_POINTCLOUD_NODELET_H
#define DETECT_AREA_DISTANCE_POINTCLOUD_NODELET_H

#include <boost/thread/mutex.hpp>
#include <message_filters/subscriber.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

namespace detect_area_distance
{
typedef tf2_ros::MessageFilter<sensor_msgs::PointCloud2> MessageFilter;
/**
* Class to process incoming pointclouds into laserscans. Some initial code was pulled from the defunct turtlebot
* pointcloud_to_laserscan implementation.
*/
class PointCloudNodelet : public nodelet::Nodelet
{
public:
  PointCloudNodelet();

private:
  virtual void onInit();

  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void failureCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                 tf2_ros::filter_failure_reasons::FilterFailureReason reason);

  void connectCb();

  void disconnectCb();

  ros::NodeHandle nh_, private_nh_;
  ros::Publisher pub_, pub_pc_, pub_pc_front_, pub_pc_frontleft_, pub_pc_frontright_, pub_pc_lateral_, pub_pc_floor_, pub_front_distance_, pub_frontleft_distance_, pub_frontright_distance_, pub_right_distance_, pub_left_distance_;
  boost::mutex connect_mutex_;

  boost::shared_ptr<tf2_ros::Buffer> tf2_;
  boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_;
  boost::shared_ptr<MessageFilter> message_filter_;

  // ROS Parameters
  unsigned int input_queue_size_;
  std::string target_frame_, prefix_, front_axis_;
  double tolerance_;
  int points_num_;
  double front_fov_, min_height_, max_height_, min_length_, max_length_, min_width_, max_width_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_, range_max_;
  bool use_inf_;
  double inf_epsilon_;
};

}  // namespace pointcloud_to_laserscan

#endif  // POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET_H
