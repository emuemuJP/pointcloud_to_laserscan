/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Eurotec, Netherlands
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Rein Appeldoorn
 */

#include <pointcloud_to_laserscan/laserscan_to_pointcloud_nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <math.h>  /* M_PI */

namespace pointcloud_to_laserscan
{
LaserScanToPointCloudNodelet::LaserScanToPointCloudNodelet()
{
}

void LaserScanToPointCloudNodelet::onInit()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  private_nh_ = getPrivateNodeHandle();

  private_nh_.param<std::string>("target_frame", target_frame_, "");
  private_nh_.param<double>("transform_tolerance", transform_tolerance_, 0.01);
  private_nh_.param<int>("points_num", points_num_, 1000);
  private_nh_.param<double>("front_fov", front_fov_, 90);
  private_nh_.param<double>("min_width", min_width_, std::numeric_limits<double>::min());
  private_nh_.param<double>("max_width", max_width_, std::numeric_limits<double>::max());
  private_nh_.param<double>("min_length", min_length_, std::numeric_limits<double>::min());
  private_nh_.param<double>("max_length", max_length_, std::numeric_limits<double>::max());
  private_nh_.param<std::string>("prefix", prefix_, "");

  int concurrency_level = private_nh_.param("concurrency_level", concurrency_level);

  // Check if explicitly single threaded, otherwise, let nodelet manager dictate thread pool size
  if (concurrency_level == 1)
  {
    nh_ = getNodeHandle();
  }
  else
  {
    nh_ = getMTNodeHandle();
  }

  // Only queue one pointcloud per running thread
  if (concurrency_level > 0)
  {
    input_queue_size_ = static_cast<unsigned int>(concurrency_level);
  }
  else
  {
    input_queue_size_ = boost::thread::hardware_concurrency();
  }
  sub_.subscribe(nh_, "scan_in", input_queue_size_);
  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty())
  {
    tf2_.reset(new tf2_ros::Buffer());
    tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
    message_filter_.reset(new MessageFilter(sub_, *tf2_, target_frame_, input_queue_size_, nh_));
    message_filter_->registerCallback(boost::bind(&LaserScanToPointCloudNodelet::scanCallback, this, _1));
    message_filter_->registerFailureCallback(boost::bind(&LaserScanToPointCloudNodelet::failureCallback, this, _1, _2));
  }
  else  // otherwise setup direct subscription
  {
    sub_.registerCallback(boost::bind(&LaserScanToPointCloudNodelet::scanCallback, this, _1));
  }

  ros::SubscriberStatusCallback status = boost::bind(&LaserScanToPointCloudNodelet::connectCb, this);

  pub_ =  nh_.advertise<sensor_msgs::PointCloud2>(prefix_ + "cloud", 10, status, status);
  pub_pc_front_ = nh_.advertise<sensor_msgs::PointCloud>(prefix_ + "cloud_front", 5, status, status);
  pub_pc_lateral_ = nh_.advertise<sensor_msgs::PointCloud>(prefix_ + "cloud_lateral", 5, status, status);
  pub_front_distance_ = nh_.advertise<std_msgs::Float64>(prefix_ + "front_distance", 5, status, status);
  pub_right_distance_ = nh_.advertise<std_msgs::Float64>(prefix_ + "right_distance", 5, status, status);
  pub_left_distance_ = nh_.advertise<std_msgs::Float64>(prefix_ + "left_distance", 5, status, status);
}

void LaserScanToPointCloudNodelet::connectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() > 0 && sub_.getSubscriber().getNumPublishers() == 0)
  {
    NODELET_INFO("Got a subscriber to cloud, starting laserscan subscriber");
  }
}

void LaserScanToPointCloudNodelet::disconnectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() == 0)
  {
    NODELET_INFO("No subscibers to cloud, shutting down subscriber to laserscan");
    sub_.unsubscribe();
  }
}

void LaserScanToPointCloudNodelet::failureCallback(const sensor_msgs::LaserScanConstPtr& scan_msg,
                                                   tf2_ros::filter_failure_reasons::FilterFailureReason reason)
{
  NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform laserscan from frame " << scan_msg->header.frame_id << " to "
                                                                            << message_filter_->getTargetFramesString()
                                                                            << " at time " << scan_msg->header.stamp
                                                                            << ", reason: " << reason);
}

void LaserScanToPointCloudNodelet::scanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  sensor_msgs::PointCloud2Ptr scan_cloud;
  scan_cloud.reset(new sensor_msgs::PointCloud2);
  projector_.projectLaser(*scan_msg, *scan_cloud);
  sensor_msgs::PointCloud cloud_front;
  sensor_msgs::PointCloud cloud_lateral;
  std::priority_queue<float> front_points;
  std::priority_queue<float, std::vector<float>, std::greater<float>> left_points;
  std::priority_queue<float> right_points;
  std_msgs::Float64 front_distance;
  std_msgs::Float64 right_distance;
  std_msgs::Float64 left_distance;

  cloud_front.header.frame_id = scan_cloud->header.frame_id;
  cloud_front.header.stamp = scan_cloud->header.stamp;
  cloud_lateral.header.frame_id = scan_cloud->header.frame_id;
  cloud_lateral.header.stamp = scan_cloud->header.stamp;
  front_distance.data = 1e18;
  right_distance.data = 1e18;
  left_distance.data = -1e18;

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*scan_cloud, "x"), iter_y(*scan_cloud, "y"),
        iter_z(*scan_cloud, "z");
        iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
    {
      NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
      continue;
    }

    geometry_msgs::Point32 point;
    point.x = *iter_x;
    point.y = *iter_y;
    point.z = *iter_z;
    double angle = atan2(*iter_y, *iter_x)*180 / M_PI;
    if (*iter_y <= max_width_ && *iter_y >= min_width_ && (-front_fov_ /2 <= angle || angle <= front_fov_ / 2 ))
    {
      cloud_front.points.push_back(point);
      if (point.x >= 0)
      {
        front_points.push(point.x);
        if(front_points.size() > points_num_) front_points.pop();
      }
    }

    if (*iter_x <= max_length_ && *iter_x >= min_length_)
    {
      cloud_lateral.points.push_back(point);
      if (-point.y >= 0)
      {
        right_points.push(-point.y);
        if(right_points.size() > points_num_) right_points.pop();
      }
      if (-point.y <= 0)
      {
        left_points.push(-point.y);
        if(left_points.size() > points_num_) left_points.pop();
      }
    }
  }

  pub_pc_front_.publish(cloud_front);
  pub_pc_lateral_.publish(cloud_lateral);
  pub_.publish(*scan_cloud);

  float mean_front_distance = 0;
  int front_size = front_points.size();
  while(!front_points.empty())
  {
    mean_front_distance += front_points.top();
    front_points.pop();
  }
  if(front_size!=0) front_distance.data = mean_front_distance / front_size;


  float mean_right_distance = 0;
  int right_size = right_points.size();
  while(!right_points.empty())
  {
    mean_right_distance += right_points.top();
    right_points.pop();
  }
  if(right_size!=0) right_distance.data = mean_right_distance / right_size;

  float mean_left_distance = 0;
  int left_size = left_points.size();
  while(!left_points.empty())
  {
    mean_left_distance += left_points.top();
    left_points.pop();
  }
  if(left_size!=0) left_distance.data = mean_left_distance / left_size;

  if(front_distance.data!=1e18) pub_front_distance_.publish(front_distance);
  if(right_distance.data!=1e18) pub_right_distance_.publish(right_distance);
  if(left_distance.data!=-1e18) pub_left_distance_.publish(left_distance);
}
}  // namespace pointcloud_to_laserscan

PLUGINLIB_EXPORT_CLASS(pointcloud_to_laserscan::LaserScanToPointCloudNodelet, nodelet::Nodelet)
