/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * Author: Paul Bovbel
 */

#include <limits>
#include <pluginlib/class_list_macros.h>
#include <pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <queue>

namespace pointcloud_to_laserscan
{
PointCloudToLaserScanNodelet::PointCloudToLaserScanNodelet()
{
}

void PointCloudToLaserScanNodelet::onInit()
{
  NODELET_INFO("Initializing nodelet.");
  boost::mutex::scoped_lock lock(connect_mutex_);
  private_nh_ = getPrivateNodeHandle();

  private_nh_.param<std::string>("target_frame", target_frame_, "");
  private_nh_.param<double>("transform_tolerance", tolerance_, 0.01);
  private_nh_.param<int>("points_num", points_num_, 1000);
  private_nh_.param<double>("front_fov", front_fov_, 90);
  private_nh_.param<std::string>("front_axis", front_axis_, "x");
  private_nh_.param<double>("min_height", min_height_, std::numeric_limits<double>::min());
  private_nh_.param<double>("max_height", max_height_, std::numeric_limits<double>::max());
  private_nh_.param<double>("min_width", min_width_, std::numeric_limits<double>::min());
  private_nh_.param<double>("max_width", max_width_, std::numeric_limits<double>::max());
  private_nh_.param<double>("min_length", min_length_, std::numeric_limits<double>::min());
  private_nh_.param<double>("max_length", max_length_, std::numeric_limits<double>::max());

  private_nh_.param<double>("angle_min", angle_min_, -M_PI);
  private_nh_.param<double>("angle_max", angle_max_, M_PI);
  private_nh_.param<double>("angle_increment", angle_increment_, M_PI / 180.0);
  private_nh_.param<double>("scan_time", scan_time_, 1.0 / 30.0);
  private_nh_.param<double>("range_min", range_min_, 0.0);
  private_nh_.param<double>("range_max", range_max_, std::numeric_limits<double>::max());
  private_nh_.param<double>("inf_epsilon", inf_epsilon_, 1.0);

  int concurrency_level;
  private_nh_.param<int>("concurrency_level", concurrency_level, 1);
  private_nh_.param<bool>("use_inf", use_inf_, true);

  private_nh_.param<std::string>("prefix", prefix_, "");

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
    input_queue_size_ = concurrency_level;
  }
  else
  {
    input_queue_size_ = boost::thread::hardware_concurrency();
  }
  sub_.subscribe(nh_, "cloud_in", input_queue_size_);

  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty())
  {
    tf2_.reset(new tf2_ros::Buffer());
    tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
    message_filter_.reset(new MessageFilter(sub_, *tf2_, target_frame_, input_queue_size_, nh_));
    message_filter_->registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
    message_filter_->registerFailureCallback(boost::bind(&PointCloudToLaserScanNodelet::failureCb, this, _1, _2));
  }
  else  // otherwise setup direct subscription
  {
    sub_.registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
  }

  ros::SubscriberStatusCallback status = boost::bind(&PointCloudToLaserScanNodelet::connectCb, this);

  // pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10, boost::bind(&PointCloudToLaserScanNodelet::connectCb, this),
  //                                              boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this));
  pub_ = nh_.advertise<sensor_msgs::LaserScan>(prefix_ + "scan", 10, status, status);
  pub_pc_ = nh_.advertise<sensor_msgs::PointCloud>(prefix_ + "cloud", 5, status, status);
  pub_pc_frontleft_ = nh_.advertise<sensor_msgs::PointCloud>(prefix_ + "cloud_frontleft", 5, status, status);
  pub_pc_frontright_ = nh_.advertise<sensor_msgs::PointCloud>(prefix_ + "cloud_frontright", 5, status, status);
  pub_pc_front_ = nh_.advertise<sensor_msgs::PointCloud>(prefix_ + "cloud_front", 5, status, status);
  pub_pc_lateral_ = nh_.advertise<sensor_msgs::PointCloud>(prefix_ + "cloud_lateral", 5, status, status);
  pub_pc_floor_ = nh_.advertise<sensor_msgs::PointCloud>(prefix_ + "cloud_floor", 5, status, status);
  pub_front_distance_ = nh_.advertise<std_msgs::Float64>(prefix_ + "front_distance", 5, status, status);
  pub_frontleft_distance_ = nh_.advertise<std_msgs::Float64>(prefix_ + "frontleft_distance", 5, status, status);
  pub_frontright_distance_ = nh_.advertise<std_msgs::Float64>(prefix_ + "frontright_distance", 5, status, status);
  pub_right_distance_ = nh_.advertise<std_msgs::Float64>(prefix_ + "right_distance", 5, status, status);
  pub_left_distance_ = nh_.advertise<std_msgs::Float64>(prefix_ + "left_distance", 5, status, status);
}

void PointCloudToLaserScanNodelet::connectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() > 0 && sub_.getSubscriber().getNumPublishers() == 0)
  {
    NODELET_INFO("Got a subscriber to scan, starting subscriber to pointcloud");
  }
}

void PointCloudToLaserScanNodelet::disconnectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() == 0)
  {
    NODELET_INFO("No subscibers to scan, shutting down subscriber to pointcloud");
    sub_.unsubscribe();
  }
}

void PointCloudToLaserScanNodelet::failureCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                                             tf2_ros::filter_failure_reasons::FilterFailureReason reason)
{
  NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform pointcloud from frame " << cloud_msg->header.frame_id << " to "
                                                                             << message_filter_->getTargetFramesString()
                                                                             << " at time " << cloud_msg->header.stamp
                                                                             << ", reason: " << reason);
}

void PointCloudToLaserScanNodelet::cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // build laserscan output
  sensor_msgs::LaserScan output;
  std::priority_queue<float> front_points;
  std::priority_queue<float> frontleft_points;
  std::priority_queue<float> frontright_points;
  std::priority_queue<float> left_points;
  std::priority_queue<float> right_points;
  output.header = cloud_msg->header;
  if (!target_frame_.empty())
  {
    output.header.frame_id = target_frame_;
  }

  output.angle_min = angle_min_;
  output.angle_max = angle_max_;
  output.angle_increment = angle_increment_;
  output.time_increment = 0.0;
  output.scan_time = scan_time_;
  output.range_min = range_min_;
  output.range_max = range_max_;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_)
  {
    output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  }
  else
  {
    output.ranges.assign(ranges_size, output.range_max + inf_epsilon_);
  }

  sensor_msgs::PointCloud2ConstPtr cloud_out;
  sensor_msgs::PointCloud2Ptr cloud_tmp;
  sensor_msgs::PointCloud cloud;
  sensor_msgs::PointCloud cloud_front;
  sensor_msgs::PointCloud cloud_frontleft;
  sensor_msgs::PointCloud cloud_frontright;
  sensor_msgs::PointCloud cloud_lateral;
  sensor_msgs::PointCloud cloud_floor;
  float distance;
  std_msgs::Float64 front_distance;
  std_msgs::Float64 frontleft_distance;
  std_msgs::Float64 frontright_distance;
  std_msgs::Float64 right_distance;
  std_msgs::Float64 left_distance;

  cloud.header.frame_id = cloud_msg->header.frame_id;
  cloud.header.stamp = cloud_msg->header.stamp;
  cloud_front.header.frame_id = cloud_msg->header.frame_id;
  cloud_front.header.stamp = cloud_msg->header.stamp;
  cloud_frontleft.header.frame_id = cloud_msg->header.frame_id;
  cloud_frontleft.header.stamp = cloud_msg->header.stamp;
  cloud_frontright.header.frame_id = cloud_msg->header.frame_id;
  cloud_frontright.header.stamp = cloud_msg->header.stamp;
  cloud_lateral.header.frame_id = cloud_msg->header.frame_id;
  cloud_lateral.header.stamp = cloud_msg->header.stamp;
  cloud_floor.header.frame_id = cloud_msg->header.frame_id;
  cloud_floor.header.stamp = cloud_msg->header.stamp;

  // Transform cloud if necessary
  if (!(output.header.frame_id == cloud_msg->header.frame_id))
  {
    try
    {
      tf2_->transform(*cloud_msg, *cloud_tmp, target_frame_, ros::Duration(tolerance_));
      cloud_out = cloud_tmp;
    }
    catch (tf2::TransformException& ex)
    {
      NODELET_ERROR_STREAM("Transform failure: " << ex.what());
      return;
    }
  }
  else
  {
    cloud_out = cloud_msg;
  }
  front_distance.data = 1e18;
  frontleft_distance.data = 1e18;
  frontright_distance.data = 1e18;
  right_distance.data = 1e18;
  left_distance.data = 1e18;

  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"),
       iter_z(*cloud_out, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
    {
      NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
      continue;
    }

    geometry_msgs::Point32 point;
    if(front_axis_ == "x")
    {
      point.x = *iter_x;
      point.y = *iter_y;
      point.z = *iter_z;
    }
    else if(front_axis_ == "z")
    {
      point.x = *iter_z;//length
      point.y = -*iter_x;//height
      point.z = -*iter_y;//width
    }

    cloud.points.push_back(point);

    if (point.z > max_height_ || point.z < min_height_)
    {
      if(point.z < min_height_) cloud_floor.points.push_back(point);
      NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", point.z, min_height_, max_height_);
      continue;
    }

    double angle = atan2(point.y, point.x)*180 / M_PI;

    if (point.y <= max_width_ && point.y >= min_width_ &&  (-front_fov_ /2 <= angle || angle <= front_fov_ / 2 ) )
    {
      cloud_front.points.push_back(point);
      if (point.x >= 0)
      {
        front_points.push(point.x);
        if(front_points.size() > points_num_) front_points.pop();
      }
    }

    distance = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);

    if(- point.y > max_width_ && angle >= -front_fov_/2)
    {
      if (point.x >= max_length_)
      {
        cloud_frontright.points.push_back(point);
        frontright_points.push(distance);
        if(frontright_points.size() > points_num_) frontright_points.pop();
      }
    }

    if(- point.y < min_width_ && angle <= front_fov_/2)
    {
      if (point.x >= max_length_)
      {
        cloud_frontleft.points.push_back(point);
        frontleft_points.push(distance);
        if(frontleft_points.size() > points_num_) frontleft_points.pop();
      }
    }

    if (point.x <= max_length_ && point.x >= min_length_)
    {
      cloud_lateral.points.push_back(point);
      if (-point.y >= 0)
      {
        right_points.push(abs(point.y));
        if(right_points.size() > points_num_) right_points.pop();
      }
      if (-point.y <= 0)
      {
        left_points.push(abs(point.y));
        if(left_points.size() > points_num_) left_points.pop();
      }
    }

    double range = hypot(point.x, point.y);
    if (range < range_min_)
    {
      NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, point.x,
                    point.y, point.z);
      continue;
    }
    if (range > range_max_)
    {
      NODELET_DEBUG("rejected for range %f above maximum value %f. Point: (%f, %f, %f)", range, range_max_, point.x,
                    point.y, point.z);
      continue;
    }

    double angle_ = atan2(point.y, point.x);
    if (angle_ < output.angle_min || angle_ > output.angle_max)
    {
      NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle_, output.angle_min, output.angle_max);
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
    int index = (angle_ - output.angle_min) / output.angle_increment;
    if (range < output.ranges[index])
    {
      output.ranges[index] = range;
    }
  }
  pub_.publish(output);
  pub_pc_.publish(cloud);
  pub_pc_front_.publish(cloud_front);
  pub_pc_frontleft_.publish(cloud_frontleft);
  pub_pc_frontright_.publish(cloud_frontright);
  pub_pc_lateral_.publish(cloud_lateral);
  pub_pc_floor_.publish(cloud_floor);

  float mean_front_distance = 0;
  int front_size = front_points.size();
  while(!front_points.empty())
  {
    mean_front_distance += front_points.top();
    front_points.pop();
  }
  if(front_size!=0) front_distance.data = mean_front_distance / front_size;

  float mean_frontleft_distance = 0;
  int frontleft_size = frontleft_points.size();
  while(!frontleft_points.empty())
  {
    mean_frontleft_distance += frontleft_points.top();
    frontleft_points.pop();
  }
  if(frontleft_size!=0) frontleft_distance.data = mean_frontleft_distance / frontleft_size;

  float mean_frontright_distance = 0;
  int frontright_size = frontright_points.size();
  while(!frontright_points.empty())
  {
    mean_frontright_distance += frontright_points.top();
    frontright_points.pop();
  }
  if(frontright_size!=0) frontright_distance.data = mean_frontright_distance / frontright_size;

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
  if(frontleft_distance.data!=1e18) pub_frontleft_distance_.publish(frontleft_distance);
  if(frontright_distance.data!=1e18) pub_frontright_distance_.publish(frontright_distance);
  if(right_distance.data!=1e18) pub_right_distance_.publish(right_distance);
  if(left_distance.data!=1e18) pub_left_distance_.publish(left_distance);
}
}  // namespace pointcloud_to_laserscan

PLUGINLIB_EXPORT_CLASS(pointcloud_to_laserscan::PointCloudToLaserScanNodelet, nodelet::Nodelet)
