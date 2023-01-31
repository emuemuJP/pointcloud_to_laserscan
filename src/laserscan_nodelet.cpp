#include <detect_area_distance/laserscan_nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <math.h>  /* M_PI */

namespace detect_area_distance
{
LaserScanNodelet::LaserScanNodelet()
{
}

void LaserScanNodelet::onInit()
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
    message_filter_->registerCallback(boost::bind(&LaserScanNodelet::scanCallback, this, _1));
    message_filter_->registerFailureCallback(boost::bind(&LaserScanNodelet::failureCallback, this, _1, _2));
  }
  else  // otherwise setup direct subscription
  {
    sub_.registerCallback(boost::bind(&LaserScanNodelet::scanCallback, this, _1));
  }

  ros::SubscriberStatusCallback status = boost::bind(&LaserScanNodelet::connectCb, this);

  pub_ =  nh_.advertise<sensor_msgs::PointCloud2>(prefix_ + "cloud", 10, status, status);
  pub_pc_front_ = nh_.advertise<sensor_msgs::PointCloud>(prefix_ + "cloud_front", 5, status, status);
  pub_pc_frontleft_ = nh_.advertise<sensor_msgs::PointCloud>(prefix_ + "cloud_frontleft", 5, status, status);
  pub_pc_frontright_ = nh_.advertise<sensor_msgs::PointCloud>(prefix_ + "cloud_frontright", 5, status, status);
  pub_pc_lateral_ = nh_.advertise<sensor_msgs::PointCloud>(prefix_ + "cloud_lateral", 5, status, status);
  pub_front_distance_ = nh_.advertise<std_msgs::Float64>(prefix_ + "front_distance", 5, status, status);
  pub_frontleft_distance_ = nh_.advertise<std_msgs::Float64>(prefix_ + "frontleft_distance", 5, status, status);
  pub_frontright_distance_ = nh_.advertise<std_msgs::Float64>(prefix_ + "frontright_distance", 5, status, status);
  pub_right_distance_ = nh_.advertise<std_msgs::Float64>(prefix_ + "right_distance", 5, status, status);
  pub_left_distance_ = nh_.advertise<std_msgs::Float64>(prefix_ + "left_distance", 5, status, status);
}

void LaserScanNodelet::connectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() > 0 && sub_.getSubscriber().getNumPublishers() == 0)
  {
    NODELET_INFO("Got a subscriber to cloud, starting laserscan subscriber");
  }
}

void LaserScanNodelet::disconnectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() == 0)
  {
    NODELET_INFO("No subscibers to cloud, shutting down subscriber to laserscan");
    sub_.unsubscribe();
  }
}

void LaserScanNodelet::failureCallback(const sensor_msgs::LaserScanConstPtr& scan_msg,
                                                   tf2_ros::filter_failure_reasons::FilterFailureReason reason)
{
  NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform laserscan from frame " << scan_msg->header.frame_id << " to "
                                                                            << message_filter_->getTargetFramesString()
                                                                            << " at time " << scan_msg->header.stamp
                                                                            << ", reason: " << reason);
}

void LaserScanNodelet::scanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  sensor_msgs::PointCloud2Ptr scan_cloud;
  scan_cloud.reset(new sensor_msgs::PointCloud2);
  projector_.projectLaser(*scan_msg, *scan_cloud);
  sensor_msgs::PointCloud cloud_front;
  sensor_msgs::PointCloud cloud_frontleft;
  sensor_msgs::PointCloud cloud_frontright;
  sensor_msgs::PointCloud cloud_lateral;
  std::priority_queue<float> front_points;
  std::priority_queue<float> frontleft_points;
  std::priority_queue<float> frontright_points;
  std::priority_queue<float> left_points;
  std::priority_queue<float> right_points;
  float distance;
  std_msgs::Float64 front_distance;
  std_msgs::Float64 frontleft_distance;
  std_msgs::Float64 frontright_distance;
  std_msgs::Float64 right_distance;
  std_msgs::Float64 left_distance;

  cloud_front.header.frame_id = scan_cloud->header.frame_id;
  cloud_front.header.stamp = scan_cloud->header.stamp;
  cloud_frontleft.header.frame_id = scan_cloud->header.frame_id;
  cloud_frontleft.header.stamp = scan_cloud->header.stamp;
  cloud_frontright.header.frame_id = scan_cloud->header.frame_id;
  cloud_frontright.header.stamp = scan_cloud->header.stamp;
  cloud_lateral.header.frame_id = scan_cloud->header.frame_id;
  cloud_lateral.header.stamp = scan_cloud->header.stamp;
  front_distance.data = 1e18;
  frontleft_distance.data = 1e18;
  frontright_distance.data = 1e18;
  right_distance.data = 1e18;
  left_distance.data = 1e18;

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

    distance = sqrt(point.x*point.x + point.y*point.y);// + point.z*point.z);

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
    if (*iter_x <= max_length_ && *iter_x >= min_length_)
    {
      cloud_lateral.points.push_back(point);
      if (-*iter_y >= 0)
      {
        right_points.push(abs(point.y));
        if(right_points.size() > points_num_) right_points.pop();
      }
      if (-*iter_y <= 0)
      {
        left_points.push(abs(point.y));
        if(left_points.size() > points_num_) left_points.pop();
      }
    }
  }

  pub_pc_front_.publish(cloud_front);
  pub_pc_frontleft_.publish(cloud_frontleft);
  pub_pc_frontright_.publish(cloud_frontright);
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
}  // namespace detect_area_distance

PLUGINLIB_EXPORT_CLASS(detect_area_distance::LaserScanNodelet, nodelet::Nodelet)
