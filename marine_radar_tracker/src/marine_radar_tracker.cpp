#pragma once
#include <rclcpp/rclcpp.hpp>
#include <marine_sensor_msgs/msg/radar_sector.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <marine_interfaces/msg/detect.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <tf2/utils.h>
#include <future>

using std::placeholders::_1;

namespace marine_radar_tracker
{

class MarineRadarTracker : public rclcpp::Node
{
public:
  MarineRadarTracker() 
  : Node("marine_radar_tracker"), 
    grid_map_({"intensity", "latest", "latest_age", "previous", "previous_age"})
  {

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    minimum_range_ = declare_parameter<double>("minimum_range", minimum_range_);
    grid_resolution_factor_ = declare_parameter<double>("grid_resolution_factor", grid_resolution_factor_);
    grid_length_factor_ = declare_parameter<double>("grid_length_factor", grid_length_factor_);
    
    auto publish_interval = declare_parameter<double>("publish_interval", publish_interval_.seconds());
    publish_interval_ = rclcpp::Duration::from_seconds(publish_interval);


    grid_map_.setFrameId(map_frame_);

    radar_subscriber_ = create_subscription<marine_sensor_msgs::msg::RadarSector>(
          "radar_data",
           100,
            std::bind(&MarineRadarTracker::radarSectorCallback, this, _1)
    );
    
    grid_map_publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>("grid_map", rclcpp::QoS(1).transient_local());
  }

private:


  void radarSectorCallback(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg)
  {
    if(last_time_.nanoseconds() == 0 || rclcpp::Time(msg->header.stamp) < last_time_)
    {
      grid_map_.clearAll();
      last_target_scan_time_ = rclcpp::Time();
      last_publish_time_ = rclcpp::Time();
    }

    if(msg->range_max != last_range_)
    {
      float grid_size = msg->range_max * 2.0 * grid_length_factor_;
      float resolution = grid_resolution_factor_* (msg->range_max - msg->range_min) / 
                                                   float(msg->intensities.front().echoes.size());
      grid_map_.setGeometry(grid_map::Length(grid_size, grid_size), resolution);
      last_range_ = msg->range_max;
    }

    geometry_msgs::msg::TransformStamped transform;
    try{
      transform = tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
      return;
    }

    geometry_msgs::msg::PoseStamped p;
    p.header = msg->header;
    p.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped radar_in_map_frame;
    tf2::doTransform(p, radar_in_map_frame, transform);

    auto yaw = tf2::getYaw(radar_in_map_frame.pose.orientation);

    grid_map::Position grid_center(radar_in_map_frame.pose.position.x, radar_in_map_frame.pose.position.y);
    grid_map_.move(grid_center);

    if (last_time_.nanoseconds() == 0)
    {
      last_time_ = rclcpp::Time(msg->header.stamp);
      return;
    }
    double dt = std::max(0.0, (rclcpp::Time(msg->header.stamp) - last_time_).seconds());

    for(grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it)
    {
      if(!isnan(grid_map_.at("latest_age",*it)))
      {
        double new_age = grid_map_.at("latest_age",*it) + dt;
        if(new_age > 5.0)
        {
          grid_map_.at("latest_age",*it) = std::nan("");
          grid_map_.at("latest",*it) = std::nan("");
        }
        else
          grid_map_.at("latest_age",*it) = new_age;
      }
      if(!isnan(grid_map_.at("previous_age",*it)))
      {
        double new_age = grid_map_.at("previous_age",*it) + dt;
        if(new_age > 5.0)
        {
          grid_map_.at("previous_age",*it) = std::nan("");
          grid_map_.at("previous",*it) = std::nan("");
        }
        else
          grid_map_.at("previous_age",*it) = new_age;
      }

      grid_map::Position position;
      grid_map_.getPosition(*it, position);
      double dx = position.x() - radar_in_map_frame.pose.position.x;
      double dy = position.y() - radar_in_map_frame.pose.position.y;
      double r = sqrt(dx*dx+dy*dy);
      if(r >= minimum_range_ && r <= msg->range_max && r >= msg->range_min)
      {
        double theta = atan2(dy, dx);
        if (theta < 0.0)
          theta += 2.0*M_PI;
        theta -= yaw;
        if(theta < 0.0)
          theta += 2.0*M_PI;
        if (theta > 2.0*M_PI)
          theta -= 2.0*M_PI;
        int i = (theta-(msg->angle_start-msg->angle_increment/2.0))/msg->angle_increment;
        if(i >= 0 && i < msg->intensities.size())
        {
          int j = (msg->intensities[i].echoes.size()-1)*(r-msg->range_min)/(msg->range_max-msg->range_min);
          float intensity = msg->intensities[i].echoes[j];
          if(!isnan(grid_map_.at("latest_age", *it)) && grid_map_.at("latest_age", *it) > 0.1)
          {
            grid_map_.at("previous", *it) = grid_map_.at("latest", *it);
            grid_map_.at("previous_age", *it) = grid_map_.at("latest_age", *it);
          }
          grid_map_.at("latest", *it) = intensity;
          grid_map_.at("latest_age", *it) = 0.0;
        }
      }
      if(isnan(grid_map_.at("latest", *it)))
        grid_map_.at("intensity", *it) = std::nan("");
      else
        if(isnan(grid_map_.at("previous", *it)))
          grid_map_.at("intensity", *it) = grid_map_.at("latest", *it)*.5;
        else
          grid_map_.at("intensity", *it) = 0.5*(grid_map_.at("latest", *it)+ grid_map_.at("previous", *it)); //std::min(grid_map_.at("latest", *it), grid_map_.at("previous", *it));
    }

    auto timestamp = rclcpp::Time(msg->header.stamp);
    grid_map_.setTimestamp(timestamp.nanoseconds());
    if(last_publish_time_.nanoseconds() == 0 ||  timestamp >= last_publish_time_+publish_interval_)
    {
      auto message = grid_map::GridMapRosConverter::toMessage(grid_map_, {"intensity"});
      grid_map_publisher_->publish(*message);
      last_publish_time_ = msg->header.stamp;
    }

    last_time_ = msg->header.stamp;

  }

  rclcpp::Subscription<marine_sensor_msgs::msg::RadarSector>::SharedPtr radar_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_publisher_;
  rclcpp::Publisher<marine_interfaces::msg::Detect>::SharedPtr detects_publisher_;

  float minimum_range_ = 0.0;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string map_frame_;

  grid_map::GridMap grid_map_;
  float grid_resolution_factor_ = 6.5;
  float grid_length_factor_ = 0.6;

  rclcpp::Time last_time_;
  double last_range_ = 0.0;

  rclcpp::Time last_target_scan_time_;
  std::future<bool> scan_done_ = std::future<bool>();

  rclcpp::Time last_publish_time_;
  rclcpp::Duration publish_interval_ = rclcpp::Duration::from_seconds(0.2);

};

} // namespace marine_radar_tracker

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<marine_radar_tracker::MarineRadarTracker>();

  //marine_radar_tracker::MarineRadarTracker mrt;
    
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}    
