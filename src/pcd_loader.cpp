#include "map_edit/pcd_loader.h"

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace map_edit
{

  PcdLoader::PcdLoader(const std::string &frame_id)
      : frame_id_(frame_id)
  {
    node_ = std::make_shared<rclcpp::Node>("map_edit_pcd_loader");
    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("map_edit_pcd", qos);
  }

  bool PcdLoader::load(const std::string &path)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    int ret = pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud);
    if (ret != 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "无法加载 PCD 文件: %s", path.c_str());
      return false;
    }

    if (cloud->empty())
    {
      RCLCPP_WARN(node_->get_logger(), "PCD 文件为空: %s", path.c_str());
      return false;
    }

    cloud_ = cloud;
    file_path_ = path;
    computeZRange();

    // 初始范围：全量点云
    z_min_ = cloud_z_min_;
    z_max_ = cloud_z_max_;

    RCLCPP_INFO(node_->get_logger(), "PCD 加载成功: %s, %zu 点, Z 范围 [%.3f, %.3f]",
                path.c_str(), cloud_->size(), cloud_z_min_, cloud_z_max_);
    publishFiltered();
    return true;
  }

  void PcdLoader::computeZRange()
  {
    if (!cloud_ || cloud_->empty())
      return;
    cloud_z_min_ = std::numeric_limits<float>::max();
    cloud_z_max_ = std::numeric_limits<float>::lowest();
    for (const auto &pt : *cloud_)
    {
      if (!std::isfinite(pt.z))
        continue;
      cloud_z_min_ = std::min(cloud_z_min_, pt.z);
      cloud_z_max_ = std::max(cloud_z_max_, pt.z);
    }
    if (cloud_z_min_ > cloud_z_max_)
    {
      cloud_z_min_ = 0.0f;
      cloud_z_max_ = 0.0f;
    }
  }

  void PcdLoader::setFrameId(const std::string &frame_id)
  {
    if (frame_id_ == frame_id)
      return;
    frame_id_ = frame_id;
    if (isLoaded())
    {
      publishFiltered();
    }
  }

  void PcdLoader::setZMin(float z)
  {
    z_min_ = z;
    publishFiltered();
  }

  void PcdLoader::setZMax(float z)
  {
    z_max_ = z;
    publishFiltered();
  }

  void PcdLoader::jetColor(float t, uint8_t &r, uint8_t &g, uint8_t &b)
  {
    t = std::max(0.0f, std::min(1.0f, t));
    float rv = 0.0f, gv = 0.0f, bv = 0.0f;
    if (t < 0.125f)
    {
      bv = 0.5f + 0.5f * (t / 0.125f);
    }
    else if (t < 0.375f)
    {
      bv = 1.0f;
      gv = (t - 0.125f) / 0.25f;
    }
    else if (t < 0.625f)
    {
      gv = 1.0f;
      rv = (t - 0.375f) / 0.25f;
    }
    else if (t < 0.875f)
    {
      rv = 1.0f;
      gv = 1.0f - (t - 0.625f) / 0.25f;
    }
    else
    {
      rv = 1.0f;
      bv = 1.0f - (t - 0.875f) / 0.125f;
    }
    r = static_cast<uint8_t>(rv * 255.0f);
    g = static_cast<uint8_t>(gv * 255.0f);
    b = static_cast<uint8_t>(bv * 255.0f);
  }

  void PcdLoader::publishFiltered()
  {
    if (!isLoaded())
      return;

    pcl::PointCloud<pcl::PointXYZRGB> filtered;
    filtered.reserve(cloud_->size());

    // 按当前过滤范围归一化，保证过滤后颜色对比明显
    float z_span = z_max_ - z_min_;
    bool has_span = z_span > 1e-6f;

    for (const auto &pt : *cloud_)
    {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
        continue;
      if (pt.z < z_min_ || pt.z > z_max_)
        continue;

      pcl::PointXYZRGB out;
      out.x = pt.x;
      out.y = pt.y;
      out.z = pt.z;
      float t = has_span ? (pt.z - z_min_) / z_span : 0.5f;
      uint8_t r, g, b;
      jetColor(t, r, g, b);
      out.r = r;
      out.g = g;
      out.b = b;
      filtered.push_back(out);
    }

    last_filtered_count_ = filtered.size();

    sensor_msgs::msg::PointCloud2 ros_pc2;
    pcl::toROSMsg(filtered, ros_pc2);
    ros_pc2.header.frame_id = frame_id_;
    ros_pc2.header.stamp = node_->now();
    pub_->publish(ros_pc2);

    RCLCPP_DEBUG(node_->get_logger(), "发布过滤点云: %zu/%zu 点, Z [%.3f, %.3f]",
                 filtered.size(), cloud_->size(), z_min_, z_max_);
  }

} // namespace map_edit
