#ifndef MAP_EDIT_PCD_LOADER_H
#define MAP_EDIT_PCD_LOADER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace map_edit
{

  /**
   * @brief PCD 点云加载与 Z 高度范围过滤器
   *
   * 负责加载 PCD 文件、按 Z 高度范围过滤点云，并将过滤后的点云
   * 以带颜色（Jet 高度渐变）的 PointCloud2 发布到 /map_edit_pcd，
   * 供 RViz 显示，辅助地图编辑（p 图）。
   */
  class PcdLoader
  {
  public:
    /**
     * @param frame_id 点云发布的坐标系（默认 map，与地图对齐）
     */
    explicit PcdLoader(const std::string &frame_id = "map");
    ~PcdLoader() = default;

    /**
     * @brief 加载 PCD 文件，成功后自动发布全量点云并初始化 Z 范围
     * @param path PCD 文件路径
     * @return 是否加载成功
     */
    bool load(const std::string &path);

    /// 设置 Z 最小过滤值（米），并立即重新发布
    void setZMin(float z);
    /// 设置 Z 最大过滤值（米），并立即重新发布
    void setZMax(float z);

    /// 设置点云发布的坐标系，并重新发布（若已加载）
    void setFrameId(const std::string &frame_id);

    /// 当前过滤 Z 范围
    float getZMin() const { return z_min_; }
    float getZMax() const { return z_max_; }
    /// 点云原始 Z 范围
    float getCloudZMin() const { return cloud_z_min_; }
    float getCloudZMax() const { return cloud_z_max_; }

    size_t pointCount() const { return cloud_ ? cloud_->size() : 0; }
    bool isLoaded() const { return cloud_ != nullptr && !cloud_->empty(); }
    const std::string &filePath() const { return file_path_; }
    size_t filteredPointCount() const { return last_filtered_count_; }

    /// 按当前 Z 范围过滤并发布点云
    void publishFiltered();

  private:
    void computeZRange();
    /// t ∈ [0,1] 映射为 Jet 渐变色（蓝→青→绿→黄→红）
    static void jetColor(float t, uint8_t &r, uint8_t &g, uint8_t &b);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    std::string frame_id_;
    std::string file_path_;

    float cloud_z_min_ = 0.0f;
    float cloud_z_max_ = 0.0f;
    float z_min_ = 0.0f;
    float z_max_ = 0.0f;
    size_t last_filtered_count_ = 0;
  };

} // namespace map_edit

#endif // MAP_EDIT_PCD_LOADER_H
