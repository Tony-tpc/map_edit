#ifndef MAP_ERASER_TOOL_H
#define MAP_ERASER_TOOL_H

#include <rviz_common/tool.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <thread>
namespace map_edit
{

  class MapEraserTool : public rviz_common::Tool
  {
    Q_OBJECT
  public:
    MapEraserTool();
    virtual ~MapEraserTool();

    virtual void onInitialize();
    virtual void activate();
    virtual void deactivate();

    virtual int processMouseEvent(rviz_common::ViewportMouseEvent &event);

    void loadMap(const std::string &filename);
    void saveMap(const std::string &filename);

    nav_msgs::msg::OccupancyGrid getCurrentMap() const;

  private Q_SLOTS:
    void updateProperties();

  private:
    void mapCallback(const std::shared_ptr<const nav_msgs::msg::OccupancyGrid> &msg);
    void eraseAtPoint(const geometry_msgs::msg::Point &point);
    void paintAtPoint(const geometry_msgs::msg::Point &point);
    geometry_msgs::msg::Point screenToMap(int screen_x, int screen_y);
    void publishModifiedMap();
    rclcpp::Node::SharedPtr nh;
    rviz_common::properties::FloatProperty *brush_size_property_;

    // tf2_ros::Buffer tf_buffer_;
    // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

    enum BrushMode
    {
      ERASE_TO_FREE,     // 擦除为自由空间 (白色)
      ERASE_TO_OCCUPIED, // 擦除为占用空间 (黑色)
      ERASE_TO_UNKNOWN   // 擦除为未知空间 (灰色)
    };

    nav_msgs::msg::OccupancyGrid current_map_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    bool map_received_ = false;
    bool mouse_pressed_;
    double brush_size_;
    BrushMode brush_mode_;

    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spin_thread_;
    bool stop_spin_ = false;
  };

} // end namespace map_edit

#endif // MAP_ERASER_TOOL_H