#ifndef MAP_ERASER_TOOL_H
#define MAP_ERASER_TOOL_H

#include <rviz_common/tool.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <QKeyEvent>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <deque>
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
    int processKeyEvent(QKeyEvent *event, rviz_common::RenderPanel *panel);

    nav_msgs::msg::OccupancyGrid getCurrentMap() const;
    void reloadMap();

  private Q_SLOTS:
    void brush_updateProperties();

  private:
    bool map_received_ = false;
    bool mouse_pressed_;
    int brush_size_set;
    int max_brush_size_;
    int min_brush_size_;
    enum BrushMode
    {
      ERASE_TO_FREE,     // 擦除为自由空间 (白色)
      ERASE_TO_OCCUPIED, // 擦除为占用空间 (黑色)
      ERASE_TO_UNKNOWN   // 擦除为未知空间 (灰色)
    };
    enum DrawType
    {
      Point,
      Line
    };
    struct DrawOperation
    {
      DrawType type;
      geometry_msgs::msg::Point start;
      geometry_msgs::msg::Point end;
      BrushMode brush_mode;
    };
    bool visual_line_visible_;
    const int MAX_SIZE = 50;
    void mapCallback(const std::shared_ptr<const nav_msgs::msg::OccupancyGrid> &msg);
    void eraseAtPoint(const geometry_msgs::msg::Point &point, BrushMode mode);
    void drawLine(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2, BrushMode mode);
    void deleteLine();

    void publishModifiedMap();
    int clamp(int value, int min, int max);
    BrushMode reverseBrushMode(BrushMode mode);

    rclcpp::Node::SharedPtr nh;
    rviz_common::properties::FloatProperty *brush_size_property_;
    rviz_common::properties::FloatProperty *line_width_property_;
    rviz_common::properties::ColorProperty *color_property_;
    std::deque<DrawOperation> with_draw_operations_;
    std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

    std::optional<std::pair<geometry_msgs::msg::Point, BrushMode>> last_point_;

    nav_msgs::msg::OccupancyGrid current_map_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    BrushMode brush_mode_;

    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>
        executor_;
    std::thread spin_thread_;
    bool stop_spin_ = false;
  };

} // end namespace map_edit

#endif // MAP_ERASER_TOOL_H