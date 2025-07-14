#include "map_edit/map_eraser_tool.h"
#include "map_edit/tool_manager.h"
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/geometry.hpp>
#include <rviz_rendering/render_window.hpp>
#include <rviz_rendering/geometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rviz_common/render_panel.hpp>
#include <opencv2/opencv.hpp>
#include <OgrePlane.h>
#include <OgreVector3.h>
#include <cmath>

namespace map_edit
{

  MapEraserTool::MapEraserTool()
      : map_received_(false), mouse_pressed_(false), brush_size_set(3), max_brush_size_(10), min_brush_size_(1), brush_mode_(ERASE_TO_FREE), visual_line_visible_(false)
  {

    // 注册到工具管理器
    ToolManager::getInstance().registerMapEraserTool(this);
    projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
    nh = std::make_shared<rclcpp::Node>("maperaser");
  }

  MapEraserTool::~MapEraserTool()
  {
    stop_spin_ = true;
    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }
    if (executor_)
    {
      executor_->remove_node(nh);
    }
  }

  void MapEraserTool::onInitialize()
  {
    // Initialize ROS2
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(nh);
    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    map_sub_ = nh->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", qos, std::bind(&MapEraserTool::mapCallback, this, std::placeholders::_1));
    map_pub_ = nh->create_publisher<nav_msgs::msg::OccupancyGrid>("map_edit", qos);
    marker_pub_ = nh->create_publisher<visualization_msgs::msg::Marker>("visiual_marker", qos);

    stop_spin_ = false;
    spin_thread_ = std::thread([this]()
                               {
  rclcpp::WallRate rate(30);
  while (!stop_spin_) {
    executor_->spin_some();
    rate.sleep();
  } });
    brush_size_property_ = new rviz_common::properties::FloatProperty("Brush Size", brush_size_set,
                                                                      "Size of the square eraser brush (NxN pixels)",
                                                                      getPropertyContainer(), SLOT(brush_updateProperties()), this);
    brush_size_property_->setMin(max_brush_size_);
    brush_size_property_->setMax(min_brush_size_);
  }

  void MapEraserTool::activate()
  {
    setStatus("黑白橡皮擦 - 左键画黑色(障碍物), 右键画白色(自由空间), 拖拽连续画");
    brush_updateProperties();
  }

  void MapEraserTool::deactivate()
  {
    mouse_pressed_ = false;
  }

  int MapEraserTool::processMouseEvent(rviz_common::ViewportMouseEvent &event)
  {
    if (!map_received_)
    {
      setStatus("等待地图数据...");
      return Render;
    }
    // 左键按下
    if (event.leftDown())
    {
      mouse_pressed_ = true;
      brush_mode_ = ERASE_TO_OCCUPIED; // 左键画黑色

      Ogre::Vector3 intersection;
      Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
      auto projection_result = projection_finder_->getViewportPointProjectionOnXYPlane(
          event.panel->getRenderWindow(), event.x, event.y);

      if (projection_result.first)
      {
        const Ogre::Vector3 &intersection = projection_result.second;
        geometry_msgs::msg::Point point;
        point.x = intersection.x;
        point.y = intersection.y;
        point.z = 0.0;
        // 画直线
        if (event.modifiers & Qt::ShiftModifier && last_point_.has_value() && last_point_.value().second == ERASE_TO_OCCUPIED)
        {
          drawLine(last_point_.value().first, point, ERASE_TO_OCCUPIED);
          deleteLine();
        }
        // 画点
        else
        {
          eraseAtPoint(point, ERASE_TO_OCCUPIED);
        }
        last_point_ = std::make_pair(point, brush_mode_);
      }
    }
    // 右键按下
    else if (event.rightDown())
    {
      mouse_pressed_ = true;
      brush_mode_ = ERASE_TO_FREE; // 右键画白色
      Ogre::Vector3 intersection;
      Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);

      auto projection_result = projection_finder_->getViewportPointProjectionOnXYPlane(
          event.panel->getRenderWindow(), event.x, event.y);

      if (projection_result.first)
      {
        const Ogre::Vector3 &intersection = projection_result.second;
        geometry_msgs::msg::Point point;
        point.x = intersection.x;
        point.y = intersection.y;
        point.z = 0.0;
        // 画直线
        if (event.modifiers & Qt::ShiftModifier && last_point_.has_value() && last_point_.value().second == ERASE_TO_FREE)
        {

          drawLine(last_point_.value().first, point, ERASE_TO_FREE);
          deleteLine();
        }
        // 画点
        else
        {
          eraseAtPoint(point, ERASE_TO_FREE);
        }
        last_point_ = std::make_pair(point, brush_mode_);
      }
    }
    // 鼠标松开
    else if (event.leftUp() || event.rightUp())
    {
      mouse_pressed_ = false;
    }
    // 鼠标移动
    else if (event.type == QEvent::MouseMove && mouse_pressed_)
    {
      Ogre::Vector3 intersection;
      Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);

      auto projection_result = projection_finder_->getViewportPointProjectionOnXYPlane(
          event.panel->getRenderWindow(), event.x, event.y);

      if (projection_result.first)
      {
        const Ogre::Vector3 &intersection = projection_result.second;
        geometry_msgs::msg::Point point;
        point.x = intersection.x;
        point.y = intersection.y;
        point.z = 0.0;
        last_point_ = std::make_pair(point, brush_mode_);
        eraseAtPoint(point, brush_mode_);
      }
    }
    // 直线指示线，按住 shift，且有上一个点
    else if (event.modifiers & Qt::ShiftModifier && last_point_.has_value())
    {
      auto projection_result = projection_finder_->getViewportPointProjectionOnXYPlane(
          event.panel->getRenderWindow(), event.x, event.y);

      if (projection_result.first)
      {
        const Ogre::Vector3 &intersection = projection_result.second;
        geometry_msgs::msg::Point point;
        point.x = intersection.x;
        point.y = intersection.y;
        point.z = 0.0;

        visualization_msgs::msg::Marker line;
        line.header.frame_id = "map";
        line.header.stamp = nh->now();
        line.ns = "single_line";
        line.id = 0; // 每次覆盖上一个
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;

        line.pose.orientation.w = 1.0;
        line.scale.x = 0.1; // 线宽
        line.color.r = 1.0;
        line.color.g = 0.0;
        line.color.b = 0.0;
        line.color.a = 1.0;

        // 设置线的起点和终点
        line.points.push_back(last_point_.value().first);
        line.points.push_back(point);
        visual_line_visible_ = true;
        marker_pub_->publish(line);
      }
    }
    else if (!(event.modifiers & Qt::ShiftModifier) && last_point_.has_value() && visual_line_visible_)
    {
      deleteLine();
    }

    return Render;
  }

  void MapEraserTool::brush_updateProperties()
  {
    brush_size_property_->setFloat(static_cast<float>(brush_size_set));
    QString status_msg = "黑白橡皮擦 - 笔刷: " + QString::number(brush_size_set) + "x" + QString::number(brush_size_set) + "像素, 左键:黑色 右键:白色";
    setStatus(status_msg);
  }

  void MapEraserTool::mapCallback(const std::shared_ptr<const nav_msgs::msg::OccupancyGrid> &msg)
  {
    if (!map_received_)
    {
      current_map_ = *msg;
    }
    map_received_ = true;
    publishModifiedMap();
  }

  void MapEraserTool::drawLine(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2, BrushMode mode)
  {
    // 计算插值步数
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double distance = std::hypot(dx, dy);

    const double resolution = current_map_.info.resolution;
    const double step = resolution * 0.5;
    int steps = static_cast<int>(distance / step);

    for (int i = 0; i <= steps; ++i)
    {
      double t = static_cast<double>(i) / steps;
      geometry_msgs::msg::Point p;
      p.x = p1.x + t * dx;
      p.y = p1.y + t * dy;
      p.z = 0.0;
      eraseAtPoint(p, mode);
    }
  }

  void MapEraserTool::deleteLine()
  {
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = nh->now();
    line.ns = "single_line";
    line.id = 0;
    line.action = visualization_msgs::msg::Marker::DELETE;
    marker_pub_->publish(line);
    visual_line_visible_ = false;
  }

  void MapEraserTool::eraseAtPoint(const geometry_msgs::msg::Point &point, BrushMode mode)
  {
    if (!map_received_)
      return;

    // Convert world coordinates to map coordinates
    int map_x = static_cast<int>((point.x - current_map_.info.origin.position.x) / current_map_.info.resolution);
    int map_y = static_cast<int>((point.y - current_map_.info.origin.position.y) / current_map_.info.resolution);

    // 画笔大小表示正方形的边长（像素）
    int brush_size = static_cast<int>(brush_size_set);

    // 确保画笔大小至少为1
    if (brush_size < 1)
    {
      brush_size = 1;
    }

    // Determine the value to paint
    int8_t paint_value;
    switch (mode)
    {
    case ERASE_TO_FREE:
      paint_value = 0; // Free space
      break;
    case ERASE_TO_OCCUPIED:
      paint_value = 100; // Occupied space
      break;
    case ERASE_TO_UNKNOWN:
    default:
      paint_value = -1; // Unknown space
      break;
    }

    // 计算正方形画笔的范围
    // 对于奇数大小：以点击位置为中心
    // 对于偶数大小：向左上角偏移
    int half_size = brush_size / 2;
    int start_x = map_x - half_size;
    int start_y = map_y - half_size;

    // 如果是奇数大小，保持中心对齐
    if (brush_size % 2 == 1)
    {
      // 奇数：中心对齐，例如3x3时从(x-1,y-1)到(x+1,y+1)
      start_x = map_x - half_size;
      start_y = map_y - half_size;
    }
    else
    {
      // 偶数：向左上角偏移，例如2x2时从(x,y)到(x+1,y+1)
      start_x = map_x;
      start_y = map_y;
    }

    // Apply square brush
    for (int dy = 0; dy < brush_size; ++dy)
    {
      for (int dx = 0; dx < brush_size; ++dx)
      {
        int target_x = start_x + dx;
        int target_y = start_y + dy;

        // Check bounds
        if (target_x >= 0 && target_x < static_cast<int>(current_map_.info.width) &&
            target_y >= 0 && target_y < static_cast<int>(current_map_.info.height))
        {
          int index = target_y * current_map_.info.width + target_x;
          if (index >= 0 && index < static_cast<int>(current_map_.data.size()))
          {
            current_map_.data[index] = paint_value;
          }
        }
      }
    }

    publishModifiedMap();
  }

  void MapEraserTool::publishModifiedMap()
  {
    current_map_.header.stamp = nh->now();
    current_map_.header.frame_id = "map";

    // 发布到编辑地图话题
    map_pub_->publish(current_map_);

    // 记录发布信息 - 更新为像素单位
    int size = brush_size_set;
    setStatus("地图已修改并发布 - 笔刷: " + QString::number(size) + "x" + QString::number(size) + "像素");
  }

  int MapEraserTool::processKeyEvent(QKeyEvent *event, rviz_common::RenderPanel *panel)
  {
    if (event->type() == QEvent::KeyPress)
    {
      if (event->key() == Qt::Key_Up)
      {
        brush_size_set += 1;
        brush_size_set = clamp(brush_size_set, min_brush_size_, max_brush_size_);
        brush_updateProperties();
        return Render;
      }
      else if (event->key() == Qt::Key_Down)
      {
        brush_size_set -= 1;
        brush_size_set = clamp(brush_size_set, min_brush_size_, max_brush_size_);
        brush_updateProperties();
        return Render;
      }
    }
    if (event->type() == QEvent::KeyRelease)
    {
      if (event->key() == Qt::Key_Shift)
      {
        std::cout << "Shift key released" << std::endl;
        visualization_msgs::msg::Marker line;
        line.header.frame_id = "map";
        line.header.stamp = nh->now();
        line.ns = "single_line";
        line.id = 0;
        line.action = visualization_msgs::msg::Marker::DELETE;
        marker_pub_->publish(line);
        return Render;
      }
    }
    return 0; // 不渲染
  }

  int MapEraserTool::clamp(int value, int min, int max)
  {
    return std::max(min, std::min(value, max));
  }

  nav_msgs::msg::OccupancyGrid MapEraserTool::getCurrentMap() const
  {
    return current_map_;
  }

  void MapEraserTool::reloadMap()
  {
    map_received_ = false;
  }

} // end namespace map_edit
#include <pluginlib/class_list_macros.hpp> // NOLINT

PLUGINLIB_EXPORT_CLASS(map_edit::MapEraserTool, rviz_common::Tool)