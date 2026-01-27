#include "map_edit/region_tool.h"
#include "map_edit/tool_manager.h"

map_edit::RegionTool *g_region_tool = nullptr;

namespace map_edit
{

  RegionTool::RegionTool()
      : marker_id_counter_(0), region_id_counter_(1), drawing_current_region_(false)
  {
    // 注册到工具管理器
    ToolManager::getInstance().registerRegionTool(this);
    projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
    nh = std::make_shared<rclcpp::Node>("regiontool");
  }

  RegionTool::~RegionTool()
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

  void RegionTool::onInitialize()
  {
    g_region_tool = this;
    // Initialize ROS2
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(nh);
    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    marker_arrary_pub_ = nh->create_publisher<visualization_msgs::msg::MarkerArray>("visual_marker_array", qos);
    marker_pub_ = nh->create_publisher<visualization_msgs::msg::Marker>("marker", qos);
    stop_spin_ = false;
    spin_thread_ = std::thread([this]()
                               {
    rclcpp::WallRate rate(30);
    while (!stop_spin_) {
      executor_->spin_some();
      rate.sleep();
    } });

    // Setup properties
    color_property_ = new rviz_common::properties::ColorProperty("Color", QColor(0, 255, 0),
                                                                 "Color of region polygons",
                                                                 getPropertyContainer(), SLOT(updateProperties()), this);

    alpha_property_ = new rviz_common::properties::FloatProperty("Alpha", 0.5,
                                                                 "Transparency of region polygons",
                                                                 getPropertyContainer(), SLOT(updateProperties()), this);
    alpha_property_->setMin(0.0);
    alpha_property_->setMax(1.0);

    region_type_property_ = new rviz_common::properties::IntProperty("Region Type", 0,
                                                                     "Type identifier for the region",
                                                                     getPropertyContainer(), SLOT(updateProperties()), this);
    region_type_property_->setMin(0);
    region_type_property_->setMax(10);

    region_param_property_ = new rviz_common::properties::FloatProperty("Region Parameter", 1.0,
                                                                        "Parameter value for the region",
                                                                        getPropertyContainer(), SLOT(updateProperties()), this);

    frame_id_property_ = new rviz_common::properties::StringProperty("Frame ID", "map",
                                                                     "Frame ID for the regions",
                                                                     getPropertyContainer(), SLOT(updateProperties()), this);

    show_points_property_ = new rviz_common::properties::BoolProperty("Show Points", true,
                                                                      "Show individual points on regions",
                                                                      getPropertyContainer(), SLOT(updateProperties()), this);
  }

  void RegionTool::activate()
  {
    setStatus("Region Tool - Left click to add polygon points, right click to finish region");
    updateMarkers(regions_);
    emit regiontoolInitialized();
  }

  void RegionTool::deactivate()
  {
    drawing_current_region_ = false;
    current_region_points_.clear();
    emit regiontooldeInitialized();
  }

  int RegionTool::processMouseEvent(rviz_common::ViewportMouseEvent &event)
  {

    // 左键按下
    if (event.leftDown())
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
        // shift
        if (event.modifiers & Qt::ShiftModifier)
        {
          addPoint(point);
          drawing_current_region_ = true;
          deleteLine();
        }
        else
        {
          std::string own_selected_region_id_ = "";
          for (const auto &region : regions_)
          {
            if (isPointInPolygon(region.points, point))
            {
              own_selected_region_id_ = region.id;
              if (selected_region_id_ == own_selected_region_id_ && own_selected_region_id_ != "")
              {
                int idx = findNearestPointIndex(region, point, 0.5);
                if (idx != -1)
                {
                  selected_point_index_ = idx;
                  dragging_point_ = true;
                }
              }
              break;
            }
          }
          selected_region_id_ = own_selected_region_id_;
          updateMarkers(regions_);
        }
      }
    }
    // 右键按下
    else if (event.rightDown() && drawing_current_region_)
    {
      finishCurrentRegion();
      deleteLine();
    }
    // 鼠标移动
    else if (event.type == QEvent::MouseMove)
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
        if (event.modifiers & Qt::ShiftModifier && drawing_current_region_)
        {
          visualization_msgs::msg::Marker line;
          line.header.frame_id = "map";
          line.header.stamp = nh->now();
          line.ns = "single_line";
          line.id = 0;
          line.type = visualization_msgs::msg::Marker::LINE_STRIP;
          line.action = visualization_msgs::msg::Marker::ADD;

          line.pose.orientation.w = 1.0;
          line.scale.x = 0.1;
          line.color.r = 1.0;
          line.color.g = 0.0;
          line.color.b = 0.0;
          line.color.a = 1.0;

          line.points.push_back(current_region_points_.back());
          line.points.push_back(point);
          marker_pub_->publish(line);
        }
        else if (dragging_point_)
        {
          for (auto &region : regions_)
          {
            if (region.id == selected_region_id_ && selected_point_index_ >= 0)
            {
              region.points[selected_point_index_] = point;
              updateMarkers(regions_);
              break;
            }
          }
        }
      }
    }
    // ========== 左键释放 ==========
    else if (event.leftUp())
    {
      dragging_point_ = false;
      selected_point_index_ = -1;
    }
    else
    {
      deleteLine();
    }

    return Render;
  }

  void RegionTool::deleteLine()
  {
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = nh->now();
    line.ns = "single_line";
    line.id = 0;
    line.action = visualization_msgs::msg::Marker::DELETE;
    marker_pub_->publish(line);
    // visual_line_visible_ = false;
  }

  void RegionTool::addPoint(const geometry_msgs::msg::Point &point)
  {
    current_region_points_.push_back(point);
    updateMarkers(regions_);
  }

  void RegionTool::finishCurrentRegion()
  {
    if (current_region_points_.size() >= 3)
    {
      Region region;
      region.points = current_region_points_;
      region.id = generateRegionId();
      region.frame_id = frame_id_property_->getStdString();
      region.type = region_type_property_->getInt();
      region.notes = "";
      regions_.push_back(region);
      emit regionsChanged();
    }

    current_region_points_.clear();
    drawing_current_region_ = false;
    updateMarkers(regions_);
  }

  std::string RegionTool::generateRegionId()
  {
    return "region" + std::to_string(region_id_counter_++);
  }

  void RegionTool::updateMarkers(const std::vector<Region> &private_regions_)
  {
    marker_id_counter_ = 0;

    QColor color = color_property_->getColor();
    float alpha = alpha_property_->getFloat();
    // Draw existing regions
    visualization_msgs::msg::MarkerArray marker_array_;
    marker_array_.markers.clear();
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array_.markers.push_back(clear_marker);
    for (const auto &region : private_regions_)
    {
      if (region.points.size() >= 3)
      {
        if (region.id == selected_region_id_)
        {
          color = QColor(255, 0, 0);
        }
        else
        {
          color = color_property_->getColor();
        }
        // Create filled polygon
        visualization_msgs::msg::Marker polygon_marker;
        polygon_marker.header.frame_id = region.frame_id;
        polygon_marker.header.stamp = nh->now();
        polygon_marker.ns = "regions";
        polygon_marker.id = marker_id_counter_++;
        polygon_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        polygon_marker.action = visualization_msgs::msg::Marker::ADD;
        polygon_marker.pose.orientation.w = 1.0;
        polygon_marker.scale.x = polygon_marker.scale.y = polygon_marker.scale.z = 1.0;
        polygon_marker.color.r = color.redF();
        polygon_marker.color.g = color.greenF();
        polygon_marker.color.b = color.blueF();
        polygon_marker.color.a = alpha;

        // Simple triangulation for convex polygons
        for (size_t i = 1; i < region.points.size() - 1; ++i)
        {
          polygon_marker.points.push_back(region.points[0]);
          polygon_marker.points.push_back(region.points[i]);
          polygon_marker.points.push_back(region.points[i + 1]);
        }

        marker_array_.markers.push_back(polygon_marker);

        // Create outline
        visualization_msgs::msg::Marker outline_marker;
        outline_marker.header.frame_id = region.frame_id;
        outline_marker.header.stamp = nh->now();
        outline_marker.ns = "region_outlines";
        outline_marker.id = marker_id_counter_++;
        outline_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        outline_marker.action = visualization_msgs::msg::Marker::ADD;
        outline_marker.pose.orientation.w = 1.0;
        outline_marker.scale.x = 0.04;
        outline_marker.color.r = color.redF();
        outline_marker.color.g = color.greenF();
        outline_marker.color.b = color.blueF();
        outline_marker.color.a = 1.0;

        for (const auto &point : region.points)
        {
          outline_marker.points.push_back(point);
        }
        // Close the polygon
        outline_marker.points.push_back(region.points[0]);

        marker_array_.markers.push_back(outline_marker);

        // Add region label
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = region.frame_id;
        text_marker.header.stamp = nh->now();
        text_marker.ns = "region_labels";
        text_marker.id = marker_id_counter_++;
        text_marker.scale.z = 1.5;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;

        // Calculate centroid
        geometry_msgs::msg::Point centroid;
        centroid.x = centroid.y = centroid.z = 0.0;
        for (const auto &point : region.points)
        {
          centroid.x += point.x;
          centroid.y += point.y;
        }
        centroid.x /= region.points.size();
        centroid.y /= region.points.size();
        centroid.z = 0.1;

        text_marker.pose.position = centroid;
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 0.3;
        text_marker.color.r = 0.2;
        text_marker.color.g = 0.2;
        text_marker.color.b = 0.2;
        text_marker.color.a = 1.0;
        text_marker.text = region.id + "(" + region.notes + ")";

        marker_array_.markers.push_back(text_marker);
      }
    }

    // Draw current region being created
    if (current_region_points_.size() >= 2)
    {
      visualization_msgs::msg::Marker current_outline;
      current_outline.header.frame_id = frame_id_property_->getStdString();
      current_outline.header.stamp = nh->now();
      current_outline.ns = "current_region";
      current_outline.id = marker_id_counter_++;
      current_outline.type = visualization_msgs::msg::Marker::LINE_STRIP;
      current_outline.action = visualization_msgs::msg::Marker::ADD;
      current_outline.pose.orientation.w = 1.0;
      current_outline.scale.x = 0.1;
      current_outline.color.r = 0.0;
      current_outline.color.g = 0.0;
      current_outline.color.b = 1.0;
      current_outline.color.a = 1.0;

      for (const auto &point : current_region_points_)
      {
        current_outline.points.push_back(point);
      }

      marker_array_.markers.push_back(current_outline);
    }

    marker_arrary_pub_->publish(marker_array_);
  }

  bool RegionTool::isPointInPolygon(const std::vector<geometry_msgs::msg::Point> &polygon,
                                    const geometry_msgs::msg::Point &pt)
  {
    int count = 0;
    size_t n = polygon.size();

    if (n < 3)
      return false; // 少于3个点不能构成多边形

    for (size_t i = 0; i < n; ++i)
    {
      geometry_msgs::msg::Point a = polygon[i];
      geometry_msgs::msg::Point b = polygon[(i + 1) % n];

      // 确保 a 在下方，b 在上方
      if (a.y > b.y)
        std::swap(a, b);

      // 判断水平射线是否穿过这条边
      if (a.y <= pt.y && b.y > pt.y)
      {
        // 计算叉积
        double crossProduct = (pt.x - a.x) * (b.y - a.y) - (b.x - a.x) * (pt.y - a.y);

        // 点在边上
        if (crossProduct == 0.0)
          return true;

        // 计算交点位置，判断射线是否穿过
        if (crossProduct > 0.0)
          count++;
      }
    }

    return (count % 2 == 1);
  }
  int RegionTool::findNearestPointIndex(const Region &region, const geometry_msgs::msg::Point &click, double tol)
  {
    double min_dist = tol;
    int nearest_index = -1;

    for (int i = 0; i < (int)region.points.size(); ++i)
    {
      double dx = region.points[i].x - click.x;
      double dy = region.points[i].y - click.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < min_dist)
      {
        min_dist = dist;
        nearest_index = i;
      }
    }
    return nearest_index;
  }

  void RegionTool::clearRegions()
  {
    regions_.clear();
    current_region_points_.clear();
    drawing_current_region_ = false;
    updateMarkers(regions_);
    setStatus("Cleared all regions");
  }

  std::vector<Region> RegionTool::getRegions() const
  {
    return regions_;
  }

  void RegionTool::setRegions(const std::vector<Region> &regions)
  {
    regions_ = regions;
    current_region_points_.clear();
    drawing_current_region_ = false;
    updateMarkers(regions_);
    setStatus("Set " + QString::number(regions_.size()) + " regions");
  }
} // end namespace map_edit

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(map_edit::RegionTool, rviz_common::Tool)