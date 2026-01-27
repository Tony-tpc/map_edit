#ifndef REGION_TOOL_H
#define REGION_TOOL_H

#include <rviz_common/tool.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include "geometry_msgs/msg/point.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_manager.hpp>
#include <OgrePlane.h>
#include <OgreVector3.h>
#include <fstream>
#include <sstream>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <map>

namespace map_edit
{

  struct Region
  {
    std::string id;
    std::string frame_id;
    int type;
    std::vector<geometry_msgs::msg::Point> points;
    std::string notes;
    std::map<std::string, std::string> attributes;
  };

  class RegionTool : public rviz_common::Tool
  {
    Q_OBJECT
  public:
    RegionTool();
    virtual ~RegionTool();

    virtual void onInitialize();
    virtual void activate();
    virtual void deactivate();

    virtual int processMouseEvent(rviz_common::ViewportMouseEvent &event);

    void clearRegions();
    void setRegions(const std::vector<Region> &regions);

    bool isPointInPolygon(const std::vector<geometry_msgs::msg::Point> &polygon,
                          const geometry_msgs::msg::Point &pt);
    int findNearestPointIndex(const Region &region, const geometry_msgs::msg::Point &click, double tol);

    std::vector<Region> getRegions() const;

  private Q_SLOTS:
    void updateProperties();

  Q_SIGNALS:
    void regiontoolInitialized();
    void regiontooldeInitialized();
    void regionsChanged();

  private:
    void updateMarkers(const std::vector<Region> &regions_);
    void addPoint(const geometry_msgs::msg::Point &point);
    void finishCurrentRegion();
    void deleteLine();

    std::string generateRegionId();

    rclcpp::Node::SharedPtr nh;
    rviz_common::properties::ColorProperty *color_property_;
    rviz_common::properties::FloatProperty *alpha_property_;
    rviz_common::properties::IntProperty *region_type_property_;
    rviz_common::properties::FloatProperty *region_param_property_;
    rviz_common::properties::StringProperty *frame_id_property_;
    rviz_common::properties::BoolProperty *show_points_property_;
    std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

    std::vector<Region> regions_;
    std::string selected_region_id_;
    std::vector<geometry_msgs::msg::Point> current_region_points_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_arrary_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    int marker_id_counter_;
    int region_id_counter_;
    int selected_point_index_;
    bool drawing_current_region_;
    bool dragging_point_;

    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>
        executor_;
    std::thread spin_thread_;
    bool stop_spin_ = false;
  };

} // end namespace map_edit

extern map_edit::RegionTool *g_region_tool;

#endif // REGION_TOOL_H