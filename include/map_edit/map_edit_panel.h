#ifndef MAP_EDIT_PANEL_HPP
#define MAP_EDIT_PANEL_HPP
#include <QWidget>
#include <QPainter>
#include <QPainterPath>
#include <QMouseEvent>
#include <QVBoxLayout>
#include <memory>
#include <string>
#include <rviz_common/panel.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QGridLayout>
#include <QFileDialog>
#include <QComboBox>
#include <QGroupBox>
#include <QMessageBox>
#include <QRadioButton>
#include <QButtonGroup>
#include <QTabWidget>
#include <QProgressBar>
#include <QListWidget>
#include <QTableWidget>
#include <QHeaderView>
#include <QCheckBox>
#include <QInputDialog>
#include <QProcess>
#include <QProgressDialog>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <QImage>
#include <QFormLayout>
#include <QSpinBox>
#include <QTextEdit>
#include <QSlider>
#include <yaml-cpp/yaml.h>
#include <QFileInfo>
#include <thread>
#include <chrono>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fstream>
#include "tool_manager.h"
#include "map_eraser_tool.h"
#include "region_tool.h"
#include "pcd_loader.h"
namespace map_edit
{

    class MapEditPanel : public rviz_common::Panel
    {
        Q_OBJECT
    public:
        explicit MapEditPanel(QWidget *parent = nullptr);
        virtual ~MapEditPanel();
        void onInitialize() override;
        void connectTool(RegionTool *tool);

    public Q_SLOTS:
        void onRegionToolInitialized();
        void onRegiontoolDeInitialized();
        void onregionsChanged();

    private Q_SLOTS:
        void openMap();
        void openRegionYaml();
        void clearRegions();
        void saveAllFiles();
        void saveRegionYaml();
        // PCD 点云辅助
        void loadPcdFile();
        void onZMinChanged(int value);
        void onZMaxChanged(int value);

    private:
        void setupUI();
        void setupLocalTab();
        void setupRegionUI();
        void setupPcdUI();
        void refreshRegionListUI();
        void updatePcdInfoLabel();
        bool isTopicExist(const std::string &topic_name);
        void loadAndPublishMap(const std::string &filename);
        void publishMap(const nav_msgs::msg::OccupancyGrid &map);
        bool saveMapFiles(const std::string &fileName, const nav_msgs::msg::OccupancyGrid &map);
        bool saveMapYaml(const std::string &yaml_file, const std::string &pgm_filename, const nav_msgs::msg::OccupancyGrid &map);
        // ros2
        std::shared_ptr<rclcpp::Node> MapEditNode;

        // UI Components
        QVBoxLayout *main_layout_;

        // 一键保存组
        QGroupBox *save_group_;
        QPushButton *save_all_btn_;
        QPushButton *save_yaml_btn_;

        QLabel *current_map_label_;

        // 地图打开选择
        QTabWidget *map_source_tabs_;
        QProgressDialog *progress_dialog_;

        // 本地地图选项卡
        QWidget *local_tab_;
        QPushButton *open_local_btn_;

        // 区域管理组
        QGroupBox *region_group;
        QPushButton *open_region_yaml_btn_{nullptr};
        QPushButton *clear_region_btn_{nullptr};
        QScrollArea *region_scroll_area_;
        QWidget *region_container_;
        QVBoxLayout *region_list_layout_;

        // 点云辅助组 (PCD)
        QGroupBox *pcd_group_;
        QPushButton *load_pcd_btn_;
        QLabel *pcd_info_label_;
        QSlider *z_min_slider_;
        QSlider *z_max_slider_;
        QLabel *z_min_caption_;
        QLabel *z_max_caption_;
        QLabel *z_min_value_label_;
        QLabel *z_max_value_label_;
        std::unique_ptr<PcdLoader> pcd_loader_;

        // 状态显示
        QLabel *status_label_;
        QLabel *info_label_;
        QLabel *local_info;

        // 当前地图文件路径和状态
        QString current_map_file_;
        QString last_opened_path_;
        bool is_saved = false;

        // 地图工具
        ToolManager &toolManager = ToolManager::getInstance();
        MapEraserTool *eraserTool = toolManager.getMapEraserTool();

        // Region
        std::vector<Region> regions_;

    protected:
        void resizeEvent(QResizeEvent *event) override;
    };

} // namespace map_edit

#endif // TF_SHIFT_PANEL_HPP
