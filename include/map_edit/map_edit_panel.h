#ifndef MAP_EDIT_PANEL_HPP
#define MAP_EDIT_PANEL_HPP
#include <QWidget>
#include <QPainter>
#include <QPainterPath>
#include <QMouseEvent>
#include <QVBoxLayout>
#include <rviz_common/panel.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QHBoxLayout>
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
#include <QCheckBox>
#include <QInputDialog>
#include <QProcess>
#include <QProgressDialog>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <QImage>
#include <yaml-cpp/yaml.h>
#include <QFileInfo>
#include <thread>
#include <chrono>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tool_manager.h"
#include "map_eraser_tool.h"

namespace map_edit
{

    class MapEditPanel : public rviz_common::Panel
    {
        Q_OBJECT
    public:
        explicit MapEditPanel(QWidget *parent = nullptr);
        virtual ~MapEditPanel();
        void onInitialize() override;

    private Q_SLOTS:
        void openMap();
        void saveAllFiles();

    private:
        void setupUI();
        void setupLocalTab();
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
        QPushButton *save_to_remote_btn_;
        QPushButton *open_map_btn_;
        QLabel *current_map_label_;

        // 地图打开选择
        QTabWidget *map_source_tabs_;
        QProgressDialog *progress_dialog_;

        // 本地地图选项卡
        QWidget *local_tab_;
        QPushButton *open_local_btn_;

        // 状态显示
        QLabel *status_label_;
        QLabel *info_label_;

        // 当前地图文件路径和状态
        QString current_map_file_;
        QString current_password_;
        QString current_temp_file_;
        bool is_remote_map_;
        QString remote_map_base_name_; // 存储远程地图的原始基础名称（不含路径和扩展名）

        // SCP传输相关
        QProcess *scp_process_;
        bool scp_password_sent_;
        QString scp_output_;
    };

} // namespace map_edit

#endif // TF_SHIFT_PANEL_HPP
