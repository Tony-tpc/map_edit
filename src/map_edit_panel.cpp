#include "map_edit/map_edit_panel.h"
#include <pluginlib/class_list_macros.hpp>

namespace map_edit
{

    // MapEditPanel implementation
    MapEditPanel::MapEditPanel(QWidget *parent)
        : rviz_common::Panel(parent)
    {
        MapEditNode = std::make_shared<rclcpp::Node>("mapedit");
        setupUI();
    }

    MapEditPanel::~MapEditPanel()
    {
    }

    void MapEditPanel::onInitialize()
    {
        // Initialization handled in constructor
    }

    void MapEditPanel::setupUI()
    {
        main_layout_ = new QVBoxLayout;

        // 一键保存组
        save_group_ = new QGroupBox("文件管理");
        QVBoxLayout *save_layout = new QVBoxLayout();

        // 当前地图显示
        current_map_label_ = new QLabel("当前地图: 未加载");
        current_map_label_->setStyleSheet("QLabel { color: #333; font-weight: bold; padding: 5px; }");
        save_layout->addWidget(current_map_label_);

        // 保存按钮组
        QHBoxLayout *save_btn_layout = new QHBoxLayout();

        save_all_btn_ = new QPushButton("保存到本地");
        save_all_btn_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-size: 12px; padding: 6px; }");
        save_btn_layout->addWidget(save_all_btn_);

        save_layout->addLayout(save_btn_layout);

        // 创建选项卡控件用于选择地图来源
        map_source_tabs_ = new QTabWidget();
        setupLocalTab();

        save_layout->addWidget(map_source_tabs_);

        save_group_->setLayout(save_layout);

        // 状态显示
        status_label_ = new QLabel("就绪 - 请先打开一个地图文件");
        status_label_->setStyleSheet("QLabel { background-color: #f0f0f0; padding: 8px; border: 1px solid #ccc; border-radius: 4px; }");

        // 文件说明
        info_label_ = new QLabel(
            "保存文件说明:\n"
            "• map.yaml - 地图配置文件\n"
            "• map.pgm - 地图图像文件\n"
            "提示: 文件将保存到当前地图的同一目录");
        info_label_->setStyleSheet("QLabel { color: #666; font-size: 11px; padding: 8px; }");

        // 话题存在判断
        if (isTopicExist("/map"))
        {
            status_label_->setText("就绪 - 地图话题存在");
        }

        // 进度对话框设置
        progress_dialog_ = new QProgressDialog(this);
        progress_dialog_->setWindowModality(Qt::WindowModal);
        progress_dialog_->setCancelButton(nullptr); // 暂时禁用取消按钮
        progress_dialog_->reset();
        progress_dialog_->setMinimumDuration(0);

        // 组装主布局
        main_layout_->addWidget(save_group_);
        main_layout_->addWidget(new QLabel("状态:"));
        main_layout_->addWidget(status_label_);
        main_layout_->addWidget(info_label_);
        main_layout_->addStretch();

        setLayout(main_layout_);

        // 连接信号
        connect(save_all_btn_, SIGNAL(clicked()), this, SLOT(saveAllFiles()));
    }

    void MapEditPanel::setupLocalTab()
    {
        local_tab_ = new QWidget();
        QVBoxLayout *local_layout = new QVBoxLayout();

        // 本地地图打开按钮
        open_local_btn_ = new QPushButton("选择本地地图文件");
        open_local_btn_->setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-size: 12px; padding: 6px; }");
        local_layout->addWidget(open_local_btn_);

        // 添加说明
        QLabel *local_info = new QLabel("支持格式: YAML配置文件 (*.yaml, *.yml)\n或PGM图像文件 (*.pgm)");
        local_info->setStyleSheet("QLabel { color: #666; font-size: 10px; padding: 4px; }");
        local_layout->addWidget(local_info);

        local_layout->addStretch();
        local_tab_->setLayout(local_layout);
        map_source_tabs_->addTab(local_tab_, "本地地图");

        // 连接信号
        connect(open_local_btn_, SIGNAL(clicked()), this, SLOT(openMap()));
    }

    void MapEditPanel::openMap()
    {
        if (isTopicExist("/map"))
        {
            status_label_->setText("存在 /map 话题，无法添加地图");
            return;
        }

        // // 设置默认路径为ros_map_edit/maps目录
        QString default_path = "/home/tony/ros/origin_nav2025_developing/src/bringup/map";

        // // 如果目录不存在，尝试其他可能的路径
        // QDir maps_dir(default_path);
        // if (!maps_dir.exists())
        // {
        //     // 尝试相对于工作目录的路径
        //     QStringList possible_paths = {
        //         "ros_map_edit/maps",
        //         "../src/ros_map_edit/maps",
        //         "../../src/ros_map_edit/maps",
        //         QDir::homePath() + "/ros_ws/cursor_ws/src/ros_map_edit/maps"};

        //     for (const QString &path : possible_paths)
        //     {
        //         if (QDir(path).exists())
        //         {
        //             default_path = path;
        //             break;
        //         }
        //     }
        // }

        QString filename = QFileDialog::getOpenFileName(this,
                                                        "打开地图文件",
                                                        default_path,
                                                        "YAML files (*.yaml)");

        if (!filename.isEmpty())
        {
            current_map_file_ = filename;

            // // 先清空所有消息，再加载并发布新地图
            // clearAllMessages();
            loadAndPublishMap(filename.toStdString());

            // 更新当前地图显示
            QString display_name = QFileInfo(filename).fileName();
            current_map_label_->setText("当前地图: " + display_name);
            current_map_label_->setStyleSheet("QLabel { color: #007700; font-weight: bold; padding: 5px; }");
        }
    }

    void MapEditPanel::loadAndPublishMap(const std::string &yaml_filename)
    {
        try
        {
            status_label_->setText("正在加载地图: " + QString::fromStdString(yaml_filename));

            // 加载 YAML 文件
            YAML::Node config = YAML::LoadFile(yaml_filename);
            std::string image_file = config["image"].as<std::string>();
            double resolution = config["resolution"].as<double>();
            auto origin = config["origin"];
            double origin_x = origin[0].as<double>();
            double origin_y = origin[1].as<double>();
            double origin_theta = origin[2].as<double>();
            int negate = config["negate"].as<int>();
            double occupied_thresh = config["occupied_thresh"].as<double>();
            double free_thresh = config["free_thresh"].as<double>();

            // 构造完整图像路径
            std::string image_path;
            QFileInfo qfi(QString::fromStdString(yaml_filename));
            QString base_dir = qfi.absolutePath();
            if (image_file[0] == '/')
                image_path = image_file;
            else
                image_path = (base_dir + "/" + QString::fromStdString(image_file)).toStdString();

            // 加载图像
            QImage image(QString::fromStdString(image_path));
            if (image.isNull())
            {
                status_label_->setText("图像加载失败: " + QString::fromStdString(image_path));
                return;
            }
            image = image.convertToFormat(QImage::Format_Grayscale8);
            int width, height;
            width = image.width();
            height = image.height();
            // 填充 OccupancyGrid
            nav_msgs::msg::OccupancyGrid map_msg;
            map_msg.info.resolution = resolution;
            map_msg.info.width = width;
            map_msg.info.height = height;
            map_msg.info.origin.position.x = origin_x;
            map_msg.info.origin.position.y = origin_y;
            map_msg.info.origin.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, origin_theta);
            map_msg.info.origin.orientation.x = q.x();
            map_msg.info.origin.orientation.y = q.y();
            map_msg.info.origin.orientation.z = q.z();
            map_msg.info.origin.orientation.w = q.w();

            map_msg.data.resize(width * height);

            for (int y = 0; y < height; ++y)
            {
                for (int x = 0; x < width; ++x)
                {
                    // 图像坐标系到地图坐标系的转换：翻转Y轴
                    // int image_index = y * width + x;              // 图像中的索引（Y轴向下）
                    int map_index = (height - 1 - y) * width + x; // 地图中的索引（Y轴向上）

                    uint8_t pixel = qGray(image.pixel(x, y));

                    // 转换像素值到概率 (根据negate标志)
                    double p;
                    if (negate)
                    {
                        p = pixel / 255.0;
                    }
                    else
                    {
                        p = (255 - pixel) / 255.0;
                    }

                    // 根据阈值转换为占用栅格值
                    if (p > occupied_thresh)
                    {
                        map_msg.data[map_index] = 100; // 占用
                    }
                    else if (p < free_thresh)
                    {
                        map_msg.data[map_index] = 0; // 自由
                    }
                    else
                    {
                        map_msg.data[map_index] = -1; // 未知
                    }
                }
            }

            // 5. 发布地图
            publishMap(map_msg);

            // 6. 状态栏提示
            status_label_->setText("地图加载完成: " + QFileInfo(QString::fromStdString(image_path)).fileName());

            // 可选：加载虚拟墙或其他附属文件
            // loadCorrespondingFiles(yaml_filename);
        }
        catch (const std::exception &e)
        {
            status_label_->setText("加载失败: " + QString::fromStdString(e.what()));
        }
    }

    void MapEditPanel::publishMap(const nav_msgs::msg::OccupancyGrid &map)
    {
        static rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
        static bool initialized = false;

        if (!initialized)
        {
            rclcpp::QoS qos(rclcpp::KeepLast(1));
            qos.transient_local();

            map_pub = MapEditNode->create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos);
            initialized = true;

            RCLCPP_INFO(MapEditNode->get_logger(), "地图发布器已初始化");
        }

        // 复制地图并设置 header
        auto map_msg = map;
        map_msg.header.stamp = MapEditNode->now();
        map_msg.header.frame_id = "map";

        // 发布地图多次以确保被接收
        for (int i = 0; i < 3; ++i)
        {
            map_pub->publish(map_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            rclcpp::spin_some(MapEditNode);
        }

        // 设置状态栏信息（Qt）
        QString debug_info = QString("地图已发布: %1x%2 像素, 分辨率: %3 m/pixel")
                                 .arg(map_msg.info.width)
                                 .arg(map_msg.info.height)
                                 .arg(map_msg.info.resolution);
        status_label_->setText(debug_info);

        RCLCPP_INFO(MapEditNode->get_logger(),
                    "地图已发布: %dx%d, 分辨率: %.3f",
                    map_msg.info.width, map_msg.info.height, map_msg.info.resolution);
    }

    void MapEditPanel::saveAllFiles()
    {
        if (!isTopicExist("/map_edit"))
        {
            QMessageBox::warning(this, "警告", "请先发布地图");
            return;
        }
        QString fileName = QFileDialog::getSaveFileName(
            this,
            "保存文件",
            "",
            "YAML files (*.yaml)");
        // 如果没有扩展名，则手动添加默认扩展名
        if (!fileName.isEmpty())
        {
            if (QFileInfo(fileName).suffix().isEmpty())
            {
                fileName += ".yaml";
            }
            try
            {
                ToolManager &toolManager = ToolManager::getInstance();

                MapEraserTool *eraserTool = toolManager.getMapEraserTool();
                if (eraserTool && eraserTool->getCurrentMap().data.size() > 0)
                {
                    std::string filename = fileName.toStdString();
                    std::string base = filename.substr(0, filename.find_last_of("."));
                    std::string pgm_filename = base + ".pgm";
                    std::string pgm_basename = pgm_filename.substr(pgm_filename.find_last_of("/\\") + 1);
                    std::string yaml_filename = base + ".yaml";
                    if (MapEditPanel::saveMapFiles(pgm_filename, eraserTool->getCurrentMap()) && MapEditPanel::saveMapYaml(yaml_filename, pgm_basename, eraserTool->getCurrentMap()))
                    {
                        QMessageBox::information(this, "消息", "成功保存文件");
                    }
                    else
                    {
                        QMessageBox::critical(this, "错误", "无法保存地图文件");
                    }
                }
            }
            catch (const std::exception &e)
            {
                QString error_msg = "保存过程中出现错误: " + QString::fromStdString(e.what());
                status_label_->setText("保存失败");
                QMessageBox::critical(this, "保存错误", error_msg);
            }
        }
    }

    bool MapEditPanel::saveMapFiles(const std::string &fileName, const nav_msgs::msg::OccupancyGrid &map)
    {
        // std::string base = filename.substr(0, filename.find_last_of("."));
        // std::string pgm_filename = base + ".pgm";
        // std::string pgm_basename = pgm_filename.substr(pgm_filename.find_last_of("/\\") + 1);
        std::ofstream file(fileName, std::ios::binary);
        if (!file.is_open())
        {
            // last_error_ = "Failed to create PGM file: " + filename;
            return false;
        }

        // 写入PGM文件头
        file << "P5\n";
        file << "# Created by ros_map_edit\n";
        file << map.info.width << " " << map.info.height << "\n";
        file << "255\n";

        // 准备图像数据
        std::vector<uint8_t> image_data(map.info.width * map.info.height);
        const double occupied_thresh = 0.65;
        const double free_thresh = 0.196;
        const bool negate = false;

        // 注意：需要翻转Y轴以匹配图像坐标系（Y轴向下）
        for (int y = 0; y < static_cast<int>(map.info.height); ++y)
        {
            for (int x = 0; x < static_cast<int>(map.info.width); ++x)
            {
                // 地图坐标系到图像坐标系的转换：翻转Y轴
                int map_index = (map.info.height - 1 - y) * map.info.width + x; // 地图中的索引（Y轴向上）
                int image_index = y * map.info.width + x;                       // 图像中的索引（Y轴向下）

                int8_t cell = map.data[map_index];
                uint8_t pixel;

                if (cell == 0)
                {
                    // 自由空间 -> 根据negate转换
                    pixel = negate ? 0 : 254;
                }
                else if (cell == 100)
                {
                    // 占用空间 -> 根据negate转换
                    pixel = negate ? 254 : 0;
                }
                else
                {
                    // 未知空间 -> 灰色
                    pixel = 205;
                }

                image_data[image_index] = pixel;
            }
        }
        file.write(reinterpret_cast<char *>(image_data.data()), image_data.size());

        if (!file)
        {
            // last_error_ = "Failed to write image data to PGM file";
            return false;
        }
        file.close();
        return true;
    }

    bool MapEditPanel::saveMapYaml(const std::string &yaml_file, const std::string &pgm_filename, const nav_msgs::msg::OccupancyGrid &map)
    {
        std::ofstream yaml_out(yaml_file);
        if (!yaml_out.is_open())
        {
            return false;
        }

        yaml_out << "image: " << pgm_filename << "\n";
        yaml_out << "resolution: " << map.info.resolution << "\n";
        yaml_out << "origin: ["
                 << map.info.origin.position.x << ", "
                 << map.info.origin.position.y << ", "
                 << tf2::getYaw(map.info.origin.orientation) << "]\n";
        yaml_out << "negate: 0\n";
        yaml_out << "occupied_thresh: 0.65\n";
        yaml_out << "free_thresh: 0.196\n";
        if (!yaml_out)
        {
            return false;
        }
        yaml_out.close();
        return true;
    }

    bool MapEditPanel::isTopicExist(const std::string &topic_name)
    {
        return (MapEditNode->get_publishers_info_by_topic(topic_name).size() > 0);
    }

} // namespace map_edit

PLUGINLIB_EXPORT_CLASS(map_edit::MapEditPanel, rviz_common::Panel)
