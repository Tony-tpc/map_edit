#include "map_edit/map_edit_panel.h"
#include <pluginlib/class_list_macros.hpp>
#include <QSlider>
#include <cmath>

namespace
{
    constexpr float kZSliderStepMeters = 0.05f;
    constexpr float kZSliderUnitsPerMeter = 1.0f / kZSliderStepMeters;
    constexpr int kZSliderMarginSteps = 10; // 在点云实际范围两端各保留 0.5 m
}

extern map_edit::RegionTool *g_region_tool;
namespace map_edit
{

    // MapEditPanel implementation
    MapEditPanel::MapEditPanel(QWidget *parent)
        : rviz_common::Panel(parent)
    {
        last_opened_path_ = QDir::currentPath();
        MapEditNode = std::make_shared<rclcpp::Node>("mapedit");
        if (g_region_tool)
        {
            connectTool(g_region_tool);
        }
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
        save_layout->addWidget(current_map_label_);

        // 保存按钮组
        QVBoxLayout *save_btn_layout = new QVBoxLayout();

        save_all_btn_ = new QPushButton("保存到本地");
        save_btn_layout->addWidget(save_all_btn_);

        save_yaml_btn_ = new QPushButton("保存到yaml文件");
        save_btn_layout->addWidget(save_yaml_btn_);

        save_layout->addLayout(save_btn_layout);

        // 创建选项卡控件用于选择地图来源
        map_source_tabs_ = new QTabWidget();
        setupRegionUI();
        setupLocalTab();
        save_layout->addWidget(map_source_tabs_);

        save_group_->setLayout(save_layout);

        // 点云辅助组 (PCD)
        setupPcdUI();

        // 状态显示
        status_label_ = new QLabel("就绪 - 请先打开一个地图文件");

        // 文件说明
        info_label_ = new QLabel(
            "保存文件说明:\n"
            "• map.yaml - 地图配置文件\n"
            "• map.pgm - 地图图像文件\n"
            "提示: 文件将保存到当前地图的同一目录");

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
        main_layout_->addWidget(save_group_, 1);         // 比例 1
        main_layout_->addWidget(pcd_group_, 1);          // 点云辅助（按内容高度，不拉伸）
        main_layout_->addWidget(new QLabel("状态:"), 0); // 固定高度，内容短
        main_layout_->addWidget(status_label_, 1);
        main_layout_->addWidget(info_label_, 1);
        main_layout_->addStretch(1);
        setLayout(main_layout_);

        // 连接信号
        connect(save_all_btn_, SIGNAL(clicked()), this, SLOT(saveAllFiles()));
        connect(save_yaml_btn_, SIGNAL(clicked()), this, SLOT(saveRegionYaml()));
    }

    void MapEditPanel::setupPcdUI()
    {
        pcd_group_ = new QGroupBox("点云辅助 (PCD)");
        pcd_group_->setToolTip("加载 PCD 点云并过滤 Z 高度范围，点云按高度 Jet 渐变着色(蓝→红)发布到 /map_edit_pcd，在 RViz 中显示为 Map Edit PCD");
        
        QVBoxLayout *pcd_layout = new QVBoxLayout();

        // 加载按钮独占一行，样式与上方“选择本地地图文件”一致（全宽）
        load_pcd_btn_ = new QPushButton("加载 PCD 文件");
        load_pcd_btn_->setToolTip("选择并加载 PCD 点云文件");
        pcd_layout->addWidget(load_pcd_btn_);

        // 点云信息（与上方 local_info 一致的字体间距）
        pcd_info_label_ = new QLabel("未加载点云");
        pcd_info_label_->setObjectName("pcdInfoLabel");
        pcd_info_label_->setWordWrap(false);
        pcd_info_label_->setMinimumWidth(0);
        pcd_layout->addWidget(pcd_info_label_);

        // Z 最小滑块
        QHBoxLayout *z_min_row = new QHBoxLayout();
        z_min_row->setSpacing(4);
        z_min_caption_ = new QLabel("Z 最小:");
        z_min_caption_->setFixedWidth(42);
        z_min_row->addWidget(z_min_caption_, 0);
        z_min_slider_ = new QSlider(Qt::Horizontal);
        z_min_slider_->setRange(-2000, 2000); // 值 = 米 * 20 (0.05m 步进)
        z_min_slider_->setSingleStep(1);
        z_min_slider_->setPageStep(2);
        z_min_slider_->setToolTip("每格调整 0.05 m");
        z_min_slider_->setEnabled(false);
        z_min_slider_->setFixedHeight(24);
        z_min_row->addWidget(z_min_slider_, 1);
        z_min_value_label_ = new QLabel("--");
        z_min_value_label_->setFixedWidth(58);
        z_min_value_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        z_min_row->addWidget(z_min_value_label_, 0);
        pcd_layout->addLayout(z_min_row);

        // Z 最大滑块
        QHBoxLayout *z_max_row = new QHBoxLayout();
        z_max_row->setSpacing(4);
        z_max_caption_ = new QLabel("Z 最大:");
        z_max_caption_->setFixedWidth(42);
        z_max_row->addWidget(z_max_caption_, 0);
        z_max_slider_ = new QSlider(Qt::Horizontal);
        z_max_slider_->setRange(-2000, 2000);
        z_max_slider_->setSingleStep(1);
        z_max_slider_->setPageStep(2);
        z_max_slider_->setToolTip("每格调整 0.05 m");
        z_max_slider_->setEnabled(false);
        z_max_slider_->setFixedHeight(24);
        z_max_row->addWidget(z_max_slider_, 1);
        z_max_value_label_ = new QLabel("--");
        z_max_value_label_->setFixedWidth(58);
        z_max_value_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        z_max_row->addWidget(z_max_value_label_, 0);
        pcd_layout->addLayout(z_max_row);

        pcd_group_->setLayout(pcd_layout);

        // 滑块样式美化（主题色与按钮一致，尺寸加大便于拖动）
        QString slider_style =
            "QSlider::groove:horizontal { height: 6px; background: #dfe3e8; border-radius: 4px; }"
            "QSlider::sub-page:horizontal { background: #2196F3; border-radius: 2px; }"
            "QSlider::add-page:horizontal { background: #dfe3e8; border-radius: 2px; }"
            "QSlider::handle:horizontal { width: 14px; height: 14px; margin: -5px 0; "
            "border-radius: 6px; background: #2196F3; border: 1px solid #ffffff; }"
            "QSlider::handle:horizontal:hover { background: #1976D2; }"
            "QSlider::groove:horizontal:disabled { background: #eceff2; }"
            "QSlider::handle:horizontal:disabled { background: #b8c0c8; }";
        z_min_slider_->setStyleSheet(slider_style);
        z_max_slider_->setStyleSheet(slider_style);

        // 连接信号
        connect(load_pcd_btn_, SIGNAL(clicked()), this, SLOT(loadPcdFile()));
        connect(z_min_slider_, SIGNAL(valueChanged(int)), this, SLOT(onZMinChanged(int)));
        connect(z_max_slider_, SIGNAL(valueChanged(int)), this, SLOT(onZMaxChanged(int)));
    }

    void MapEditPanel::loadPcdFile()
    {
        QString filename = QFileDialog::getOpenFileName(this,
                                                        "打开 PCD 点云文件",
                                                        last_opened_path_,
                                                        "PCD files (*.pcd);;All files (*)");
        if (filename.isEmpty())
            return;

        last_opened_path_ = QFileInfo(filename).absolutePath();

        if (!pcd_loader_)
        {
            pcd_loader_ = std::make_unique<PcdLoader>("map");
        }

        QApplication::setOverrideCursor(Qt::WaitCursor);
        bool ok = pcd_loader_->load(filename.toStdString());
        QApplication::restoreOverrideCursor();

        if (!ok)
        {
            QMessageBox::critical(this, "错误", "加载 PCD 文件失败:\n" + filename);
            return;
        }

        // 每个整数刻度对应 0.05 m，并在点云实际范围两端各扩展 0.5 m。
        const int cloud_min_step = static_cast<int>(std::floor(pcd_loader_->getCloudZMin() * kZSliderUnitsPerMeter));
        const int cloud_max_step = static_cast<int>(std::ceil(pcd_loader_->getCloudZMax() * kZSliderUnitsPerMeter));
        const int lo = cloud_min_step - kZSliderMarginSteps;
        const int hi = cloud_max_step + kZSliderMarginSteps;
        z_min_slider_->setRange(lo, hi);
        z_max_slider_->setRange(lo, hi);
        z_min_slider_->setValue(cloud_min_step);
        z_max_slider_->setValue(cloud_max_step);
        z_min_slider_->setEnabled(true);
        z_max_slider_->setEnabled(true);
        z_min_value_label_->setText(QString::number(cloud_min_step * kZSliderStepMeters, 'f', 2) + " m");
        z_max_value_label_->setText(QString::number(cloud_max_step * kZSliderStepMeters, 'f', 2) + " m");

        updatePcdInfoLabel();
        status_label_->setText(QString("PCD 加载成功: %1 (%2 点)").arg(QFileInfo(filename).fileName()).arg(pcd_loader_->pointCount()));
    }

    void MapEditPanel::onZMinChanged(int value)
    {
        if (!pcd_loader_ || !pcd_loader_->isLoaded())
            return;
        // 互锁：Z 最小不能超过 Z 最大
        if (value >= z_max_slider_->value())
        {
            z_max_slider_->setValue(value);
        }
        const float z = value * kZSliderStepMeters;
        pcd_loader_->setZMin(z);
        z_min_value_label_->setText(QString::number(z, 'f', 2) + " m");
        updatePcdInfoLabel();
    }

    void MapEditPanel::onZMaxChanged(int value)
    {
        if (!pcd_loader_ || !pcd_loader_->isLoaded())
            return;
        // 互锁：Z 最大不能小于 Z 最小
        if (value <= z_min_slider_->value())
        {
            z_min_slider_->setValue(value);
        }
        const float z = value * kZSliderStepMeters;
        pcd_loader_->setZMax(z);
        z_max_value_label_->setText(QString::number(z, 'f', 2) + " m");
        updatePcdInfoLabel();
    }

    void MapEditPanel::updatePcdInfoLabel()
    {
        if (!pcd_loader_ || !pcd_loader_->isLoaded())
        {
            pcd_info_label_->setText("未加载点云");
            return;
        }
        const QString info =
            QString("%1 · 总 %2 点 · 显示 %3 点 · Z [%4, %5] m")
                .arg(QFileInfo(QString::fromStdString(pcd_loader_->filePath())).fileName())
                .arg(pcd_loader_->pointCount())
                .arg(pcd_loader_->filteredPointCount())
                .arg(pcd_loader_->getCloudZMin(), 0, 'f', 2)
                .arg(pcd_loader_->getCloudZMax(), 0, 'f', 2);
        pcd_info_label_->setText(info);
        pcd_info_label_->setToolTip(info);
    }

    void MapEditPanel::setupRegionUI()
    {
        // 区域管理组
        region_group = new QGroupBox("区域管理");
        QVBoxLayout *region_layout_ = new QVBoxLayout();
        QVBoxLayout *btn_layout = new QVBoxLayout();

        open_region_yaml_btn_ = new QPushButton("打开区域 YAML 文件");
        clear_region_btn_ = new QPushButton("清空区域");

        btn_layout->addWidget(open_region_yaml_btn_, 2);
        btn_layout->addWidget(clear_region_btn_, 2);
        region_layout_->addLayout(btn_layout);
        // 滚动区域，用于显示多个区域卡片
        region_scroll_area_ = new QScrollArea();
        region_scroll_area_->setWidgetResizable(true);
        region_scroll_area_->setFrameShape(QFrame::NoFrame);

        region_container_ = new QWidget();
        region_list_layout_ = new QVBoxLayout(region_container_);
        region_list_layout_->setSpacing(8);
        region_list_layout_->setAlignment(Qt::AlignTop);

        region_scroll_area_->setWidget(region_container_);
        region_layout_->addWidget(region_scroll_area_, 1);

        region_group->setLayout(region_layout_);
        main_layout_->addWidget(region_group, 3);
        region_group->setVisible(false);
        connect(open_region_yaml_btn_, SIGNAL(clicked()), this, SLOT(openRegionYaml()));
        connect(clear_region_btn_, SIGNAL(clicked()), this, SLOT(clearRegions()));
    }
    void MapEditPanel::setupLocalTab()
    {
        local_tab_ = new QWidget();
        QVBoxLayout *local_layout = new QVBoxLayout();

        // 本地地图打开按钮
        open_local_btn_ = new QPushButton("选择本地地图文件");
        local_layout->addWidget(open_local_btn_);

        // 添加说明
        local_info = new QLabel("支持格式: YAML配置文件 (*.yaml, *.yml)\n或PGM图像文件 (*.pgm)");
        local_layout->addWidget(local_info);

        local_layout->addStretch();
        local_tab_->setLayout(local_layout);
        map_source_tabs_->addTab(local_tab_, "本地地图");

        // 连接信号
        connect(open_local_btn_, SIGNAL(clicked()), this, SLOT(openMap()));
    }
    void MapEditPanel::connectTool(RegionTool *tool)
    {
        QObject::connect(tool, &RegionTool::regiontoolInitialized,
                         this, &MapEditPanel::onRegionToolInitialized);
        QObject::connect(tool, &RegionTool::regiontooldeInitialized,
                         this, &MapEditPanel::onRegiontoolDeInitialized);
        QObject::connect(tool, &RegionTool::regionsChanged,
                         this, &MapEditPanel::onregionsChanged);
    }

    void MapEditPanel::onRegionToolInitialized()
    {
        save_group_->setVisible(false);
        region_group->setVisible(true);
    }
    void MapEditPanel::onRegiontoolDeInitialized()
    {
        save_group_->setVisible(true);
        region_group->setVisible(false);
    }

    void MapEditPanel::onregionsChanged()
    {
        regions_ = g_region_tool->getRegions();
        refreshRegionListUI();
    }

    void MapEditPanel::openMap()
    {
        if (isTopicExist("/map"))
        {
            status_label_->setText("存在 /map 话题，无法添加地图");
            return;
        }

        // 检查是否需要保存
        if (eraserTool && eraserTool->getCurrentMap().data.size() > 0 && !is_saved)
        {
            QMessageBox::StandardButton reply = QMessageBox::question(this, "保存地图", "当前地图未保存，是否保存？", QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
            if (reply == QMessageBox::Yes)
            {
                saveAllFiles();
                return;
            }
            else if (reply == QMessageBox::Cancel)
            {
                return;
            }
            else if (reply == QMessageBox::No)
            {
                eraserTool->reloadMap();
            }
        }

        QString filename = QFileDialog::getOpenFileName(this,
                                                        "打开地图文件",
                                                        last_opened_path_,
                                                        "YAML files (*.yaml)");

        if (!filename.isEmpty())
        {
            last_opened_path_ = QFileInfo(filename).absolutePath();
            current_map_file_ = filename;

            // // 先清空所有消息，再加载并发布新地图
            // clearAllMessages();
            loadAndPublishMap(filename.toStdString());

            // 更新当前地图显示
            QString display_name = QFileInfo(filename).fileName();
            current_map_label_->setText("当前地图: " + display_name);
        }
    }

    void MapEditPanel::openRegionYaml()
    {
        if (eraserTool == nullptr || eraserTool->getCurrentMap().data.size() == 0)
        {
            QMessageBox::warning(this, "警告", "请先发布地图");
            return;
        }

        QString filename = QFileDialog::getOpenFileName(this,
                                                        "打开区域文件",
                                                        last_opened_path_,
                                                        "YAML files (*.yaml)");

        if (filename.isEmpty())
            return;

        last_opened_path_ = QFileInfo(filename).absolutePath();

        try
        {
            YAML::Node root = YAML::LoadFile(filename.toStdString());
            regions_.clear();

            if (root["regions"] && root["regions"].IsSequence())
            {
                for (const auto &region_node : root["regions"])
                {
                    Region region;
                    // Standard fields
                    if (region_node["id"])
                        region.id = region_node["id"].as<std::string>();
                    if (region_node["frame_id"])
                        region.frame_id = region_node["frame_id"].as<std::string>();
                    if (region_node["type"])
                        region.type = region_node["type"].as<int>();
                    if (region_node["notes"])
                        region.notes = region_node["notes"].as<std::string>();

                    // Points
                    if (region_node["points"] && region_node["points"].IsSequence())
                    {
                        for (const auto &point_node : region_node["points"])
                        {
                            geometry_msgs::msg::Point point;
                            if (point_node["x"])
                                point.x = point_node["x"].as<double>();
                            if (point_node["y"])
                                point.y = point_node["y"].as<double>();
                            if (point_node["z"])
                                point.z = point_node["z"].as<double>();
                            region.points.push_back(point);
                        }
                    }

                    // Attributes (Iterate all keys)
                    for (YAML::const_iterator it = region_node.begin(); it != region_node.end(); ++it)
                    {
                        std::string key = it->first.as<std::string>();
                        // Skip standard keys
                        if (key == "id" || key == "frame_id" || key == "type" || key == "notes" || key == "points")
                        {
                            continue;
                        }
                        // Add to attributes (store as string)
                        region.attributes[key] = it->second.as<std::string>();
                    }

                    if (region.points.size() >= 3)
                    {
                        regions_.push_back(region);
                    }
                }
                refreshRegionListUI();
                g_region_tool->setRegions(regions_);
            }
            else
            {
                QMessageBox::warning(this, "警告", "YAML 文件格式错误: 找不到 regions 列表");
            }
        }
        catch (const YAML::Exception &e)
        {
            QMessageBox::critical(this, "错误", QString("解析 YAML 失败:\n") + e.what());
        }
    }

    void MapEditPanel::clearRegions()
    {
        QMessageBox::StandardButton reply = QMessageBox::question(this, "清空地图区域", "你确定要清空区域吗", QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::Yes)
        {
            g_region_tool->clearRegions();
            onregionsChanged();
            return;
        }
        else if (reply == QMessageBox::No)
        {
            return;
        }
    }

    void MapEditPanel::refreshRegionListUI()
    {
        // 清空旧内容
        QLayoutItem *item;
        while ((item = region_list_layout_->takeAt(0)) != nullptr)
        {
            delete item->widget();
            delete item;
        }

        // 为每个 region 创建一个卡片
        for (size_t i = 0; i < regions_.size(); ++i)
        {
            const auto &region = regions_[i];
            QWidget *card = new QWidget();
            QHBoxLayout *card_layout = new QHBoxLayout(card);
            card_layout->setContentsMargins(8, 4, 8, 4);

            QLabel *label = new QLabel(QString::fromStdString(region.id + " (" + region.notes + ")"));
            label->setStyleSheet("font-weight: bold; color: #333;");

            QPushButton *edit_btn = new QPushButton("编辑");
            QPushButton *del_btn = new QPushButton("删除");

            edit_btn->setFixedWidth(50);
            del_btn->setFixedWidth(50);

            card_layout->addWidget(label);
            card_layout->addStretch();
            card_layout->addWidget(edit_btn);
            card_layout->addWidget(del_btn);

            card->setStyleSheet(
                "QWidget { background-color: #f8f8f8; border: 1px solid #ccc; border-radius: 6px; }"
                "QPushButton { background-color: #e6e6e6; border-radius: 4px; padding: 2px; }"
                "QPushButton:hover { background-color: #d0d0d0; }");

            region_list_layout_->addWidget(card);

            // 绑定信号
            connect(edit_btn, &QPushButton::clicked, this, [this, i]()
                    {
            Region &r = regions_[i];

            QDialog dialog(this);
            dialog.setWindowTitle("编辑区域信息");
            dialog.resize(500, 600);
            QVBoxLayout *layout = new QVBoxLayout(&dialog);

            QFormLayout *form = new QFormLayout();

            QLineEdit *id_edit = new QLineEdit(QString::fromStdString(r.id));
            QLineEdit *frame_edit = new QLineEdit(QString::fromStdString(r.frame_id));
            QSpinBox *type_spin = new QSpinBox();
            type_spin->setRange(0, 100);
            type_spin->setValue(r.type);
            QTextEdit *notes_edit = new QTextEdit(QString::fromStdString(r.notes));
            notes_edit->setMaximumHeight(60);

            form->addRow("区域ID：", id_edit);
            form->addRow("Frame ID：", frame_edit);
            form->addRow("类型（Type）：", type_spin);
            form->addRow("备注（Notes）：", notes_edit);
            layout->addLayout(form);

            // Attributes Table
            QGroupBox *attr_group = new QGroupBox("自定义属性");
            QVBoxLayout *attr_layout = new QVBoxLayout(attr_group);

            QTableWidget *attr_table = new QTableWidget();
            attr_table->setColumnCount(2);
            attr_table->setHorizontalHeaderLabels({"Key", "Value"});
            attr_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
            attr_table->setSelectionBehavior(QAbstractItemView::SelectRows);

            // Load attributes
            attr_table->setRowCount(r.attributes.size());
            int row = 0;
            for (const auto &kv : r.attributes)
            {
                attr_table->setItem(row, 0, new QTableWidgetItem(QString::fromStdString(kv.first)));
                attr_table->setItem(row, 1, new QTableWidgetItem(QString::fromStdString(kv.second)));
                row++;
            }

            attr_layout->addWidget(attr_table);

            // Add/Remove buttons
            QHBoxLayout *attr_btn_layout = new QHBoxLayout();
            QPushButton *add_attr_btn = new QPushButton("添加属性");
            QPushButton *del_attr_btn = new QPushButton("删除属性");
            attr_btn_layout->addWidget(add_attr_btn);
            attr_btn_layout->addWidget(del_attr_btn);
            attr_layout->addLayout(attr_btn_layout);

            layout->addWidget(attr_group);

            // Button connections
            connect(add_attr_btn, &QPushButton::clicked, [attr_table]()
                    {
                        int row = attr_table->rowCount();
                        attr_table->insertRow(row);
                        attr_table->setItem(row, 0, new QTableWidgetItem("key"));
                        attr_table->setItem(row, 1, new QTableWidgetItem("value"));
                    });

            connect(del_attr_btn, &QPushButton::clicked, [attr_table]()
                    {
                        int row = attr_table->currentRow();
                        if (row >= 0)
                            attr_table->removeRow(row);
                    });

            QHBoxLayout *btn_layout = new QHBoxLayout();
            QPushButton *ok_btn = new QPushButton("确定");
            QPushButton *cancel_btn = new QPushButton("取消");
            btn_layout->addStretch();
            btn_layout->addWidget(ok_btn);
            btn_layout->addWidget(cancel_btn);
            layout->addLayout(btn_layout);

            connect(ok_btn, &QPushButton::clicked, &dialog, &QDialog::accept);
            connect(cancel_btn, &QPushButton::clicked, &dialog, &QDialog::reject);

            if (dialog.exec() == QDialog::Accepted)
            {
                r.id = id_edit->text().toStdString();
                r.frame_id = frame_edit->text().toStdString();
                r.type = type_spin->value();
                r.notes = notes_edit->toPlainText().toStdString();

                // Save attributes
                r.attributes.clear();
                for (int j = 0; j < attr_table->rowCount(); ++j)
                {
                    QString key = attr_table->item(j, 0)->text();
                    QString val = attr_table->item(j, 1)->text();
                    if (!key.isEmpty())
                    {
                        r.attributes[key.toStdString()] = val.toStdString();
                    }
                }

                g_region_tool->setRegions(regions_);
                refreshRegionListUI();
            } });

            connect(del_btn, &QPushButton::clicked, this, [this, i]()
                    {
            regions_.erase(regions_.begin() + i);
            g_region_tool->setRegions(regions_);
            refreshRegionListUI(); });
        }

        region_list_layout_->addStretch();
        region_container_->adjustSize();
        region_scroll_area_->updateGeometry();
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
        if (eraserTool == nullptr || eraserTool->getCurrentMap().data.size() == 0)
        {
            QMessageBox::warning(this, "警告", "请先发布地图");
            return;
        }
        QString fileName = QFileDialog::getSaveFileName(
            this,
            "保存文件",
            last_opened_path_,
            "YAML files (*.yaml)");
        // 如果没有扩展名，则手动添加默认扩展名
        if (!fileName.isEmpty())
        {
            last_opened_path_ = QFileInfo(fileName).absolutePath();
            if (QFileInfo(fileName).suffix().isEmpty())
            {
                fileName += ".yaml";
            }
            try
            {
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
                        is_saved = true;
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

    void MapEditPanel::saveRegionYaml()
    {
        if (eraserTool == nullptr || eraserTool->getCurrentMap().data.size() == 0)
        {
            QMessageBox::warning(this, "警告", "请先发布地图");
            return;
        }
        regions_ = g_region_tool->getRegions();
        if (regions_.empty())
        {
            QMessageBox::warning(this, "警告", "无区域数据");
            return;
        }

        QString fileName = QFileDialog::getSaveFileName(
            this,
            "保存文件",
            last_opened_path_,
            "YAML files (*.yaml)");

        if (fileName.isEmpty())
            return;

        last_opened_path_ = QFileInfo(fileName).absolutePath();

        if (QFileInfo(fileName).suffix().isEmpty())
        {
            fileName += ".yaml";
        }

        try
        {
            YAML::Emitter out;
            out << YAML::BeginMap;
            out << YAML::Key << "regions" << YAML::Value << YAML::BeginSeq;

            for (const auto &region : regions_)
            {
                if (region.points.size() >= 3)
                {
                    out << YAML::BeginMap;
                    out << YAML::Key << "id" << YAML::Value << region.id;
                    out << YAML::Key << "frame_id" << YAML::Value << region.frame_id;
                    out << YAML::Key << "type" << YAML::Value << region.type;
                    out << YAML::Key << "notes" << YAML::Value << region.notes;

                    out << YAML::Key << "points" << YAML::Value << YAML::BeginSeq;
                    for (const auto &point : region.points)
                    {
                        out << YAML::BeginMap;
                        out << YAML::Key << "x" << YAML::Value << point.x;
                        out << YAML::Key << "y" << YAML::Value << point.y;
                        out << YAML::Key << "z" << YAML::Value << point.z;
                        out << YAML::EndMap;
                    }
                    out << YAML::EndSeq;

                    // Attributes (Flat structure)
                    for (const auto &kv : region.attributes)
                    {
                        out << YAML::Key << kv.first << YAML::Value << kv.second;
                    }

                    out << YAML::EndMap;
                }
            }
            out << YAML::EndSeq;
            out << YAML::EndMap;

            std::ofstream file(fileName.toStdString());
            if (file.is_open())
            {
                file << out.c_str();
                QMessageBox::information(this, "消息", "Saved " + QString::number(regions_.size()) + " regions");
            }
            else
            {
                QMessageBox::critical(this, "错误", "Failed to save regions file");
            }
        }
        catch (const YAML::Exception &e)
        {
            QMessageBox::critical(this, "错误", QString("保存 YAML 失败:\n") + e.what());
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
        const double occupied_thresh = 0.65;
        const double free_thresh = 0.196;
        yaml_out << "image: " << pgm_filename << "\n";
        yaml_out << "resolution: " << map.info.resolution << "\n";
        yaml_out << "origin: ["
                 << map.info.origin.position.x << ", "
                 << map.info.origin.position.y << ", "
                 << tf2::getYaw(map.info.origin.orientation) << "]\n";
        yaml_out << "negate: 0\n";
        yaml_out << "occupied_thresh: " << occupied_thresh << "\n";
        yaml_out << "free_thresh: " << free_thresh << "\n";
        if (!yaml_out)
        {
            return false;
        }
        yaml_out.close();
        return true;
    }
    void MapEditPanel::resizeEvent(QResizeEvent *event)
    {
        QWidget::resizeEvent(event);

        int panel_width = event->size().width();
        int panel_height = event->size().height();

        int base_size = std::max(panel_width, panel_height);
        int font_size = std::clamp(base_size / 60, 5, 18);
        int padding_px = std::clamp(base_size / 90, 5, 18);

        QFont font;
        font.setPointSize(font_size);

        current_map_label_->setFont(font);
        status_label_->setFont(font);
        info_label_->setFont(font);
        save_all_btn_->setFont(font);
        open_local_btn_->setFont(font);
        load_pcd_btn_->setFont(font);
        pcd_info_label_->setFont(font);
        z_min_caption_->setFont(font);
        z_max_caption_->setFont(font);
        z_min_value_label_->setFont(font);
        z_max_value_label_->setFont(font);

        QString label_style = QString(
                                  "QLabel { color: #333; font-weight: bold; padding: %1px; }")
                                  .arg(padding_px);

        current_map_label_->setStyleSheet(label_style);

        QString status_style = QString(
                                   "QLabel { background-color: #f0f0f0; padding: %1px; border: 1px solid #ccc; border-radius: 4px; }")
                                   .arg(padding_px);
        status_label_->setStyleSheet(status_style);

        QString info_style = QString(
                                 "QLabel { color: #666; font-size: %1pt; padding: %2px; }")
                                 .arg(font_size - 2)
                                 .arg(padding_px);
        info_label_->setStyleSheet(info_style);

        QString button_style = QString(
                                   "QPushButton { background-color: #4CAF50; color: white; font-size: %1pt; padding: %2px; }")
                                   .arg(font_size)
                                   .arg(padding_px);
        save_all_btn_->setStyleSheet(button_style);

        QString save_yaml_button_style = QString(
                                             "QPushButton { background-color:rgb(173, 175, 76); color: white; font-size: %1pt; padding: %2px; }")
                                             .arg(font_size)
                                             .arg(padding_px);
        save_yaml_btn_->setStyleSheet(save_yaml_button_style);

        QString open_btn_style = QString(
                                     "QPushButton { background-color: #2196F3; color: white; font-size: %1pt; padding: %2px; }")
                                     .arg(font_size)
                                     .arg(padding_px);

        QString open_pcd_style = QString(
                                "QPushButton { background-color: #6C63C7; color: white; font-size: %1pt; padding: %2px; }")
                                .arg(font_size)
                                .arg(padding_px);

        open_local_btn_->setStyleSheet(open_btn_style);
        load_pcd_btn_->setStyleSheet(open_pcd_style);

        QString open_region_button_style = QString(
                                               "QPushButton { background-color:rgb(120, 175, 76); color: white; font-size: %1pt; padding: %2px; }")
                                               .arg(font_size)
                                               .arg(padding_px);
        open_region_yaml_btn_->setStyleSheet(open_region_button_style);

        QString clear_region_btn_style = QString(
                                             "QPushButton { background-color:rgb(206, 83, 53); color: white; font-size: %1pt; padding: %2px; }")
                                             .arg(font_size)
                                             .arg(padding_px);
        clear_region_btn_->setStyleSheet(clear_region_btn_style);

        QString local_info_style = QString(
                                       "QLabel { color: #666; font-size: %1pt; padding: %2px; }")
                                       .arg(font_size - 2)
                                       .arg(padding_px);
        local_info->setStyleSheet(local_info_style);
        pcd_info_label_->setStyleSheet(local_info_style);
    }

    bool MapEditPanel::isTopicExist(const std::string &topic_name)
    {
        return (MapEditNode->get_publishers_info_by_topic(topic_name).size() > 0);
    }

} // namespace map_edit

PLUGINLIB_EXPORT_CLASS(map_edit::MapEditPanel, rviz_common::Panel)
