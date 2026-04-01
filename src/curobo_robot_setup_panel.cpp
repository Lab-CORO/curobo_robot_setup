#include "curobo_robot_setup/curobo_robot_setup_panel.hpp"
#include "curobo_robot_setup/collision_config_dialog.hpp"
#include "ui_curobo_robot_setup.h"

#include <QFileDialog>
#include <QFileInfo>
#include <QFont>
#include <QMessageBox>
#include <QMetaObject>
#include <QVBoxLayout>
#include <fstream>
#include <sstream>

#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>

namespace curobo_robot_setup
{

// ══════════════════════════════════════════════════════════════════════════════
// Construction / Destruction
// ══════════════════════════════════════════════════════════════════════════════

CuRoboSetupPanel::CuRoboSetupPanel(QWidget * parent)
: rviz_common::Panel(parent)
, ui_(new Ui::Dialog)
{
  curobo_config_ = std::make_unique<CuRoboConfig>();

  ui_->setupUi(this);

  status_label_ = new QLabel("Ready to load URDF", ui_->urdf_loader);
  status_label_->setGeometry(10, 150, 380, 30);
  status_label_->setWordWrap(true);
  status_label_->setStyleSheet("QLabel { color: #666; }");

  setupConnections();
}

CuRoboSetupPanel::~CuRoboSetupPanel()
{
  delete ui_;
}

// ──────────────────────────────────────────────────────────────────────────────
void CuRoboSetupPanel::setupConnections()
{
  // Tab 1
  connect(ui_->pushButton_load_urdf, &QPushButton::clicked,
    this, &CuRoboSetupPanel::onLoadUrdf);

  // Tab 2 – tree / spinboxes
  connect(ui_->treeWidget_links, &QTreeWidget::itemClicked,
    this, &CuRoboSetupPanel::onLinkSelected);
  connect(ui_->pushButton_add_sphere, &QPushButton::clicked,
    this, &CuRoboSetupPanel::onAddSphere);
  connect(ui_->pushButton_delete_sphere, &QPushButton::clicked,
    this, &CuRoboSetupPanel::onDeleteSphere);
  connect(ui_->treeWidget_spheres, &QTreeWidget::itemClicked,
    this, &CuRoboSetupPanel::onSphereSelected);
  connect(ui_->doubleSpinBox_radius,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this, &CuRoboSetupPanel::onRadiusChanged);
  connect(ui_->doubleSpinBox_x,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this, &CuRoboSetupPanel::onPositionChanged);
  connect(ui_->doubleSpinBox_y,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this, &CuRoboSetupPanel::onPositionChanged);
  connect(ui_->doubleSpinBox_z,
    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this, &CuRoboSetupPanel::onPositionChanged);

  // Tab 2 – auto-generate
  connect(ui_->pushButton_gen_link, &QPushButton::clicked,
    this, &CuRoboSetupPanel::onGenerateLink);
  connect(ui_->pushButton_gen_all, &QPushButton::clicked,
    this, &CuRoboSetupPanel::onGenerateAll);

  // Tab 3
  connect(ui_->pushButton_configure_joints, &QPushButton::clicked,
    this, &CuRoboSetupPanel::onConfigureJoints);
  connect(ui_->pushButton_save_yaml, &QPushButton::clicked,
    this, &CuRoboSetupPanel::onSaveYaml);
  connect(ui_->pushButton_load_yaml, &QPushButton::clicked,
    this, &CuRoboSetupPanel::onLoadYaml);
}

// ══════════════════════════════════════════════════════════════════════════════
// URDF Loading
// ══════════════════════════════════════════════════════════════════════════════

void CuRoboSetupPanel::onLoadUrdf()
{
  QString filename = QFileDialog::getOpenFileName(
    this, "Load URDF File", QString(),
    "URDF Files (*.urdf *.xacro);;All Files (*)");

  if (filename.isEmpty()) return;

  if (loadUrdfFromFile(filename.toStdString())) {
    updateStatusLabel(
      QString("✓ Loaded: %1").arg(QFileInfo(filename).fileName()), true);
    RCLCPP_INFO(node_->get_logger(), "URDF loaded: %s",
      filename.toStdString().c_str());

    populateLinksTree();
    populateLinkComboBoxes();
    initializeJointConfigs();
    updateConfigTab();
    ui_->tabWidget->setCurrentIndex(1);
  } else {
    updateStatusLabel("✗ Failed to load URDF", false);
  }
}

bool CuRoboSetupPanel::loadUrdfFromFile(const std::string & filepath)
{
  try {
    std::ifstream file(filepath);
    if (!file.is_open()) {
      QMessageBox::critical(this, "Error",
        QString("Could not open file: %1").arg(QString::fromStdString(filepath)));
      return false;
    }

    std::stringstream buf;
    buf << file.rdbuf();
    current_urdf_string_ = buf.str();
    file.close();

    if (current_urdf_string_.empty()) {
      QMessageBox::critical(this, "Error", "URDF file is empty");
      return false;
    }

    robot_model_ = std::make_shared<urdf::Model>();
    if (!robot_model_->initString(current_urdf_string_)) {
      QMessageBox::critical(this, "Error", "Failed to parse URDF");
      return false;
    }

    current_urdf_path_ = filepath;
    publishUrdf();

    QMessageBox::information(this, "URDF Loaded",
      QString("Robot: %1\nLinks: %2\nJoints: %3\n\n"
              "Add RobotModel display in RViz to visualize.")
        .arg(QString::fromStdString(robot_model_->getName()))
        .arg(robot_model_->links_.size())
        .arg(robot_model_->joints_.size()));
    return true;

  } catch (const std::exception & e) {
    QMessageBox::critical(this, "Error",
      QString("Exception: %1").arg(e.what()));
    return false;
  }
}

void CuRoboSetupPanel::publishUrdf()
{
  if (current_urdf_string_.empty()) return;
  auto msg = std_msgs::msg::String();
  msg.data = current_urdf_string_;
  urdf_publisher_->publish(msg);
}

void CuRoboSetupPanel::updateStatusLabel(const QString & message, bool success)
{
  status_label_->setStyleSheet(success
    ? "QLabel { color: green; font-weight: bold; }"
    : "QLabel { color: red; font-weight: bold; }");
  status_label_->setText(message);
}

// ══════════════════════════════════════════════════════════════════════════════
// Links
// ══════════════════════════════════════════════════════════════════════════════

void CuRoboSetupPanel::populateLinksTree()
{
  if (!robot_model_) return;
  ui_->treeWidget_links->clear();
  buildLinkTreeRecursive(nullptr, robot_model_->getRoot()->name);
  ui_->treeWidget_links->expandAll();
}

void CuRoboSetupPanel::buildLinkTreeRecursive(
  QTreeWidgetItem * parent, const std::string & link_name)
{
  auto * item = new QTreeWidgetItem();
  item->setText(0, QString::fromStdString(link_name));
  item->setText(1, QString::fromStdString(getLinkType(link_name)));
  item->setData(0, Qt::UserRole, QString::fromStdString(link_name));

  if (parent) {
    parent->addChild(item);
  } else {
    ui_->treeWidget_links->addTopLevelItem(item);
  }

  for (const auto & jt : robot_model_->joints_) {
    if (jt.second->parent_link_name == link_name) {
      buildLinkTreeRecursive(item, jt.second->child_link_name);
    }
  }
}

std::string CuRoboSetupPanel::getLinkType(const std::string & link_name) const
{
  if (!robot_model_) return "unknown";
  if (robot_model_->getRoot()->name == link_name) return "root";
  for (const auto & jt : robot_model_->joints_) {
    if (jt.second->parent_link_name == link_name) return "child";
  }
  return "leaf";
}

void CuRoboSetupPanel::populateLinkComboBoxes()
{
  if (!robot_model_) return;
  ui_->comboBox_base_link->clear();
  ui_->comboBox_ee_link->clear();

  for (const auto & lk : robot_model_->links_) {
    ui_->comboBox_base_link->addItem(QString::fromStdString(lk.first));
    ui_->comboBox_ee_link->addItem(QString::fromStdString(lk.first));
  }

  ui_->comboBox_base_link->setCurrentText(
    QString::fromStdString(robot_model_->getRoot()->name));

  for (const auto & lk : robot_model_->links_) {
    if (getLinkType(lk.first) == "leaf") {
      ui_->comboBox_ee_link->setCurrentText(QString::fromStdString(lk.first));
      break;
    }
  }
}

void CuRoboSetupPanel::onLinkSelected(QTreeWidgetItem * item, int /*column*/)
{
  if (!item) return;
  selected_link_ = item->data(0, Qt::UserRole).toString().toStdString();
  ui_->label_selected_link->setText(QString::fromStdString(selected_link_));
}

void CuRoboSetupPanel::selectLinkInTree(
  QTreeWidgetItem * item, const std::string & link_name)
{
  if (!item) return;
  if (item->data(0, Qt::UserRole).toString().toStdString() == link_name) {
    ui_->treeWidget_links->setCurrentItem(item);
    return;
  }
  for (int i = 0; i < item->childCount(); ++i) {
    selectLinkInTree(item->child(i), link_name);
  }
}

// ══════════════════════════════════════════════════════════════════════════════
// Sphere Management
// ══════════════════════════════════════════════════════════════════════════════

void CuRoboSetupPanel::onAddSphere()
{
  if (selected_link_.empty()) {
    QMessageBox::warning(this, "No Link Selected", "Please select a link first.");
    return;
  }

  Eigen::Vector3d pos(
    ui_->doubleSpinBox_x->value(),
    ui_->doubleSpinBox_y->value(),
    ui_->doubleSpinBox_z->value());

  sphere_manager_->addSphere(selected_link_, ui_->doubleSpinBox_radius->value(), pos);
  updateSpheresTree();
  updateConfigTab();
}

void CuRoboSetupPanel::onDeleteSphere()
{
  if (selected_sphere_id_.empty()) {
    QMessageBox::warning(this, "No Sphere Selected", "Please select a sphere first.");
    return;
  }

  sphere_manager_->setSelectedSphere("");
  sphere_manager_->removeSphere(selected_sphere_id_);
  selected_sphere_id_.clear();
  updateSpheresTree();
  updateConfigTab();
}

void CuRoboSetupPanel::onSphereSelected(QTreeWidgetItem * item, int /*column*/)
{
  if (!item) return;
  QString id_str = item->data(0, Qt::UserRole).toString();
  if (id_str.isEmpty()) return;

  selected_sphere_id_ = id_str.toStdString();
  sphere_manager_->setSelectedSphere(selected_sphere_id_);

  const Sphere * sphere = sphere_manager_->getSphere(selected_sphere_id_);
  if (!sphere) return;

  {
    QSignalBlocker b1(ui_->doubleSpinBox_radius);
    QSignalBlocker b2(ui_->doubleSpinBox_x);
    QSignalBlocker b3(ui_->doubleSpinBox_y);
    QSignalBlocker b4(ui_->doubleSpinBox_z);
    ui_->doubleSpinBox_radius->setValue(sphere->radius);
    ui_->doubleSpinBox_x->setValue(sphere->position.x());
    ui_->doubleSpinBox_y->setValue(sphere->position.y());
    ui_->doubleSpinBox_z->setValue(sphere->position.z());
  }

  if (sphere->parent_link != selected_link_) {
    selected_link_ = sphere->parent_link;
    ui_->label_selected_link->setText(QString::fromStdString(selected_link_));
    for (int i = 0; i < ui_->treeWidget_links->topLevelItemCount(); ++i) {
      selectLinkInTree(ui_->treeWidget_links->topLevelItem(i), selected_link_);
    }
  }
}

void CuRoboSetupPanel::updateSpheresTree()
{
  ui_->treeWidget_spheres->clear();

  for (const auto & pair : sphere_manager_->getAllSpheresByLink()) {
    auto * link_item = new QTreeWidgetItem();
    link_item->setText(0, QString("📦 %1").arg(
      QString::fromStdString(pair.first)));
    link_item->setFont(0, QFont("", -1, QFont::Bold));
    ui_->treeWidget_spheres->addTopLevelItem(link_item);

    for (const Sphere & s : pair.second) {
      auto * si = new QTreeWidgetItem(link_item);
      si->setText(0, QString("  ⚫ %1").arg(QString::fromStdString(s.id)));
      si->setText(1, QString::number(s.radius, 'f', 4));
      si->setText(2, QString("(%.3f, %.3f, %.3f)")
        .arg(s.position.x(), 0, 'f', 3)
        .arg(s.position.y(), 0, 'f', 3)
        .arg(s.position.z(), 0, 'f', 3));
      si->setData(0, Qt::UserRole, QString::fromStdString(s.id));
    }
  }

  ui_->treeWidget_spheres->expandAll();
}

void CuRoboSetupPanel::onRadiusChanged(double value)
{
  if (selected_sphere_id_.empty()) return;
  const Sphere * s = sphere_manager_->getSphere(selected_sphere_id_);
  if (!s || s->parent_link != selected_link_) return;
  sphere_manager_->updateSphere(selected_sphere_id_, value, s->position);
  updateSpheresTree();
}

void CuRoboSetupPanel::onPositionChanged(double /*value*/)
{
  if (selected_sphere_id_.empty()) return;
  const Sphere * s = sphere_manager_->getSphere(selected_sphere_id_);
  if (!s || s->parent_link != selected_link_) return;

  Eigen::Vector3d new_pos(
    ui_->doubleSpinBox_x->value(),
    ui_->doubleSpinBox_y->value(),
    ui_->doubleSpinBox_z->value());
  sphere_manager_->updateSphere(selected_sphere_id_, s->radius, new_pos);
  updateSpheresTree();
}

// ══════════════════════════════════════════════════════════════════════════════
// Auto-Generate Spheres
// ══════════════════════════════════════════════════════════════════════════════

void CuRoboSetupPanel::setGenStatus(const QString & msg, bool ok)
{
  ui_->label_gen_status->setStyleSheet(ok
    ? "QLabel { color: green; }"
    : "QLabel { color: red; }");
  ui_->label_gen_status->setText(msg);
}

void CuRoboSetupPanel::onGenerateLink()
{
  if (!robot_model_) {
    QMessageBox::warning(this, "No Robot", "Load URDF first.");
    return;
  }
  if (selected_link_.empty()) {
    QMessageBox::warning(this, "No Link", "Select a link first.");
    return;
  }
  generateSpheresForLink(selected_link_);
}

void CuRoboSetupPanel::onGenerateAll()
{
  if (!robot_model_) {
    QMessageBox::warning(this, "No Robot", "Load URDF first.");
    return;
  }

  // Collect all link names
  std::vector<std::string> links;
  links.reserve(robot_model_->links_.size());
  for (const auto & lk : robot_model_->links_) {
    links.push_back(lk.first);
  }

  setGenStatus("Generating spheres for all links…");
  for (const auto & lk : links) {
    generateSpheresForLink(lk);
  }
}

void CuRoboSetupPanel::generateSpheresForLink(const std::string & link_name)
{
  if (current_urdf_path_.empty()) {
    setGenStatus("Save/load the URDF to a file first.", false);
    return;
  }

  if (!gen_spheres_client_->service_is_ready()) {
    setGenStatus("Service 'generate_spheres' not available.\n"
                 "Start: ros2 run curobo_robot_setup sphere_fit_service.py", false);
    return;
  }

  auto req = std::make_shared<srv::GenerateSpheres::Request>();
  req->urdf_path = current_urdf_path_;
  req->link_name = link_name;
  req->n_spheres = ui_->spinBox_n_spheres->value();
  req->surface_sphere_radius = ui_->doubleSpinBox_surface_radius->value();
  req->fit_type = ui_->comboBox_fit_type->currentText().toStdString();

  setGenStatus(QString("Generating for '%1'…").arg(
    QString::fromStdString(link_name)));

  gen_spheres_client_->async_send_request(
    req,
    [this, link_name](
      rclcpp::Client<srv::GenerateSpheres>::SharedFuture future)
    {
      auto resp = future.get();

      QMetaObject::invokeMethod(
        this,
        [this, link_name, resp]() {
          if (!resp->success) {
            // Silently skip links without a collision mesh
            RCLCPP_DEBUG(node_->get_logger(),
              "generate_spheres skipped '%s': %s",
              link_name.c_str(), resp->message.c_str());
            return;
          }

          // Replace existing spheres for this link
          for (const auto & s : sphere_manager_->getSpheresByLink(link_name)) {
            sphere_manager_->removeSphere(s.id);
          }

          for (size_t i = 0; i < resp->positions_x.size(); ++i) {
            Eigen::Vector3d pos(
              resp->positions_x[i],
              resp->positions_y[i],
              resp->positions_z[i]);
            sphere_manager_->addSphere(link_name, resp->radii[i], pos);
          }

          updateSpheresTree();
          updateConfigTab();

          setGenStatus(
            QString("✓ '%1': %2 sphères générées")
              .arg(QString::fromStdString(link_name))
              .arg(resp->positions_x.size()),
            true);
        },
        Qt::QueuedConnection);
    });
}

// ══════════════════════════════════════════════════════════════════════════════
// Configuration
// ══════════════════════════════════════════════════════════════════════════════

void CuRoboSetupPanel::initializeJointConfigs()
{
  if (!robot_model_) return;
  joint_configs_.clear();
  for (const auto & jn : CuRoboConfig::getActiveJointNames(robot_model_)) {
    joint_configs_[jn] = CuRoboConfig::getDefaultJointConfig(robot_model_, jn);
  }
}

void CuRoboSetupPanel::onConfigureJoints()
{
  if (!robot_model_) {
    QMessageBox::warning(this, "No Robot", "Load URDF first");
    return;
  }
  CollisionConfigDialog dialog(robot_model_, collision_config_, this);
  if (dialog.exec() == QDialog::Accepted) {
    updateConfigTab();
    QMessageBox::information(this, "Configuration Updated",
      "Self-collision configuration updated.\n"
      "Joint limits are read from URDF by cuRobo.");
  }
}

void CuRoboSetupPanel::updateConfigTab()
{
  if (!robot_model_) return;

  int active_joints = 0;
  for (const auto & p : joint_configs_) {
    if (p.second.active) active_joints++;
  }

  int ignore_pairs = 0;
  for (const auto & p : collision_config_.ignore_pairs) {
    ignore_pairs += static_cast<int>(p.second.size());
  }

  ui_->label_stats->setText(
    QString("URDF: %1\nLinks: %2\nJoints: %3\nSphères: %4")
      .arg(QString::fromStdString(robot_model_->getName()))
      .arg(robot_model_->links_.size())
      .arg(robot_model_->joints_.size())
      .arg(sphere_manager_->getTotalSphereCount()));

  ui_->label_joints_summary->setText(
    QString("%1 joints actifs | %2 paires collision ignorées")
      .arg(active_joints).arg(ignore_pairs));
}

void CuRoboSetupPanel::onSaveYaml()
{
  if (!robot_model_) {
    QMessageBox::warning(this, "No Robot", "Load URDF first");
    return;
  }

  QString filename = QFileDialog::getSaveFileName(
    this, "Save cuRobo Configuration", "", "YAML Files (*.yml *.yaml)");
  if (filename.isEmpty()) return;

  std::string base_link = ui_->comboBox_base_link->currentText().toStdString();
  std::string ee_link   = ui_->comboBox_ee_link->currentText().toStdString();
  auto spheres_by_link  = sphere_manager_->getAllSpheresByLink();

  std::string error;
  if (!curobo_config_->validateConfig(
      robot_model_, base_link, ee_link, joint_configs_, error))
  {
    QMessageBox::critical(this, "Validation Error", QString::fromStdString(error));
    return;
  }

  if (curobo_config_->saveToYaml(filename.toStdString(), current_urdf_path_,
        base_link, ee_link, spheres_by_link, joint_configs_,
        cspace_config_, collision_config_))
  {
    QMessageBox::information(this, "Success",
      QString("Configuration saved to:\n%1").arg(filename));
  } else {
    QMessageBox::critical(this, "Error", "Failed to save YAML");
  }
}

void CuRoboSetupPanel::onLoadYaml()
{
  QString filename = QFileDialog::getOpenFileName(
    this, "Load cuRobo Configuration", "", "YAML Files (*.yml *.yaml)");
  if (filename.isEmpty()) return;

  std::string urdf_path, base_link, ee_link;
  std::map<std::string, std::vector<Sphere>> spheres_by_link;
  std::map<std::string, JointConfig> loaded_joints;
  CSpaceConfig loaded_cspace;
  SelfCollisionConfig loaded_collision;

  if (!curobo_config_->loadFromYaml(filename.toStdString(), urdf_path,
        base_link, ee_link, spheres_by_link, loaded_joints,
        loaded_cspace, loaded_collision))
  {
    QMessageBox::critical(this, "Error", "Failed to load YAML");
    return;
  }

  if (!urdf_path.empty() && urdf_path != current_urdf_path_) {
    if (QMessageBox::question(this, "Load URDF",
          QString("Load URDF from:\n%1").arg(
            QString::fromStdString(urdf_path))) == QMessageBox::Yes)
    {
      loadUrdfFromFile(urdf_path);
      populateLinksTree();
      populateLinkComboBoxes();
    }
  }

  sphere_manager_->clear();
  for (const auto & p : spheres_by_link) {
    for (const auto & s : p.second) {
      sphere_manager_->addSphere(s.parent_link, s.radius, s.position);
    }
  }
  updateSpheresTree();

  joint_configs_    = loaded_joints;
  cspace_config_    = loaded_cspace;
  collision_config_ = loaded_collision;

  ui_->comboBox_base_link->setCurrentText(QString::fromStdString(base_link));
  ui_->comboBox_ee_link->setCurrentText(QString::fromStdString(ee_link));

  updateConfigTab();
  QMessageBox::information(this, "Success", "Configuration loaded successfully");
}

// ══════════════════════════════════════════════════════════════════════════════
// RViz Panel lifecycle
// ══════════════════════════════════════════════════════════════════════════════

void CuRoboSetupPanel::onInitialize()
{
  Panel::onInitialize();

  // Use RViz2's own node — it is already spinning in RViz2's executor
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // URDF publisher
  urdf_publisher_ = node_->create_publisher<std_msgs::msg::String>(
    "robot_description",
    rclcpp::QoS(1).transient_local());

  publish_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(1000),
    [this]() {
      if (!current_urdf_string_.empty()) {
        publishUrdf();
      }
    });

  // Service client for sphere auto-generation
  gen_spheres_client_ = node_->create_client<srv::GenerateSpheres>("generate_spheres");

  // Sphere manager (interactive markers)
  sphere_manager_ = std::make_unique<SphereManager>(node_);

  // Callback fired on RViz2's executor thread — marshal to Qt main thread
  sphere_manager_->setOnMovedCallback(
    [this](const std::string & id, const Eigen::Vector3d & new_pos) {
      QMetaObject::invokeMethod(
        this,
        [this, id, new_pos]() {
          if (selected_sphere_id_ == id) {
            QSignalBlocker bx(ui_->doubleSpinBox_x);
            QSignalBlocker by(ui_->doubleSpinBox_y);
            QSignalBlocker bz(ui_->doubleSpinBox_z);
            ui_->doubleSpinBox_x->setValue(new_pos.x());
            ui_->doubleSpinBox_y->setValue(new_pos.y());
            ui_->doubleSpinBox_z->setValue(new_pos.z());
          }
          updateSpheresTree();
          updateConfigTab();
        },
        Qt::QueuedConnection);
    });

  RCLCPP_INFO(node_->get_logger(), "CuRobo Robot Setup Panel initialized");

  ensureIMDisplay();
}

void CuRoboSetupPanel::ensureIMDisplay()
{
  auto * root = getDisplayContext()->getRootDisplayGroup();
  const QString class_id = "rviz_default_plugins/InteractiveMarkers";

  for (int i = 0; i < root->numDisplays(); ++i) {
    if (root->getDisplayAt(i)->getClassId() == class_id) {
      return;  // already present
    }
  }

  auto * display = root->createDisplay(class_id);
  if (!display) {
    RCLCPP_WARN(node_->get_logger(),
      "Could not create InteractiveMarkers display automatically");
    return;
  }
  display->initialize(getDisplayContext());
  display->setTopic(
    "/collision_spheres/update",
    "visualization_msgs/msg/InteractiveMarkerUpdate");
  display->setEnabled(true);

  RCLCPP_INFO(node_->get_logger(),
    "Auto-created InteractiveMarkers display on /collision_spheres/update");
}

void CuRoboSetupPanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
}

void CuRoboSetupPanel::save(rviz_common::Config config) const
{
  Panel::save(config);
}

}  // namespace curobo_robot_setup

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(curobo_robot_setup::CuRoboSetupPanel, rviz_common::Panel)
