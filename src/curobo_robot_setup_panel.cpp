#include "curobo_robot_setup/curobo_robot_setup_panel.hpp"
#include "curobo_robot_setup/collision_config_dialog.hpp"
#include "ui_curobo_robot_setup.h"

#include <QFileDialog>
#include <QVBoxLayout>
#include <QMessageBox>
#include <fstream>
#include <sstream>

namespace curobo_robot_setup
{

CuRoboSetupPanel::CuRoboSetupPanel(QWidget* parent)
  : rviz_common::Panel(parent)
  , ui_(new Ui::Dialog)
{
  // Fix Bug #4: Don't call rclcpp::init() in RViz plugin context
  // ROS2 is already initialized by RViz2, just create the node
  node_ = rclcpp::Node::make_shared("curobo_robot_setup_panel");

  // Publisher pour robot_description
  urdf_publisher_ = node_->create_publisher<std_msgs::msg::String>(
    "robot_description", 
    rclcpp::QoS(1).transient_local()
  );

  // Timer pour publier périodiquement
  publish_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(1000),
    [this]() {
      if (!current_urdf_string_.empty()) {
        publishUrdf();
      }
    }
  );

  // Créer les managers
  sphere_manager_ = std::make_unique<SphereManager>(node_);
  curobo_config_ = std::make_unique<CuRoboConfig>();

  // Setup UI
  ui_->setupUi(this);
  
  // Ajouter label de status
  status_label_ = new QLabel("Ready to load URDF", ui_->urdf_loader);
  status_label_->setGeometry(10, 150, 380, 30);
  status_label_->setWordWrap(true);
  status_label_->setStyleSheet("QLabel { color: #666; }");

  // Connecter les signaux
  setupConnections();

  RCLCPP_INFO(node_->get_logger(), "CuRobo Robot Setup Panel initialized (MVP 3)");
}

CuRoboSetupPanel::~CuRoboSetupPanel()
{
  delete ui_;
}

void CuRoboSetupPanel::setupConnections()
{
  // Tab 1: URDF Loader
  connect(ui_->pushButton_load_urdf, &QPushButton::clicked, 
          this, &CuRoboSetupPanel::onLoadUrdf);

  // Tab 2: Sphere Editor
  connect(ui_->treeWidget_links, &QTreeWidget::itemClicked,
          this, &CuRoboSetupPanel::onLinkSelected);

  connect(ui_->pushButton_add_sphere, &QPushButton::clicked,
          this, &CuRoboSetupPanel::onAddSphere);
  
  connect(ui_->pushButton_delete_sphere, &QPushButton::clicked,
          this, &CuRoboSetupPanel::onDeleteSphere);

  connect(ui_->treeWidget_spheres, &QTreeWidget::itemClicked,
          this, &CuRoboSetupPanel::onSphereSelected);

  connect(ui_->doubleSpinBox_radius, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &CuRoboSetupPanel::onRadiusChanged);
  
  connect(ui_->doubleSpinBox_x, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &CuRoboSetupPanel::onPositionChanged);
  
  connect(ui_->doubleSpinBox_y, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &CuRoboSetupPanel::onPositionChanged);
  
  connect(ui_->doubleSpinBox_z, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &CuRoboSetupPanel::onPositionChanged);

  // Tab 3: Configuration
  connect(ui_->pushButton_configure_joints, &QPushButton::clicked,
          this, &CuRoboSetupPanel::onConfigureJoints);
  
  connect(ui_->pushButton_save_yaml, &QPushButton::clicked,
          this, &CuRoboSetupPanel::onSaveYaml);
  
  connect(ui_->pushButton_load_yaml, &QPushButton::clicked,
          this, &CuRoboSetupPanel::onLoadYaml);
}

// ============================================================================
// URDF Loading
// ============================================================================

void CuRoboSetupPanel::onLoadUrdf()
{
  QString filename = QFileDialog::getOpenFileName(
    this,
    "Load URDF File",
    QString(),
    "URDF Files (*.urdf *.xacro);;All Files (*)"
  );

  if (filename.isEmpty()) {
    return;
  }

  if (loadUrdfFromFile(filename.toStdString())) {
    updateStatusLabel(
      QString("✓ Successfully loaded: %1").arg(QFileInfo(filename).fileName()), 
      true
    );
    RCLCPP_INFO(node_->get_logger(), "URDF loaded: %s", filename.toStdString().c_str());
    
    // Peupler l'arbre des links
    populateLinksTree();
    
    // Peupler les combo boxes
    populateLinkComboBoxes();
    
    // Initialiser les configs des joints
    initializeJointConfigs();
    
    // Mettre à jour le tab Configuration
    updateConfigTab();
    
    // Basculer vers Sphere Editor
    ui_->tabWidget->setCurrentIndex(1);
  } else {
    updateStatusLabel("✗ Failed to load URDF", false);
  }
}

bool CuRoboSetupPanel::loadUrdfFromFile(const std::string& filepath)
{
  try {
    std::ifstream file(filepath);
    if (!file.is_open()) {
      QMessageBox::critical(this, "Error", 
        QString("Could not open file: %1").arg(QString::fromStdString(filepath)));
      return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    current_urdf_string_ = buffer.str();
    file.close();

    if (current_urdf_string_.empty()) {
      QMessageBox::critical(this, "Error", "URDF file is empty");
      return false;
    }

    // Parser l'URDF
    robot_model_ = std::make_shared<urdf::Model>();
    if (!robot_model_->initString(current_urdf_string_)) {
      QMessageBox::critical(this, "Error", "Failed to parse URDF");
      return false;
    }

    current_urdf_path_ = filepath;
    publishUrdf();

    QString info = QString("Robot: %1\nLinks: %2\nJoints: %3\n\n"
                          "Add RobotModel display in RViz to visualize.")
      .arg(QString::fromStdString(robot_model_->getName()))
      .arg(robot_model_->links_.size())
      .arg(robot_model_->joints_.size());
    
    QMessageBox::information(this, "URDF Loaded", info);
    return true;

  } catch (const std::exception& e) {
    QMessageBox::critical(this, "Error", QString("Exception: %1").arg(e.what()));
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

void CuRoboSetupPanel::updateStatusLabel(const QString& message, bool success)
{
  status_label_->setStyleSheet(success ? 
    "QLabel { color: green; font-weight: bold; }" : 
    "QLabel { color: red; font-weight: bold; }");
  status_label_->setText(message);
}

// ============================================================================
// Links Management
// ============================================================================

void CuRoboSetupPanel::populateLinksTree()
{
  if (!robot_model_) return;

  ui_->treeWidget_links->clear();
  std::string root_link = robot_model_->getRoot()->name;
  buildLinkTreeRecursive(nullptr, root_link);
  ui_->treeWidget_links->expandAll();
}

void CuRoboSetupPanel::buildLinkTreeRecursive(
  QTreeWidgetItem* parent, 
  const std::string& link_name)
{
  QTreeWidgetItem* item = new QTreeWidgetItem();
  item->setText(0, QString::fromStdString(link_name));
  item->setText(1, QString::fromStdString(getLinkType(link_name)));
  item->setData(0, Qt::UserRole, QString::fromStdString(link_name));
  
  if (parent) {
    parent->addChild(item);
  } else {
    ui_->treeWidget_links->addTopLevelItem(item);
  }
  
  for (const auto& joint_pair : robot_model_->joints_) {
    if (joint_pair.second->parent_link_name == link_name) {
      buildLinkTreeRecursive(item, joint_pair.second->child_link_name);
    }
  }
}

std::string CuRoboSetupPanel::getLinkType(const std::string& link_name) const
{
  if (!robot_model_) return "unknown";
  
  if (robot_model_->getRoot()->name == link_name) return "root";
  
  for (const auto& joint_pair : robot_model_->joints_) {
    if (joint_pair.second->parent_link_name == link_name) {
      return "child";
    }
  }
  return "leaf";
}

void CuRoboSetupPanel::populateLinkComboBoxes()
{
  if (!robot_model_) return;

  ui_->comboBox_base_link->clear();
  ui_->comboBox_ee_link->clear();

  for (const auto& link_pair : robot_model_->links_) {
    QString link_name = QString::fromStdString(link_pair.first);
    ui_->comboBox_base_link->addItem(link_name);
    ui_->comboBox_ee_link->addItem(link_name);
  }

  // Set defaults
  ui_->comboBox_base_link->setCurrentText(
    QString::fromStdString(robot_model_->getRoot()->name));
  
  // Find leaf link for EE
  for (const auto& link_pair : robot_model_->links_) {
    std::string link_name = link_pair.first;
    if (getLinkType(link_name) == "leaf") {
      ui_->comboBox_ee_link->setCurrentText(QString::fromStdString(link_name));
      break;
    }
  }
}

// ============================================================================
// Sphere Management
// ============================================================================

void CuRoboSetupPanel::onLinkSelected(QTreeWidgetItem* item, int column)
{
  if (!item) return;
  
  selected_link_ = item->data(0, Qt::UserRole).toString().toStdString();
  ui_->label_selected_link->setText(QString::fromStdString(selected_link_));
}

void CuRoboSetupPanel::onAddSphere()
{
  if (selected_link_.empty()) {
    QMessageBox::warning(this, "No Link Selected", 
      "Please select a link first.");
    return;
  }
  
  double radius = ui_->doubleSpinBox_radius->value();
  Eigen::Vector3d position(
    ui_->doubleSpinBox_x->value(),
    ui_->doubleSpinBox_y->value(),
    ui_->doubleSpinBox_z->value()
  );
  
  sphere_manager_->addSphere(selected_link_, radius, position);
  updateSpheresTree();
  updateConfigTab();
}

void CuRoboSetupPanel::onDeleteSphere()
{
  if (selected_sphere_id_.empty()) {
    QMessageBox::warning(this, "No Sphere Selected", 
      "Please select a sphere first.");
    return;
  }
  
  sphere_manager_->removeSphere(selected_sphere_id_);
  selected_sphere_id_.clear();
  updateSpheresTree();
  updateConfigTab();
}

void CuRoboSetupPanel::onSphereSelected(QTreeWidgetItem* item, int column)
{
  if (!item) return;

  QString sphere_id = item->data(0, Qt::UserRole).toString();
  if (sphere_id.isEmpty()) return;

  selected_sphere_id_ = sphere_id.toStdString();

  const Sphere* sphere = sphere_manager_->getSphere(selected_sphere_id_);
  if (sphere) {
    // Fix Bug: Block signals while updating spinboxes to prevent unwanted updates
    ui_->doubleSpinBox_radius->blockSignals(true);
    ui_->doubleSpinBox_x->blockSignals(true);
    ui_->doubleSpinBox_y->blockSignals(true);
    ui_->doubleSpinBox_z->blockSignals(true);

    ui_->doubleSpinBox_radius->setValue(sphere->radius);
    ui_->doubleSpinBox_x->setValue(sphere->position.x());
    ui_->doubleSpinBox_y->setValue(sphere->position.y());
    ui_->doubleSpinBox_z->setValue(sphere->position.z());

    ui_->doubleSpinBox_radius->blockSignals(false);
    ui_->doubleSpinBox_x->blockSignals(false);
    ui_->doubleSpinBox_y->blockSignals(false);
    ui_->doubleSpinBox_z->blockSignals(false);

    // Auto-select the parent link of the sphere
    if (sphere->parent_link != selected_link_) {
      selected_link_ = sphere->parent_link;
      ui_->label_selected_link->setText(QString::fromStdString(selected_link_));

      // Find and select the link in the tree
      for (int i = 0; i < ui_->treeWidget_links->topLevelItemCount(); ++i) {
        QTreeWidgetItem* topItem = ui_->treeWidget_links->topLevelItem(i);
        selectLinkInTree(topItem, selected_link_);
      }
    }
  }
}

// Helper function to recursively find and select a link in the tree
void CuRoboSetupPanel::selectLinkInTree(QTreeWidgetItem* item, const std::string& link_name)
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

void CuRoboSetupPanel::updateSpheresTree()
{
  ui_->treeWidget_spheres->clear();
  
  auto spheres_by_link = sphere_manager_->getAllSpheresByLink();
  
  for (const auto& pair : spheres_by_link) {
    QTreeWidgetItem* link_item = new QTreeWidgetItem();
    link_item->setText(0, QString("📦 %1").arg(QString::fromStdString(pair.first)));
    link_item->setFont(0, QFont("", -1, QFont::Bold));
    ui_->treeWidget_spheres->addTopLevelItem(link_item);
    
    for (const Sphere& sphere : pair.second) {
      QTreeWidgetItem* sphere_item = new QTreeWidgetItem(link_item);
      sphere_item->setText(0, QString("  ⚫ %1").arg(QString::fromStdString(sphere.id)));
      sphere_item->setText(1, QString::number(sphere.radius, 'f', 4));
      sphere_item->setText(2, QString("(%.3f, %.3f, %.3f)")
        .arg(sphere.position.x(), 0, 'f', 3)
        .arg(sphere.position.y(), 0, 'f', 3)
        .arg(sphere.position.z(), 0, 'f', 3));
      sphere_item->setData(0, Qt::UserRole, QString::fromStdString(sphere.id));
    }
  }
  
  ui_->treeWidget_spheres->expandAll();
}

void CuRoboSetupPanel::onRadiusChanged(double value)
{
  if (!selected_sphere_id_.empty()) {
    const Sphere* sphere = sphere_manager_->getSphere(selected_sphere_id_);
    if (sphere) {
      // Fix Bug: Only allow modification if the selected link matches sphere's parent link
      if (sphere->parent_link != selected_link_) {
        RCLCPP_WARN(node_->get_logger(),
                    "Cannot modify sphere '%s': belongs to link '%s' but '%s' is selected",
                    selected_sphere_id_.c_str(), sphere->parent_link.c_str(), selected_link_.c_str());
        return;
      }

      sphere_manager_->updateSphere(selected_sphere_id_, value, sphere->position);
      updateSpheresTree();
    }
  }
}

void CuRoboSetupPanel::onPositionChanged(double value)
{
  if (!selected_sphere_id_.empty()) {
    const Sphere* sphere = sphere_manager_->getSphere(selected_sphere_id_);
    if (sphere) {
      // Fix Bug: Only allow modification if the selected link matches sphere's parent link
      if (sphere->parent_link != selected_link_) {
        RCLCPP_WARN(node_->get_logger(),
                    "Cannot modify sphere '%s': belongs to link '%s' but '%s' is selected",
                    selected_sphere_id_.c_str(), sphere->parent_link.c_str(), selected_link_.c_str());
        return;
      }

      Eigen::Vector3d new_pos(
        ui_->doubleSpinBox_x->value(),
        ui_->doubleSpinBox_y->value(),
        ui_->doubleSpinBox_z->value()
      );
      sphere_manager_->updateSphere(selected_sphere_id_, sphere->radius, new_pos);
      updateSpheresTree();
    }
  }
}

// ============================================================================
// Configuration
// ============================================================================

void CuRoboSetupPanel::initializeJointConfigs()
{
  if (!robot_model_) return;

  joint_configs_.clear();
  
  auto joint_names = CuRoboConfig::getActiveJointNames(robot_model_);
  for (const auto& joint_name : joint_names) {
    joint_configs_[joint_name] = CuRoboConfig::getDefaultJointConfig(robot_model_, joint_name);
  }
}

void CuRoboSetupPanel::onConfigureJoints()
{
  if (!robot_model_) {
    QMessageBox::warning(this, "No Robot", "Load URDF first");
    return;
  }

  // New interface: Configure self-collision ignore pairs and buffers
  CollisionConfigDialog dialog(robot_model_, collision_config_, this);
  if (dialog.exec() == QDialog::Accepted) {
    updateConfigTab();
    QMessageBox::information(this, "Configuration Updated",
      "Self-collision configuration has been updated.\n"
      "Joint limits will be read directly from the URDF by cuRobo.");
  }
}

void CuRoboSetupPanel::updateConfigTab()
{
  if (!robot_model_) return;

  // Update statistics
  int active_joints = 0;
  for (const auto& pair : joint_configs_) {
    if (pair.second.active) active_joints++;
  }

  // Count collision ignore pairs
  int ignore_pairs_count = 0;
  for (const auto& pair : collision_config_.ignore_pairs) {
    ignore_pairs_count += pair.second.size();
  }

  QString stats = QString("URDF: %1\nLinks: %2\nJoints: %3\nSpheres: %4")
    .arg(QString::fromStdString(robot_model_->getName()))
    .arg(robot_model_->links_.size())
    .arg(robot_model_->joints_.size())
    .arg(sphere_manager_->getTotalSphereCount());

  ui_->label_stats->setText(stats);

  ui_->label_joints_summary->setText(
    QString("%1 active joints | %2 collision ignore pairs configured")
    .arg(active_joints)
    .arg(ignore_pairs_count));
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

  // Get configuration
  std::string base_link = ui_->comboBox_base_link->currentText().toStdString();
  std::string ee_link = ui_->comboBox_ee_link->currentText().toStdString();
  auto spheres_by_link = sphere_manager_->getAllSpheresByLink();

  // Validate
  std::string error;
  if (!curobo_config_->validateConfig(robot_model_, base_link, ee_link, joint_configs_, error)) {
    QMessageBox::critical(this, "Validation Error", QString::fromStdString(error));
    return;
  }

  // Save
  if (curobo_config_->saveToYaml(filename.toStdString(), current_urdf_path_,
                                  base_link, ee_link, spheres_by_link, joint_configs_,
                                  cspace_config_, collision_config_)) {
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

  // Load
  std::string urdf_path, base_link, ee_link;
  std::map<std::string, std::vector<Sphere>> spheres_by_link;
  std::map<std::string, JointConfig> loaded_configs;
  CSpaceConfig loaded_cspace_config;
  SelfCollisionConfig loaded_collision_config;

  if (!curobo_config_->loadFromYaml(filename.toStdString(), urdf_path,
                                     base_link, ee_link, spheres_by_link, loaded_configs,
                                     loaded_cspace_config, loaded_collision_config)) {
    QMessageBox::critical(this, "Error", "Failed to load YAML");
    return;
  }

  // Load URDF if specified and different
  if (!urdf_path.empty() && urdf_path != current_urdf_path_) {
    if (QMessageBox::question(this, "Load URDF", 
        QString("Load URDF from:\n%1").arg(QString::fromStdString(urdf_path))) 
        == QMessageBox::Yes) {
      loadUrdfFromFile(urdf_path);
      populateLinksTree();
      populateLinkComboBoxes();
    }
  }

  // Clear and load spheres
  sphere_manager_->clear();
  for (const auto& pair : spheres_by_link) {
    for (const auto& sphere : pair.second) {
      sphere_manager_->addSphere(sphere.parent_link, sphere.radius, sphere.position);
    }
  }
  updateSpheresTree();

  // Load configs
  joint_configs_ = loaded_configs;
  cspace_config_ = loaded_cspace_config;
  collision_config_ = loaded_collision_config;

  // Set base/ee links
  ui_->comboBox_base_link->setCurrentText(QString::fromStdString(base_link));
  ui_->comboBox_ee_link->setCurrentText(QString::fromStdString(ee_link));

  updateConfigTab();
  
  QMessageBox::information(this, "Success", "Configuration loaded successfully");
}

// ============================================================================
// RViz Config
// ============================================================================

void CuRoboSetupPanel::load(const rviz_common::Config& config)
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