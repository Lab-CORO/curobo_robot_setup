#include "curobo_robot_setup/curobo_robot_setup_panel.hpp"
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
  // Initialiser ROS2 node
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
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

  // Créer le Sphere Manager
  sphere_manager_ = std::make_unique<SphereManager>(node_);

  // Setup UI
  ui_->setupUi(this);
  
  // Ajouter label de status dans le tab URDF loader (si layout existe)
  status_label_ = new QLabel("Ready to load URDF", ui_->urdf_loader);
  status_label_->setGeometry(10, 150, 340, 30);
  status_label_->setWordWrap(true);
  status_label_->setStyleSheet("QLabel { color: #666; }");

  // Connecter les signaux
  setupConnections();

  RCLCPP_INFO(node_->get_logger(), "CuRobo Robot Setup Panel initialized");
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

  // Tab 2: Sphere Editor - Link selection
  connect(ui_->treeWidget_links, &QTreeWidget::itemClicked,
          this, &CuRoboSetupPanel::onLinkSelected);

  // Sphere actions
  connect(ui_->pushButton_add_sphere, &QPushButton::clicked,
          this, &CuRoboSetupPanel::onAddSphere);
  
  connect(ui_->pushButton_delete_sphere, &QPushButton::clicked,
          this, &CuRoboSetupPanel::onDeleteSphere);

  // Sphere selection
  connect(ui_->treeWidget_spheres, &QTreeWidget::itemClicked,
          this, &CuRoboSetupPanel::onSphereSelected);

  // Sphere properties changed
  connect(ui_->doubleSpinBox_radius, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &CuRoboSetupPanel::onRadiusChanged);
  
  connect(ui_->doubleSpinBox_x, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &CuRoboSetupPanel::onPositionChanged);
  
  connect(ui_->doubleSpinBox_y, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &CuRoboSetupPanel::onPositionChanged);
  
  connect(ui_->doubleSpinBox_z, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &CuRoboSetupPanel::onPositionChanged);
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
    
    // Basculer vers l'onglet Sphere Editor
    ui_->tabWidget->setCurrentIndex(1);
  } else {
    updateStatusLabel("✗ Failed to load URDF", false);
    RCLCPP_ERROR(node_->get_logger(), "Failed to load URDF: %s", 
                 filename.toStdString().c_str());
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
      QMessageBox::critical(this, "Error",
        "Failed to parse URDF. Check file format.");
      return false;
    }

    current_urdf_path_ = filepath;

    // Publier l'URDF
    publishUrdf();

    // Afficher info
    QString info = QString("Robot: %1\nLinks: %2\nJoints: %3\n\n"
                          "Now add a RobotModel display in RViz to visualize the robot.")
      .arg(QString::fromStdString(robot_model_->getName()))
      .arg(robot_model_->links_.size())
      .arg(robot_model_->joints_.size());
    
    QMessageBox::information(this, "URDF Loaded", info);

    return true;

  } catch (const std::exception& e) {
    QMessageBox::critical(this, "Error", 
      QString("Exception while loading URDF: %1").arg(e.what()));
    RCLCPP_ERROR(node_->get_logger(), "Exception: %s", e.what());
    return false;
  }
}

void CuRoboSetupPanel::publishUrdf()
{
  if (current_urdf_string_.empty()) {
    return;
  }

  auto msg = std_msgs::msg::String();
  msg.data = current_urdf_string_;
  urdf_publisher_->publish(msg);

  RCLCPP_DEBUG(node_->get_logger(), "Published robot_description");
}

void CuRoboSetupPanel::updateStatusLabel(const QString& message, bool success)
{
  if (success) {
    status_label_->setStyleSheet("QLabel { color: green; font-weight: bold; }");
  } else {
    status_label_->setStyleSheet("QLabel { color: red; font-weight: bold; }");
  }
  status_label_->setText(message);
}

// ============================================================================
// Links Tree
// ============================================================================

void CuRoboSetupPanel::populateLinksTree()
{
  if (!robot_model_) {
    return;
  }

  ui_->treeWidget_links->clear();
  
  // Trouver le lien racine
  std::string root_link = robot_model_->getRoot()->name;
  
  // Construire l'arbre récursivement
  buildLinkTreeRecursive(nullptr, root_link);
  
  ui_->treeWidget_links->expandAll();
  
  RCLCPP_INFO(node_->get_logger(), "Populated links tree with %zu links", 
              robot_model_->links_.size());
}

void CuRoboSetupPanel::buildLinkTreeRecursive(
  QTreeWidgetItem* parent, 
  const std::string& link_name)
{
  // Créer l'item pour ce link
  QTreeWidgetItem* item = new QTreeWidgetItem();
  item->setText(0, QString::fromStdString(link_name));
  item->setText(1, QString::fromStdString(getLinkType(link_name)));
  item->setData(0, Qt::UserRole, QString::fromStdString(link_name));  // Stocker le nom
  
  if (parent) {
    parent->addChild(item);
  } else {
    ui_->treeWidget_links->addTopLevelItem(item);
  }
  
  // Trouver les enfants de ce link
  for (const auto& joint_pair : robot_model_->joints_) {
    const auto& joint = joint_pair.second;
    if (joint->parent_link_name == link_name) {
      // Ce joint part du link actuel
      buildLinkTreeRecursive(item, joint->child_link_name);
    }
  }
}

std::string CuRoboSetupPanel::getLinkType(const std::string& link_name) const
{
  if (!robot_model_) {
    return "unknown";
  }
  
  // Vérifier si c'est la racine
  if (robot_model_->getRoot()->name == link_name) {
    return "root";
  }
  
  // Vérifier si c'est une feuille (pas de joint enfant)
  bool has_children = false;
  for (const auto& joint_pair : robot_model_->joints_) {
    if (joint_pair.second->parent_link_name == link_name) {
      has_children = true;
      break;
    }
  }
  
  return has_children ? "child" : "leaf";
}

// ============================================================================
// Link Selection
// ============================================================================

void CuRoboSetupPanel::onLinkSelected(QTreeWidgetItem* item, int column)
{
  if (!item) {
    return;
  }
  
  selected_link_ = item->data(0, Qt::UserRole).toString().toStdString();
  
  ui_->label_selected_link->setText(QString::fromStdString(selected_link_));
  ui_->label_selected_link->setStyleSheet("QLabel { font-weight: bold; color: #2196F3; }");
  
  RCLCPP_INFO(node_->get_logger(), "Selected link: %s", selected_link_.c_str());
}

// ============================================================================
// Sphere Management
// ============================================================================

void CuRoboSetupPanel::onAddSphere()
{
  if (selected_link_.empty()) {
    QMessageBox::warning(this, "No Link Selected", 
      "Please select a link in the tree first.");
    return;
  }
  
  // Récupérer les propriétés depuis l'UI
  double radius = ui_->doubleSpinBox_radius->value();
  Eigen::Vector3d position(
    ui_->doubleSpinBox_x->value(),
    ui_->doubleSpinBox_y->value(),
    ui_->doubleSpinBox_z->value()
  );
  
  // Ajouter la sphère
  std::string sphere_id = sphere_manager_->addSphere(selected_link_, radius, position);
  
  // Mettre à jour l'arbre des sphères
  updateSpheresTree();
  
  RCLCPP_INFO(node_->get_logger(), 
              "Added sphere '%s' to link '%s'", 
              sphere_id.c_str(), selected_link_.c_str());
}

void CuRoboSetupPanel::onDeleteSphere()
{
  if (selected_sphere_id_.empty()) {
    QMessageBox::warning(this, "No Sphere Selected", 
      "Please select a sphere in the list first.");
    return;
  }
  
  sphere_manager_->removeSphere(selected_sphere_id_);
  
  selected_sphere_id_.clear();
  
  // Mettre à jour l'arbre
  updateSpheresTree();
  
  RCLCPP_INFO(node_->get_logger(), "Deleted sphere");
}

void CuRoboSetupPanel::onSphereSelected(QTreeWidgetItem* item, int column)
{
  if (!item) {
    return;
  }
  
  // Vérifier si c'est un item de sphère (pas un link parent)
  QString sphere_id = item->data(0, Qt::UserRole).toString();
  
  if (!sphere_id.isEmpty()) {
    selected_sphere_id_ = sphere_id.toStdString();
    
    // Charger les propriétés de la sphère dans l'UI
    const Sphere* sphere = sphere_manager_->getSphere(selected_sphere_id_);
    if (sphere) {
      ui_->doubleSpinBox_radius->setValue(sphere->radius);
      ui_->doubleSpinBox_x->setValue(sphere->position.x());
      ui_->doubleSpinBox_y->setValue(sphere->position.y());
      ui_->doubleSpinBox_z->setValue(sphere->position.z());
      
      RCLCPP_INFO(node_->get_logger(), "Selected sphere: %s", selected_sphere_id_.c_str());
    }
  }
}

void CuRoboSetupPanel::updateSpheresTree()
{
  ui_->treeWidget_spheres->clear();
  
  // Obtenir toutes les sphères groupées par link
  auto spheres_by_link = sphere_manager_->getAllSpheresByLink();
  
  // Créer un item pour chaque link qui a des sphères
  for (const auto& pair : spheres_by_link) {
    const std::string& link_name = pair.first;
    const std::vector<Sphere>& spheres = pair.second;
    
    // Item parent pour le link
    QTreeWidgetItem* link_item = new QTreeWidgetItem();
    link_item->setText(0, QString("📦 %1").arg(QString::fromStdString(link_name)));
    link_item->setFont(0, QFont("", -1, QFont::Bold));
    ui_->treeWidget_spheres->addTopLevelItem(link_item);
    
    // Items enfants pour chaque sphère
    for (const Sphere& sphere : spheres) {
      QTreeWidgetItem* sphere_item = new QTreeWidgetItem(link_item);
      sphere_item->setText(0, QString("  ⚫ %1").arg(QString::fromStdString(sphere.id)));
      sphere_item->setText(1, QString::number(sphere.radius, 'f', 4));
      sphere_item->setText(2, QString("(%.3f, %.3f, %.3f)")
        .arg(sphere.position.x(), 0, 'f', 3)
        .arg(sphere.position.y(), 0, 'f', 3)
        .arg(sphere.position.z(), 0, 'f', 3));
      
      // Stocker l'ID de la sphère
      sphere_item->setData(0, Qt::UserRole, QString::fromStdString(sphere.id));
    }
  }
  
  ui_->treeWidget_spheres->expandAll();
  
  RCLCPP_INFO(node_->get_logger(), 
              "Updated spheres tree: %zu links, %zu total spheres",
              spheres_by_link.size(), 
              sphere_manager_->getTotalSphereCount());
}

// ============================================================================
// Sphere Properties
// ============================================================================

void CuRoboSetupPanel::onRadiusChanged(double value)
{
  // Si une sphère est sélectionnée, la mettre à jour
  if (!selected_sphere_id_.empty()) {
    const Sphere* sphere = sphere_manager_->getSphere(selected_sphere_id_);
    if (sphere) {
      sphere_manager_->updateSphere(
        selected_sphere_id_,
        value,
        sphere->position
      );
      updateSpheresTree();
    }
  }
}

void CuRoboSetupPanel::onPositionChanged(double value)
{
  // Si une sphère est sélectionnée, la mettre à jour
  if (!selected_sphere_id_.empty()) {
    const Sphere* sphere = sphere_manager_->getSphere(selected_sphere_id_);
    if (sphere) {
      Eigen::Vector3d new_position(
        ui_->doubleSpinBox_x->value(),
        ui_->doubleSpinBox_y->value(),
        ui_->doubleSpinBox_z->value()
      );
      
      sphere_manager_->updateSphere(
        selected_sphere_id_,
        sphere->radius,
        new_position
      );
      updateSpheresTree();
    }
  }
}

// ============================================================================
// RViz Config
// ============================================================================

void CuRoboSetupPanel::load(const rviz_common::Config& config)
{
  Panel::load(config);
  
  QString urdf_path;
  if (config.mapGetString("urdf_path", &urdf_path)) {
    current_urdf_path_ = urdf_path.toStdString();
    if (!current_urdf_path_.empty()) {
      if (loadUrdfFromFile(current_urdf_path_)) {
        populateLinksTree();
      }
    }
  }
}

void CuRoboSetupPanel::save(rviz_common::Config config) const
{
  Panel::save(config);
  config.mapSetValue("urdf_path", QString::fromStdString(current_urdf_path_));
}

}  // namespace curobo_robot_setup

// Export du plugin pour RViz2
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(curobo_robot_setup::CuRoboSetupPanel, rviz_common::Panel)