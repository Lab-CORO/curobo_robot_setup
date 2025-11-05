#ifndef CUROBO_ROBOT_SETUP_PANEL_HPP
#define CUROBO_ROBOT_SETUP_PANEL_HPP

#include <memory>
#include <string>
#include <map>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <urdf/model.h>

#include "curobo_robot_setup/sphere_manager.hpp"
#include "curobo_robot_setup/curobo_config.hpp"

#include <QWidget>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QTreeWidget>
#include <QLabel>
#include <QTabWidget>
#include <QComboBox>

// Forward declaration pour la classe UI générée par Qt
namespace Ui {
class Dialog;
}

namespace curobo_robot_setup
{

class CuRoboSetupPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit CuRoboSetupPanel(QWidget* parent = nullptr);
  virtual ~CuRoboSetupPanel();

  // Méthodes RViz Panel
  virtual void load(const rviz_common::Config& config) override;
  virtual void save(rviz_common::Config config) const override;

private Q_SLOTS:
  // Tab 1: URDF
  void onLoadUrdf();
  
  // Tab 2: Sphere Editor
  void onLinkSelected(QTreeWidgetItem* item, int column);
  void onAddSphere();
  void onDeleteSphere();
  void onSphereSelected(QTreeWidgetItem* item, int column);
  void onRadiusChanged(double value);
  void onPositionChanged(double value);
  
  // Tab 3: Configuration
  void onConfigureJoints();
  void onSaveYaml();
  void onLoadYaml();

private:
  void setupConnections();

  // URDF
  bool loadUrdfFromFile(const std::string& filepath);
  void publishUrdf();
  void updateStatusLabel(const QString& message, bool success = true);

  // Links
  void populateLinksTree();
  void buildLinkTreeRecursive(QTreeWidgetItem* parent, const std::string& link_name);
  std::string getLinkType(const std::string& link_name) const;
  void populateLinkComboBoxes();
  void selectLinkInTree(QTreeWidgetItem* item, const std::string& link_name);

  // Spheres
  void updateSpheresTree();

  // Configuration
  void updateConfigTab();
  void initializeJointConfigs();
  
  // UI générée par Qt Designer
  Ui::Dialog* ui_;

  // ROS2
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr urdf_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Managers
  std::unique_ptr<SphereManager> sphere_manager_;
  std::unique_ptr<CuRoboConfig> curobo_config_;

  // Données
  std::shared_ptr<urdf::Model> robot_model_;
  std::string current_urdf_string_;
  std::string current_urdf_path_;
  std::string selected_link_;
  std::string selected_sphere_id_;

  std::map<std::string, JointConfig> joint_configs_;
  CSpaceConfig cspace_config_;
  SelfCollisionConfig collision_config_;

  // Status
  QLabel* status_label_;
};

}  // namespace curobo_robot_setup

#endif  // CUROBO_ROBOT_SETUP_PANEL_HPP