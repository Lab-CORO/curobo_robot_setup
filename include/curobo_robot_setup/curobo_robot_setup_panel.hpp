#ifndef CUROBO_ROBOT_SETUP_PANEL_HPP
#define CUROBO_ROBOT_SETUP_PANEL_HPP

#include <memory>
#include <string>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <urdf/model.h>

#include "curobo_robot_setup/sphere_manager.hpp"

#include <QWidget>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QTreeWidget>
#include <QLabel>
#include <QTabWidget>

// Classe générée depuis le fichier .ui
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
  // URDF Tab
  void onLoadUrdf();
  
  // Sphere Editor Tab
  void onLinkSelected(QTreeWidgetItem* item, int column);
  void onAddSphere();
  void onDeleteSphere();
  void onSphereSelected(QTreeWidgetItem* item, int column);
  
  // Sphere properties changed
  void onRadiusChanged(double value);
  void onPositionChanged(double value);

private:
  void setupConnections();
  
  // URDF
  bool loadUrdfFromFile(const std::string& filepath);
  void publishUrdf();
  void updateStatusLabel(const QString& message, bool success = true);
  
  // Links tree
  void populateLinksTree();
  void buildLinkTreeRecursive(QTreeWidgetItem* parent, 
                              const std::string& link_name);
  std::string getLinkType(const std::string& link_name) const;
  
  // Spheres tree
  void updateSpheresTree();
  
  // UI générée par Qt Designer
  Ui::Dialog* ui_;

  // ROS2
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr urdf_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Sphere Manager
  std::unique_ptr<SphereManager> sphere_manager_;

  // Données
  std::shared_ptr<urdf::Model> robot_model_;
  std::string current_urdf_string_;
  std::string current_urdf_path_;
  std::string selected_link_;
  std::string selected_sphere_id_;
  
  // Status
  QLabel* status_label_;
};

}  // namespace curobo_robot_setup

#endif  // CUROBO_ROBOT_SETUP_PANEL_HPP