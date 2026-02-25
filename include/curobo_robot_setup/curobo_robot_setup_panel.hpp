#ifndef CUROBO_ROBOT_SETUP_PANEL_HPP
#define CUROBO_ROBOT_SETUP_PANEL_HPP

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <urdf/model.h>

#include "curobo_robot_setup/sphere_manager.hpp"
#include "curobo_robot_setup/curobo_config.hpp"
#include "curobo_robot_setup/srv/generate_spheres.hpp"

#include <QLabel>
#include <QTabWidget>
#include <QTreeWidget>
#include <QWidget>

namespace Ui {
class Dialog;
}

namespace curobo_robot_setup
{

class CuRoboSetupPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit CuRoboSetupPanel(QWidget * parent = nullptr);
  ~CuRoboSetupPanel() override;

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  // Tab 1 – URDF
  void onLoadUrdf();

  // Tab 2 – Sphere Editor
  void onLinkSelected(QTreeWidgetItem * item, int column);
  void onAddSphere();
  void onDeleteSphere();
  void onSphereSelected(QTreeWidgetItem * item, int column);
  void onRadiusChanged(double value);
  void onPositionChanged(double value);

  // Auto-generate slots
  void onGenerateLink();
  void onGenerateAll();

  // Tab 3 – Configuration
  void onConfigureJoints();
  void onSaveYaml();
  void onLoadYaml();

private:
  void setupConnections();

  // URDF
  bool loadUrdfFromFile(const std::string & filepath);
  void publishUrdf();
  void updateStatusLabel(const QString & message, bool success = true);

  // Links
  void populateLinksTree();
  void buildLinkTreeRecursive(QTreeWidgetItem * parent, const std::string & link_name);
  std::string getLinkType(const std::string & link_name) const;
  void populateLinkComboBoxes();
  void selectLinkInTree(QTreeWidgetItem * item, const std::string & link_name);

  // Spheres
  void updateSpheresTree();

  // Auto-generate
  void generateSpheresForLink(const std::string & link_name);
  void setGenStatus(const QString & msg, bool ok = true);

  // Configuration
  void updateConfigTab();
  void initializeJointConfigs();

  // ── Members ───────────────────────────────────────────────────────────────
  Ui::Dialog * ui_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr urdf_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  rclcpp::Client<srv::GenerateSpheres>::SharedPtr gen_spheres_client_;

  std::unique_ptr<SphereManager> sphere_manager_;
  std::unique_ptr<CuRoboConfig> curobo_config_;

  std::shared_ptr<urdf::Model> robot_model_;
  std::string current_urdf_string_;
  std::string current_urdf_path_;
  std::string selected_link_;
  std::string selected_sphere_id_;

  std::map<std::string, JointConfig> joint_configs_;
  CSpaceConfig cspace_config_;
  SelfCollisionConfig collision_config_;

  QLabel * status_label_;
};

}  // namespace curobo_robot_setup

#endif  // CUROBO_ROBOT_SETUP_PANEL_HPP
