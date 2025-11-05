#ifndef COLLISION_CONFIG_DIALOG_HPP
#define COLLISION_CONFIG_DIALOG_HPP

#include <QDialog>
#include <QTableWidget>
#include <QPushButton>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QListWidget>
#include <map>
#include <set>
#include <memory>

#include "curobo_robot_setup/curobo_config.hpp"
#include <urdf/model.h>

namespace curobo_robot_setup
{

/**
 * Dialog for configuring self-collision parameters
 * Allows user to define which link pairs should ignore collision
 * and set collision buffers for each link
 */
class CollisionConfigDialog : public QDialog
{
  Q_OBJECT

public:
  explicit CollisionConfigDialog(
    const std::shared_ptr<urdf::Model>& robot_model,
    SelfCollisionConfig& collision_config,
    QWidget* parent = nullptr
  );

  ~CollisionConfigDialog() = default;

private Q_SLOTS:
  void onAccept();
  void onReject();
  void onLinkSelected();
  void onAddIgnorePair();
  void onRemoveIgnorePair();
  void onBufferChanged(double value);
  void onGenerateAdjacentPairs();
  void onClearAllIgnores();

private:
  void setupUI();
  void populateLinkList();
  void updateIgnoreList();
  void updateBufferValue();
  void saveConfiguration();

  // Get list of all links in the robot
  std::vector<std::string> getAllLinks() const;

  // Generate ignore pairs for adjacent links automatically
  std::map<std::string, std::vector<std::string>> generateAdjacentIgnorePairs() const;

  // Check if two links are adjacent in the kinematic chain
  bool areLinksAdjacent(const std::string& link1, const std::string& link2) const;

  std::shared_ptr<urdf::Model> robot_model_;
  SelfCollisionConfig& collision_config_;

  // Current selected link
  std::string selected_link_;

  // UI Components
  QListWidget* link_list_;
  QListWidget* available_links_list_;
  QListWidget* ignored_links_list_;
  QDoubleSpinBox* buffer_spinbox_;

  QPushButton* add_button_;
  QPushButton* remove_button_;
  QPushButton* generate_adjacent_button_;
  QPushButton* clear_all_button_;
  QPushButton* ok_button_;
  QPushButton* cancel_button_;
};

}  // namespace curobo_robot_setup

#endif  // COLLISION_CONFIG_DIALOG_HPP
