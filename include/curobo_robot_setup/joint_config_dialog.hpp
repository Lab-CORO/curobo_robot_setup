#ifndef JOINT_CONFIG_DIALOG_HPP
#define JOINT_CONFIG_DIALOG_HPP

#include <QDialog>
#include <QTableWidget>
#include <QPushButton>
#include <map>
#include <memory>

#include "curobo_robot_setup/curobo_config.hpp"
#include <urdf/model.h>

namespace curobo_robot_setup
{

class JointConfigDialog : public QDialog
{
  Q_OBJECT

public:
  explicit JointConfigDialog(
    const std::shared_ptr<urdf::Model>& robot_model,
    std::map<std::string, JointConfig>& joint_configs,
    QWidget* parent = nullptr
  );
  
  ~JointConfigDialog() = default;

private Q_SLOTS:
  void onAccept();
  void onReject();
  void onSelectAll();
  void onDeselectAll();
  void onImportFromUrdf();
  void onCellChanged(int row, int column);

private:
  void setupUI();
  void populateTable();
  void updateJointConfigs();

  std::shared_ptr<urdf::Model> robot_model_;
  std::map<std::string, JointConfig>& joint_configs_;
  
  QTableWidget* table_;
  QPushButton* ok_button_;
  QPushButton* cancel_button_;
  QPushButton* import_button_;
  QPushButton* select_all_button_;
  QPushButton* deselect_all_button_;
};

}  // namespace curobo_robot_setup

#endif  // JOINT_CONFIG_DIALOG_HPP