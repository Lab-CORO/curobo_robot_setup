#include "curobo_robot_setup/joint_config_dialog.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QMessageBox>

namespace curobo_robot_setup
{

JointConfigDialog::JointConfigDialog(
  const std::shared_ptr<urdf::Model>& robot_model,
  std::map<std::string, JointConfig>& joint_configs,
  QWidget* parent)
  : QDialog(parent)
  , robot_model_(robot_model)
  , joint_configs_(joint_configs)
{
  setWindowTitle("Joint Configuration");
  resize(800, 500);
  
  setupUI();
  populateTable();
}

void JointConfigDialog::setupUI()
{
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  
  // Info label
  QLabel* info_label = new QLabel(
    "Configure joint limits and properties. Check 'Active' to include joints in cuRobo configuration."
  );
  info_label->setWordWrap(true);
  main_layout->addWidget(info_label);
  
  // Buttons at top
  QHBoxLayout* top_buttons = new QHBoxLayout();
  
  import_button_ = new QPushButton("Import from URDF");
  connect(import_button_, &QPushButton::clicked, this, &JointConfigDialog::onImportFromUrdf);
  top_buttons->addWidget(import_button_);
  
  select_all_button_ = new QPushButton("Select All");
  connect(select_all_button_, &QPushButton::clicked, this, &JointConfigDialog::onSelectAll);
  top_buttons->addWidget(select_all_button_);
  
  deselect_all_button_ = new QPushButton("Deselect All");
  connect(deselect_all_button_, &QPushButton::clicked, this, &JointConfigDialog::onDeselectAll);
  top_buttons->addWidget(deselect_all_button_);
  
  top_buttons->addStretch();
  main_layout->addLayout(top_buttons);
  
  // Table
  table_ = new QTableWidget(this);
  table_->setColumnCount(7);
  table_->setHorizontalHeaderLabels({
    "Active", "Joint Name", "Pos Min", "Pos Max", "Vel Max", "Acc Max", "Jerk Max"
  });
  
  table_->horizontalHeader()->setStretchLastSection(false);
  table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  table_->setAlternatingRowColors(true);
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  
  connect(table_, &QTableWidget::cellChanged, this, &JointConfigDialog::onCellChanged);
  
  main_layout->addWidget(table_);
  
  // Bottom buttons
  QHBoxLayout* bottom_buttons = new QHBoxLayout();
  bottom_buttons->addStretch();
  
  cancel_button_ = new QPushButton("Cancel");
  connect(cancel_button_, &QPushButton::clicked, this, &JointConfigDialog::onReject);
  bottom_buttons->addWidget(cancel_button_);
  
  ok_button_ = new QPushButton("OK");
  ok_button_->setDefault(true);
  connect(ok_button_, &QPushButton::clicked, this, &JointConfigDialog::onAccept);
  bottom_buttons->addWidget(ok_button_);
  
  main_layout->addLayout(bottom_buttons);
}

void JointConfigDialog::populateTable()
{
  if (!robot_model_) {
    return;
  }
  
  // Get all non-fixed joints
  std::vector<std::string> joint_names;
  for (const auto& pair : robot_model_->joints_) {
    if (pair.second->type != urdf::Joint::FIXED) {
      joint_names.push_back(pair.first);
    }
  }
  
  table_->setRowCount(joint_names.size());
  
  // Block signals while populating
  table_->blockSignals(true);
  
  for (size_t i = 0; i < joint_names.size(); ++i) {
    const std::string& joint_name = joint_names[i];
    
    // Get or create joint config
    JointConfig config;
    if (joint_configs_.find(joint_name) != joint_configs_.end()) {
      config = joint_configs_[joint_name];
    } else {
      config = CuRoboConfig::getDefaultJointConfig(robot_model_, joint_name);
      joint_configs_[joint_name] = config;
    }
    
    // Column 0: Active checkbox
    QCheckBox* checkbox = new QCheckBox();
    checkbox->setChecked(config.active);
    QWidget* checkbox_widget = new QWidget();
    QHBoxLayout* checkbox_layout = new QHBoxLayout(checkbox_widget);
    checkbox_layout->addWidget(checkbox);
    checkbox_layout->setAlignment(Qt::AlignCenter);
    checkbox_layout->setContentsMargins(0, 0, 0, 0);
    table_->setCellWidget(i, 0, checkbox_widget);
    
    // Column 1: Joint name
    QTableWidgetItem* name_item = new QTableWidgetItem(QString::fromStdString(joint_name));
    name_item->setFlags(name_item->flags() & ~Qt::ItemIsEditable);
    table_->setItem(i, 1, name_item);
    
    // Columns 2-6: Numeric values
    QTableWidgetItem* pos_min_item = new QTableWidgetItem(QString::number(config.pos_min, 'f', 4));
    QTableWidgetItem* pos_max_item = new QTableWidgetItem(QString::number(config.pos_max, 'f', 4));
    QTableWidgetItem* vel_max_item = new QTableWidgetItem(QString::number(config.vel_max, 'f', 4));
    QTableWidgetItem* acc_max_item = new QTableWidgetItem(QString::number(config.acc_max, 'f', 4));
    QTableWidgetItem* jerk_max_item = new QTableWidgetItem(QString::number(config.jerk_max, 'f', 4));
    
    table_->setItem(i, 2, pos_min_item);
    table_->setItem(i, 3, pos_max_item);
    table_->setItem(i, 4, vel_max_item);
    table_->setItem(i, 5, acc_max_item);
    table_->setItem(i, 6, jerk_max_item);
  }
  
  table_->blockSignals(false);
}

void JointConfigDialog::updateJointConfigs()
{
  for (int row = 0; row < table_->rowCount(); ++row) {
    // Fix Bug #3: Check for null pointer before dereferencing
    QTableWidgetItem* name_item = table_->item(row, 1);
    if (!name_item) {
      qWarning() << "Missing joint name item at row" << row;
      continue;
    }
    QString joint_name = name_item->text();

    JointConfig config;

    // Get active state from checkbox
    QWidget* checkbox_widget = table_->cellWidget(row, 0);
    if (checkbox_widget) {
      QCheckBox* checkbox = checkbox_widget->findChild<QCheckBox*>();
      if (checkbox) {
        config.active = checkbox->isChecked();
      }
    }

    // Fix Bug #3: Get numeric values with null checks
    bool ok;

    QTableWidgetItem* pos_min_item = table_->item(row, 2);
    if (pos_min_item) {
      config.pos_min = pos_min_item->text().toDouble(&ok);
    }

    QTableWidgetItem* pos_max_item = table_->item(row, 3);
    if (pos_max_item) {
      config.pos_max = pos_max_item->text().toDouble(&ok);
    }

    QTableWidgetItem* vel_max_item = table_->item(row, 4);
    if (vel_max_item) {
      config.vel_max = vel_max_item->text().toDouble(&ok);
    }

    QTableWidgetItem* acc_max_item = table_->item(row, 5);
    if (acc_max_item) {
      config.acc_max = acc_max_item->text().toDouble(&ok);
    }

    QTableWidgetItem* jerk_max_item = table_->item(row, 6);
    if (jerk_max_item) {
      config.jerk_max = jerk_max_item->text().toDouble(&ok);
    }

    // Keep existing collision ignores if any
    if (joint_configs_.find(joint_name.toStdString()) != joint_configs_.end()) {
      config.ignore_collisions = joint_configs_[joint_name.toStdString()].ignore_collisions;
    }

    joint_configs_[joint_name.toStdString()] = config;
  }
}

void JointConfigDialog::onAccept()
{
  // Validate data
  for (int row = 0; row < table_->rowCount(); ++row) {
    bool ok;
    double pos_min = table_->item(row, 2)->text().toDouble(&ok);
    if (!ok) {
      QMessageBox::warning(this, "Invalid Data", 
        QString("Invalid position min value at row %1").arg(row + 1));
      return;
    }
    
    double pos_max = table_->item(row, 3)->text().toDouble(&ok);
    if (!ok) {
      QMessageBox::warning(this, "Invalid Data", 
        QString("Invalid position max value at row %1").arg(row + 1));
      return;
    }
    
    if (pos_min >= pos_max) {
      QMessageBox::warning(this, "Invalid Data", 
        QString("Position min must be less than position max at row %1").arg(row + 1));
      return;
    }
    
    double vel_max = table_->item(row, 4)->text().toDouble(&ok);
    if (!ok || vel_max <= 0) {
      QMessageBox::warning(this, "Invalid Data", 
        QString("Velocity must be positive at row %1").arg(row + 1));
      return;
    }
    
    double acc_max = table_->item(row, 5)->text().toDouble(&ok);
    if (!ok || acc_max <= 0) {
      QMessageBox::warning(this, "Invalid Data", 
        QString("Acceleration must be positive at row %1").arg(row + 1));
      return;
    }
    
    double jerk_max = table_->item(row, 6)->text().toDouble(&ok);
    if (!ok || jerk_max <= 0) {
      QMessageBox::warning(this, "Invalid Data", 
        QString("Jerk must be positive at row %1").arg(row + 1));
      return;
    }
  }
  
  // Update configs
  updateJointConfigs();
  
  accept();
}

void JointConfigDialog::onReject()
{
  reject();
}

void JointConfigDialog::onSelectAll()
{
  for (int row = 0; row < table_->rowCount(); ++row) {
    QWidget* checkbox_widget = table_->cellWidget(row, 0);
    QCheckBox* checkbox = checkbox_widget->findChild<QCheckBox*>();
    if (checkbox) {
      checkbox->setChecked(true);
    }
  }
}

void JointConfigDialog::onDeselectAll()
{
  for (int row = 0; row < table_->rowCount(); ++row) {
    QWidget* checkbox_widget = table_->cellWidget(row, 0);
    QCheckBox* checkbox = checkbox_widget->findChild<QCheckBox*>();
    if (checkbox) {
      checkbox->setChecked(false);
    }
  }
}

void JointConfigDialog::onImportFromUrdf()
{
  if (!robot_model_) {
    return;
  }
  
  // Re-import all joint configs from URDF
  for (int row = 0; row < table_->rowCount(); ++row) {
    QString joint_name = table_->item(row, 1)->text();
    
    JointConfig config = CuRoboConfig::getDefaultJointConfig(
      robot_model_, 
      joint_name.toStdString()
    );
    
    // Update table
    table_->item(row, 2)->setText(QString::number(config.pos_min, 'f', 4));
    table_->item(row, 3)->setText(QString::number(config.pos_max, 'f', 4));
    table_->item(row, 4)->setText(QString::number(config.vel_max, 'f', 4));
    table_->item(row, 5)->setText(QString::number(config.acc_max, 'f', 4));
    table_->item(row, 6)->setText(QString::number(config.jerk_max, 'f', 4));
  }
  
  QMessageBox::information(this, "Import Complete", 
    "Joint limits imported from URDF");
}

void JointConfigDialog::onCellChanged(int row, int column)
{
  // Real-time validation could go here if needed
}

}  // namespace curobo_robot_setup