#include "curobo_robot_setup/collision_config_dialog.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <algorithm>

namespace curobo_robot_setup
{

CollisionConfigDialog::CollisionConfigDialog(
  const std::shared_ptr<urdf::Model>& robot_model,
  SelfCollisionConfig& collision_config,
  QWidget* parent)
  : QDialog(parent)
  , robot_model_(robot_model)
  , collision_config_(collision_config)
{
  setWindowTitle("Self-Collision Configuration");
  resize(900, 600);

  setupUI();
  populateLinkList();
}

void CollisionConfigDialog::setupUI()
{
  QVBoxLayout* main_layout = new QVBoxLayout(this);

  // Info label
  QLabel* info_label = new QLabel(
    "Configure which link pairs should ignore self-collision and set collision buffers.\n"
    "Tip: Use 'Generate Adjacent Pairs' to automatically ignore adjacent links in the kinematic chain."
  );
  info_label->setWordWrap(true);
  info_label->setStyleSheet("QLabel { background-color: #e3f2fd; padding: 10px; border-radius: 5px; }");
  main_layout->addWidget(info_label);

  // Main content layout
  QHBoxLayout* content_layout = new QHBoxLayout();

  // Left panel: Link selection
  QGroupBox* link_group = new QGroupBox("Robot Links");
  QVBoxLayout* link_layout = new QVBoxLayout();

  link_list_ = new QListWidget();
  link_list_->setSelectionMode(QAbstractItemView::SingleSelection);
  connect(link_list_, &QListWidget::currentTextChanged, this, &CollisionConfigDialog::onLinkSelected);
  link_layout->addWidget(new QLabel("Select a link to configure:"));
  link_layout->addWidget(link_list_);

  link_group->setLayout(link_layout);
  content_layout->addWidget(link_group, 1);

  // Middle panel: Collision ignore configuration
  QGroupBox* ignore_group = new QGroupBox("Collision Ignore List");
  QVBoxLayout* ignore_layout = new QVBoxLayout();

  QLabel* selected_link_label = new QLabel("Available links to ignore:");
  ignore_layout->addWidget(selected_link_label);

  available_links_list_ = new QListWidget();
  available_links_list_->setSelectionMode(QAbstractItemView::MultiSelection);
  ignore_layout->addWidget(available_links_list_);

  add_button_ = new QPushButton("➜ Add to Ignore List");
  connect(add_button_, &QPushButton::clicked, this, &CollisionConfigDialog::onAddIgnorePair);
  ignore_layout->addWidget(add_button_);

  ignore_layout->addWidget(new QLabel("Currently ignored links:"));

  ignored_links_list_ = new QListWidget();
  ignored_links_list_->setSelectionMode(QAbstractItemView::MultiSelection);
  ignore_layout->addWidget(ignored_links_list_);

  remove_button_ = new QPushButton("✖ Remove from Ignore List");
  connect(remove_button_, &QPushButton::clicked, this, &CollisionConfigDialog::onRemoveIgnorePair);
  ignore_layout->addWidget(remove_button_);

  ignore_group->setLayout(ignore_layout);
  content_layout->addWidget(ignore_group, 2);

  // Right panel: Buffer configuration
  QGroupBox* buffer_group = new QGroupBox("Collision Buffer");
  QVBoxLayout* buffer_layout = new QVBoxLayout();

  buffer_layout->addWidget(new QLabel("Self-collision buffer for selected link (meters):"));

  buffer_spinbox_ = new QDoubleSpinBox();
  buffer_spinbox_->setRange(0.0, 0.1);
  buffer_spinbox_->setSingleStep(0.001);
  buffer_spinbox_->setDecimals(4);
  buffer_spinbox_->setValue(0.0);
  connect(buffer_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &CollisionConfigDialog::onBufferChanged);
  buffer_layout->addWidget(buffer_spinbox_);

  buffer_layout->addWidget(new QLabel(
    "\nBuffer adds safety margin to collision checking.\n"
    "0.0 = no extra margin\n"
    "0.005-0.01 = typical values"
  ));

  buffer_layout->addStretch();
  buffer_group->setLayout(buffer_layout);
  content_layout->addWidget(buffer_group, 1);

  main_layout->addLayout(content_layout);

  // Utility buttons
  QHBoxLayout* utility_layout = new QHBoxLayout();

  generate_adjacent_button_ = new QPushButton("🔗 Generate Adjacent Pairs");
  generate_adjacent_button_->setToolTip("Automatically add ignore pairs for adjacent links in kinematic chain");
  connect(generate_adjacent_button_, &QPushButton::clicked, this, &CollisionConfigDialog::onGenerateAdjacentPairs);
  utility_layout->addWidget(generate_adjacent_button_);

  clear_all_button_ = new QPushButton("🗑 Clear All Ignores");
  clear_all_button_->setToolTip("Remove all collision ignore pairs");
  connect(clear_all_button_, &QPushButton::clicked, this, &CollisionConfigDialog::onClearAllIgnores);
  utility_layout->addWidget(clear_all_button_);

  utility_layout->addStretch();
  main_layout->addLayout(utility_layout);

  // Bottom buttons
  QHBoxLayout* button_layout = new QHBoxLayout();
  button_layout->addStretch();

  cancel_button_ = new QPushButton("Cancel");
  connect(cancel_button_, &QPushButton::clicked, this, &CollisionConfigDialog::onReject);
  button_layout->addWidget(cancel_button_);

  ok_button_ = new QPushButton("OK");
  ok_button_->setDefault(true);
  connect(ok_button_, &QPushButton::clicked, this, &CollisionConfigDialog::onAccept);
  button_layout->addWidget(ok_button_);

  main_layout->addLayout(button_layout);
}

void CollisionConfigDialog::populateLinkList()
{
  if (!robot_model_) return;

  link_list_->clear();

  std::vector<std::string> links = getAllLinks();
  for (const auto& link : links) {
    link_list_->addItem(QString::fromStdString(link));
  }

  // Select first link by default
  if (link_list_->count() > 0) {
    link_list_->setCurrentRow(0);
  }
}

std::vector<std::string> CollisionConfigDialog::getAllLinks() const
{
  std::vector<std::string> links;
  if (!robot_model_) return links;

  for (const auto& link_pair : robot_model_->links_) {
    links.push_back(link_pair.first);
  }

  std::sort(links.begin(), links.end());
  return links;
}

void CollisionConfigDialog::onLinkSelected()
{
  QListWidgetItem* item = link_list_->currentItem();
  if (!item) return;

  selected_link_ = item->text().toStdString();
  updateIgnoreList();
  updateBufferValue();
}

void CollisionConfigDialog::updateIgnoreList()
{
  available_links_list_->clear();
  ignored_links_list_->clear();

  if (selected_link_.empty()) return;

  // Get currently ignored links for selected link
  std::set<std::string> ignored_set;
  auto it = collision_config_.ignore_pairs.find(selected_link_);
  if (it != collision_config_.ignore_pairs.end()) {
    ignored_set.insert(it->second.begin(), it->second.end());
  }

  // Populate available and ignored lists
  std::vector<std::string> all_links = getAllLinks();
  for (const auto& link : all_links) {
    // Skip self
    if (link == selected_link_) continue;

    if (ignored_set.count(link) > 0) {
      ignored_links_list_->addItem(QString::fromStdString(link));
    } else {
      available_links_list_->addItem(QString::fromStdString(link));
    }
  }
}

void CollisionConfigDialog::updateBufferValue()
{
  if (selected_link_.empty()) return;

  // Get buffer value for selected link
  auto it = collision_config_.buffer_distances.find(selected_link_);
  if (it != collision_config_.buffer_distances.end()) {
    buffer_spinbox_->setValue(it->second);
  } else {
    buffer_spinbox_->setValue(0.0);
  }
}

void CollisionConfigDialog::onAddIgnorePair()
{
  if (selected_link_.empty()) {
    QMessageBox::warning(this, "No Link Selected", "Please select a link first.");
    return;
  }

  QList<QListWidgetItem*> selected_items = available_links_list_->selectedItems();
  if (selected_items.isEmpty()) {
    QMessageBox::warning(this, "No Links Selected", "Please select one or more links to ignore.");
    return;
  }

  // Add selected links to ignore list
  for (QListWidgetItem* item : selected_items) {
    std::string link_to_ignore = item->text().toStdString();
    collision_config_.ignore_pairs[selected_link_].push_back(link_to_ignore);
  }

  // Refresh display
  updateIgnoreList();
}

void CollisionConfigDialog::onRemoveIgnorePair()
{
  if (selected_link_.empty()) return;

  QList<QListWidgetItem*> selected_items = ignored_links_list_->selectedItems();
  if (selected_items.isEmpty()) {
    QMessageBox::warning(this, "No Links Selected", "Please select one or more links to remove.");
    return;
  }

  // Remove selected links from ignore list
  auto it = collision_config_.ignore_pairs.find(selected_link_);
  if (it != collision_config_.ignore_pairs.end()) {
    for (QListWidgetItem* item : selected_items) {
      std::string link_to_remove = item->text().toStdString();
      auto& ignore_vec = it->second;
      ignore_vec.erase(
        std::remove(ignore_vec.begin(), ignore_vec.end(), link_to_remove),
        ignore_vec.end()
      );
    }

    // Remove entry if empty
    if (it->second.empty()) {
      collision_config_.ignore_pairs.erase(it);
    }
  }

  // Refresh display
  updateIgnoreList();
}

void CollisionConfigDialog::onBufferChanged(double value)
{
  if (selected_link_.empty()) return;

  // Update buffer for selected link
  collision_config_.buffer_distances[selected_link_] = value;
}

void CollisionConfigDialog::onGenerateAdjacentPairs()
{
  auto adjacent_pairs = generateAdjacentIgnorePairs();

  if (adjacent_pairs.empty()) {
    QMessageBox::information(this, "No Adjacent Pairs",
      "No adjacent link pairs found in the kinematic chain.");
    return;
  }

  // Merge with existing ignore pairs
  for (const auto& pair : adjacent_pairs) {
    auto& existing = collision_config_.ignore_pairs[pair.first];
    for (const auto& link : pair.second) {
      // Add only if not already present
      if (std::find(existing.begin(), existing.end(), link) == existing.end()) {
        existing.push_back(link);
      }
    }
  }

  // Refresh display
  updateIgnoreList();

  QMessageBox::information(this, "Adjacent Pairs Generated",
    QString("Added ignore pairs for %1 links.").arg(adjacent_pairs.size()));
}

std::map<std::string, std::vector<std::string>>
CollisionConfigDialog::generateAdjacentIgnorePairs() const
{
  std::map<std::string, std::vector<std::string>> pairs;
  if (!robot_model_) return pairs;

  // For each joint, add ignore pair between parent and child links
  for (const auto& joint_pair : robot_model_->joints_) {
    const auto& joint = joint_pair.second;

    // Skip fixed joints
    if (joint->type == urdf::Joint::FIXED) continue;

    std::string parent_link = joint->parent_link_name;
    std::string child_link = joint->child_link_name;

    // Add bidirectional ignore
    pairs[parent_link].push_back(child_link);
    pairs[child_link].push_back(parent_link);
  }

  return pairs;
}

bool CollisionConfigDialog::areLinksAdjacent(
  const std::string& link1,
  const std::string& link2) const
{
  if (!robot_model_) return false;

  // Check if there's a joint connecting these links
  for (const auto& joint_pair : robot_model_->joints_) {
    const auto& joint = joint_pair.second;

    if ((joint->parent_link_name == link1 && joint->child_link_name == link2) ||
        (joint->parent_link_name == link2 && joint->child_link_name == link1)) {
      return true;
    }
  }

  return false;
}

void CollisionConfigDialog::onClearAllIgnores()
{
  auto reply = QMessageBox::question(this, "Clear All Ignores",
    "Are you sure you want to clear all collision ignore pairs?",
    QMessageBox::Yes | QMessageBox::No);

  if (reply == QMessageBox::Yes) {
    collision_config_.ignore_pairs.clear();
    updateIgnoreList();
  }
}

void CollisionConfigDialog::saveConfiguration()
{
  // Configuration is already saved in collision_config_ reference
  // No additional work needed
}

void CollisionConfigDialog::onAccept()
{
  saveConfiguration();
  accept();
}

void CollisionConfigDialog::onReject()
{
  reject();
}

}  // namespace curobo_robot_setup
