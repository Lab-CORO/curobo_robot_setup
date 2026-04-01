#include "curobo_robot_setup/sphere_manager.hpp"

#include <visualization_msgs/msg/interactive_marker_control.hpp>

namespace curobo_robot_setup
{

using namespace visualization_msgs::msg;  // NOLINT

// ──────────────────────────────────────────────────────────────────────────────
SphereManager::SphereManager(rclcpp::Node::SharedPtr node)
: node_(node)
{
  im_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "collision_spheres",
    node_
  );

  RCLCPP_INFO(node_->get_logger(), "SphereManager: interactive marker server ready");
}

// ──────────────────────────────────────────────────────────────────────────────
std::string SphereManager::generateSphereId()
{
  return "sphere_" + std::to_string(next_sphere_id_++);
}

// ──────────────────────────────────────────────────────────────────────────────
std::string SphereManager::addSphere(
  const std::string & parent_link,
  double radius,
  const Eigen::Vector3d & position)
{
  if (radius <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
      "Invalid radius %.4f for link '%s'", radius, parent_link.c_str());
    return "";
  }

  Sphere sphere(parent_link, radius, position);
  {
    std::lock_guard<std::mutex> lock(spheres_mutex_);
    sphere.id = generateSphereId();
    spheres_[sphere.id] = sphere;
  }

  insertMarker(sphere);

  RCLCPP_INFO(node_->get_logger(),
    "Added sphere '%s' to link '%s' (r=%.4f, pos=[%.3f, %.3f, %.3f])",
    sphere.id.c_str(), parent_link.c_str(), radius,
    position.x(), position.y(), position.z());

  return sphere.id;
}

// ──────────────────────────────────────────────────────────────────────────────
bool SphereManager::removeSphere(const std::string & sphere_id)
{
  {
    std::lock_guard<std::mutex> lock(spheres_mutex_);
    auto it = spheres_.find(sphere_id);
    if (it == spheres_.end()) {
      RCLCPP_WARN(node_->get_logger(), "Sphere '%s' not found", sphere_id.c_str());
      return false;
    }
    spheres_.erase(it);
  }

  im_server_->erase(sphere_id);
  im_server_->applyChanges();

  RCLCPP_INFO(node_->get_logger(), "Removed sphere '%s'", sphere_id.c_str());
  return true;
}

// ──────────────────────────────────────────────────────────────────────────────
bool SphereManager::updateSphere(
  const std::string & sphere_id,
  double radius,
  const Eigen::Vector3d & position)
{
  Sphere updated;
  {
    std::lock_guard<std::mutex> lock(spheres_mutex_);
    auto it = spheres_.find(sphere_id);
    if (it == spheres_.end()) {
      RCLCPP_WARN(node_->get_logger(), "Sphere '%s' not found", sphere_id.c_str());
      return false;
    }
    it->second.radius = radius;
    it->second.position = position;
    updated = it->second;
  }

  // Re-insert the interactive marker with updated data
  insertMarker(updated);

  RCLCPP_DEBUG(node_->get_logger(),
    "Updated sphere '%s' (r=%.4f, pos=[%.3f, %.3f, %.3f])",
    sphere_id.c_str(), radius,
    position.x(), position.y(), position.z());

  return true;
}

// ──────────────────────────────────────────────────────────────────────────────
void SphereManager::setSelectedSphere(const std::string & sphere_id)
{
  std::string prev_id;
  {
    std::lock_guard<std::mutex> lock(spheres_mutex_);
    prev_id = selected_sphere_id_;
    selected_sphere_id_ = sphere_id;
  }

  // Rebuild previous selected sphere without controls
  if (!prev_id.empty() && prev_id != sphere_id) {
    std::lock_guard<std::mutex> lock(spheres_mutex_);
    auto it = spheres_.find(prev_id);
    if (it != spheres_.end()) {
      auto im = buildInteractiveMarker(it->second, false);
      im_server_->insert(im);
      im_server_->setCallback(
        prev_id,
        std::bind(&SphereManager::processFeedback, this, std::placeholders::_1));
    }
  }

  // Rebuild new selected sphere with controls
  if (!sphere_id.empty()) {
    std::lock_guard<std::mutex> lock(spheres_mutex_);
    auto it = spheres_.find(sphere_id);
    if (it != spheres_.end()) {
      auto im = buildInteractiveMarker(it->second, true);
      im_server_->insert(im);
      im_server_->setCallback(
        sphere_id,
        std::bind(&SphereManager::processFeedback, this, std::placeholders::_1));
    }
  }

  im_server_->applyChanges();
}

// ──────────────────────────────────────────────────────────────────────────────
void SphereManager::clear()
{
  {
    std::lock_guard<std::mutex> lock(spheres_mutex_);
    spheres_.clear();
    selected_sphere_id_.clear();
    next_sphere_id_ = 0;
  }

  im_server_->clear();
  im_server_->applyChanges();

  RCLCPP_INFO(node_->get_logger(), "Cleared all spheres");
}

// ──────────────────────────────────────────────────────────────────────────────
const Sphere * SphereManager::getSphere(const std::string & sphere_id) const
{
  std::lock_guard<std::mutex> lock(spheres_mutex_);
  auto it = spheres_.find(sphere_id);
  return (it != spheres_.end()) ? &it->second : nullptr;
}

std::vector<Sphere> SphereManager::getSpheresByLink(const std::string & link) const
{
  std::lock_guard<std::mutex> lock(spheres_mutex_);
  std::vector<Sphere> result;
  for (const auto & kv : spheres_) {
    if (kv.second.parent_link == link) {
      result.push_back(kv.second);
    }
  }
  return result;
}

std::map<std::string, std::vector<Sphere>> SphereManager::getAllSpheresByLink() const
{
  std::lock_guard<std::mutex> lock(spheres_mutex_);
  std::map<std::string, std::vector<Sphere>> result;
  for (const auto & kv : spheres_) {
    result[kv.second.parent_link].push_back(kv.second);
  }
  return result;
}

std::vector<Sphere> SphereManager::getAllSpheres() const
{
  std::lock_guard<std::mutex> lock(spheres_mutex_);
  std::vector<Sphere> result;
  result.reserve(spheres_.size());
  for (const auto & kv : spheres_) {
    result.push_back(kv.second);
  }
  return result;
}

size_t SphereManager::getTotalSphereCount() const
{
  std::lock_guard<std::mutex> lock(spheres_mutex_);
  return spheres_.size();
}

// ──────────────────────────────────────────────────────────────────────────────
void SphereManager::setMarkerColor(float r, float g, float b, float a)
{
  marker_color_r_ = r;
  marker_color_g_ = g;
  marker_color_b_ = b;
  marker_color_a_ = a;

  // Rebuild all markers with new color
  std::vector<Sphere> all;
  {
    std::lock_guard<std::mutex> lock(spheres_mutex_);
    for (const auto & kv : spheres_) {
      all.push_back(kv.second);
    }
  }
  for (const auto & s : all) {
    insertMarker(s);  // insert is an upsert
  }
}

// ──────────────────────────────────────────────────────────────────────────────
// Private helpers
// ──────────────────────────────────────────────────────────────────────────────

void SphereManager::insertMarker(const Sphere & sphere)
{
  bool selected;
  {
    std::lock_guard<std::mutex> lock(spheres_mutex_);
    selected = (sphere.id == selected_sphere_id_);
  }
  auto im = buildInteractiveMarker(sphere, selected);
  im_server_->insert(im);
  im_server_->setCallback(
    sphere.id,
    std::bind(&SphereManager::processFeedback, this, std::placeholders::_1));
  im_server_->applyChanges();
}

// ──────────────────────────────────────────────────────────────────────────────
InteractiveMarker SphereManager::buildInteractiveMarker(
  const Sphere & sphere, bool selected) const
{
  InteractiveMarker im;
  im.header.frame_id    = sphere.parent_link;
  im.header.stamp       = rclcpp::Time(0);
  im.name               = sphere.id;
  im.description        = "";
  im.scale              = static_cast<float>(std::max(sphere.radius * 8.0, 0.1));
  im.pose.position.x    = sphere.position.x();
  im.pose.position.y    = sphere.position.y();
  im.pose.position.z    = sphere.position.z();
  im.pose.orientation.w = 1.0;

  // ── Contrôle 0 : sphère visuelle ─────────────────────────────────────────
  // Sélectionnée → MOVE_3D (Shift+Ctrl+drag) + alpha plein
  // Non sélectionnée → NONE (visible uniquement) + alpha réduit
  {
    auto sphere_visual = buildSphereVisual(sphere);
    sphere_visual.color.a = selected ? marker_color_a_ : marker_color_a_ * 0.35f;

    InteractiveMarkerControl ctrl;
    ctrl.name             = "move_3d";
    ctrl.always_visible   = true;
    ctrl.orientation.w    = 1.0;
    ctrl.interaction_mode = selected
      ? InteractiveMarkerControl::MOVE_3D
      : InteractiveMarkerControl::NONE;
    ctrl.markers.push_back(sphere_visual);
    im.controls.push_back(ctrl);
  }

  if (!selected) {
    return im;  // pas de flèches pour les sphères non sélectionnées
  }

  // ── Contrôles 1-3 : flèches MOVE_AXIS (clic-glisser normal) ───────────────
  // ctrl.orientation : axe de déplacement (local X du contrôle = axe monde)
  //   X → quat 90° autour X : (K,K,0,0) → local X = monde X
  //   Y → quat 90° autour Z : (K,0,0,K) → local X = monde Y
  //   Z → quat 90° autour Y : (K,0,K,0) → local X = monde -Z (bidirectionnel ⇒ OK)
  //
  // arrow.pose.orientation : orientation visuelle de la flèche dans le frame du marker
  //   ARROW pointe par défaut le long de son X local.
  //   Les markers sont dans le frame du marker (pas du contrôle), donc :
  //   X → identité
  //   Y → 90° autour Z : (K,0,0,K)  → local X → monde Y
  //   Z → -90° autour Y : (K,0,-K,0) → local X → monde +Z

  static const float K = 0.70711f;

  struct AxisDef {
    const char * name;
    float qw, qx, qy, qz;          // orientation du contrôle
    float r, g, b;                  // couleur
    float aw, ax, ay, az;           // orientation visuelle de la flèche
  };
  static const AxisDef axes[3] = {
    {"move_x", K, K,  0,  0,  1.0f, 0.0f, 0.0f,  1.f,  0.f,  0.f,  0.f},
    {"move_y", K, 0,  0,  K,  0.0f, 1.0f, 0.0f,  K,    0.f,  0.f,  K  },
    {"move_z", K, 0,  K,  0,  0.0f, 0.0f, 1.0f,  K,    0.f, -K,    0.f},
  };

  for (const auto & ax : axes) {
    InteractiveMarkerControl ctrl;
    ctrl.name             = ax.name;
    ctrl.always_visible   = true;
    ctrl.orientation_mode = InteractiveMarkerControl::FIXED;
    ctrl.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    ctrl.orientation.w    = ax.qw;
    ctrl.orientation.x    = ax.qx;
    ctrl.orientation.y    = ax.qy;
    ctrl.orientation.z    = ax.qz;

    Marker arrow;
    arrow.type    = Marker::ARROW;
    arrow.action  = Marker::ADD;
    arrow.scale.x = sphere.radius * 6.0;   // longueur
    arrow.scale.y = sphere.radius * 0.4;   // diamètre fût
    arrow.scale.z = sphere.radius * 0.6;   // diamètre tête
    arrow.color.r = ax.r;
    arrow.color.g = ax.g;
    arrow.color.b = ax.b;
    arrow.color.a = 0.9f;
    arrow.pose.orientation.w = ax.aw;
    arrow.pose.orientation.x = ax.ax;
    arrow.pose.orientation.y = ax.ay;
    arrow.pose.orientation.z = ax.az;
    ctrl.markers.push_back(arrow);
    im.controls.push_back(ctrl);
  }

  return im;
}

// ──────────────────────────────────────────────────────────────────────────────
Marker SphereManager::buildSphereVisual(const Sphere & sphere) const
{
  Marker m;
  m.type = Marker::SPHERE;
  m.action = Marker::ADD;
  m.scale.x = sphere.radius * 2.0;
  m.scale.y = sphere.radius * 2.0;
  m.scale.z = sphere.radius * 2.0;
  m.color.r = marker_color_r_;
  m.color.g = marker_color_g_;
  m.color.b = marker_color_b_;
  m.color.a = marker_color_a_;
  m.pose.orientation.w = 1.0;
  return m;
}

// ──────────────────────────────────────────────────────────────────────────────
void SphereManager::processFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & fb)
{
  if (fb->event_type !=
      visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
  {
    return;
  }

  Eigen::Vector3d new_pos(
    fb->pose.position.x,
    fb->pose.position.y,
    fb->pose.position.z);

  // Update internal data (runs on ROS2 executor thread)
  {
    std::lock_guard<std::mutex> lock(spheres_mutex_);
    auto it = spheres_.find(fb->marker_name);
    if (it == spheres_.end()) return;
    it->second.position = new_pos;
  }

  // Notify Qt panel (it will use QueuedConnection to marshal to main thread)
  if (on_moved_callback_) {
    on_moved_callback_(fb->marker_name, new_pos);
  }
}

}  // namespace curobo_robot_setup
