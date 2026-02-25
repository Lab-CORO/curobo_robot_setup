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
void SphereManager::clear()
{
  {
    std::lock_guard<std::mutex> lock(spheres_mutex_);
    spheres_.clear();
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
  auto im = buildInteractiveMarker(sphere);
  im_server_->insert(im);
  im_server_->setCallback(
    sphere.id,
    std::bind(&SphereManager::processFeedback, this, std::placeholders::_1));
  im_server_->applyChanges();
}

// ──────────────────────────────────────────────────────────────────────────────
InteractiveMarker SphereManager::buildInteractiveMarker(const Sphere & sphere) const
{
  InteractiveMarker im;
  im.header.frame_id = sphere.parent_link;
  im.header.stamp = rclcpp::Time(0);
  im.name = sphere.id;
  im.description = "";
  // Control handle scale – at least 3× the sphere so handles are visible
  im.scale = static_cast<float>(std::max(sphere.radius * 4.0, 0.05));

  im.pose.position.x = sphere.position.x();
  im.pose.position.y = sphere.position.y();
  im.pose.position.z = sphere.position.z();
  im.pose.orientation.w = 1.0;

  // ── Visual + MOVE_3D control (drag sphere directly) ──────────────────────
  InteractiveMarkerControl move_ctrl;
  move_ctrl.name = "move_3d";
  move_ctrl.always_visible = true;
  move_ctrl.orientation.w = 1.0;
  move_ctrl.interaction_mode = InteractiveMarkerControl::MOVE_3D;
  move_ctrl.markers.push_back(buildSphereVisual(sphere));
  im.controls.push_back(move_ctrl);

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
