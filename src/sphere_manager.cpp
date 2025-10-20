#include "curobo_robot_setup/sphere_manager.hpp"

namespace curobo_robot_setup
{

SphereManager::SphereManager(rclcpp::Node::SharedPtr node)
  : node_(node)
  , marker_color_r_(0.0f)
  , marker_color_g_(0.8f)
  , marker_color_b_(1.0f)
  , marker_color_a_(0.7f)
{
  // Créer publisher pour les markers
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/curobo_collision_spheres",
    10
  );
  
  RCLCPP_INFO(node_->get_logger(), "SphereManager initialized");
}

std::string SphereManager::addSphere(
  const std::string& parent_link,
  double radius,
  const Eigen::Vector3d& position)
{
  Sphere sphere(parent_link, radius, position);
  std::string id = sphere.id;
  
  spheres_[id] = sphere;
  
  RCLCPP_INFO(node_->get_logger(), 
              "Added sphere '%s' to link '%s' (r=%.4f, pos=[%.3f, %.3f, %.3f])",
              id.c_str(), parent_link.c_str(), radius,
              position.x(), position.y(), position.z());
  
  // Publier les markers mis à jour
  publishMarkers();
  
  return id;
}

bool SphereManager::removeSphere(const std::string& sphere_id)
{
  auto it = spheres_.find(sphere_id);
  if (it == spheres_.end()) {
    RCLCPP_WARN(node_->get_logger(), "Sphere '%s' not found", sphere_id.c_str());
    return false;
  }
  
  spheres_.erase(it);
  RCLCPP_INFO(node_->get_logger(), "Removed sphere '%s'", sphere_id.c_str());
  
  // Publier les markers mis à jour
  publishMarkers();
  
  return true;
}

bool SphereManager::updateSphere(
  const std::string& sphere_id,
  double radius,
  const Eigen::Vector3d& position)
{
  auto it = spheres_.find(sphere_id);
  if (it == spheres_.end()) {
    RCLCPP_WARN(node_->get_logger(), "Sphere '%s' not found", sphere_id.c_str());
    return false;
  }
  
  it->second.radius = radius;
  it->second.position = position;
  
  RCLCPP_INFO(node_->get_logger(), 
              "Updated sphere '%s' (r=%.4f, pos=[%.3f, %.3f, %.3f])",
              sphere_id.c_str(), radius,
              position.x(), position.y(), position.z());
  
  // Publier les markers mis à jour
  publishMarkers();
  
  return true;
}

void SphereManager::clear()
{
  spheres_.clear();
  RCLCPP_INFO(node_->get_logger(), "Cleared all spheres");
  
  // Publier pour supprimer tous les markers
  publishMarkers();
}

const Sphere* SphereManager::getSphere(const std::string& sphere_id) const
{
  auto it = spheres_.find(sphere_id);
  if (it != spheres_.end()) {
    return &(it->second);
  }
  return nullptr;
}

std::vector<Sphere> SphereManager::getSpheresByLink(const std::string& link) const
{
  std::vector<Sphere> result;
  
  for (const auto& pair : spheres_) {
    if (pair.second.parent_link == link) {
      result.push_back(pair.second);
    }
  }
  
  return result;
}

std::map<std::string, std::vector<Sphere>> SphereManager::getAllSpheresByLink() const
{
  std::map<std::string, std::vector<Sphere>> result;
  
  for (const auto& pair : spheres_) {
    result[pair.second.parent_link].push_back(pair.second);
  }
  
  return result;
}

std::vector<Sphere> SphereManager::getAllSpheres() const
{
  std::vector<Sphere> result;
  result.reserve(spheres_.size());
  
  for (const auto& pair : spheres_) {
    result.push_back(pair.second);
  }
  
  return result;
}

void SphereManager::publishMarkers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  
  // Supprimer tous les markers existants d'abord
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);
  
  // Ajouter un marker pour chaque sphère
  int marker_id = 0;
  for (const auto& pair : spheres_) {
    visualization_msgs::msg::Marker marker = createMarker(pair.second);
    marker.id = marker_id++;
    marker_array.markers.push_back(marker);
  }
  
  marker_pub_->publish(marker_array);
  
  RCLCPP_DEBUG(node_->get_logger(), 
               "Published %zu sphere markers", spheres_.size());
}

visualization_msgs::msg::Marker SphereManager::createMarker(const Sphere& sphere)
{
  visualization_msgs::msg::Marker marker;
  
  marker.header.frame_id = sphere.parent_link;
  marker.header.stamp = rclcpp::Time(0);  // Use latest available TF
  marker.ns = "collision_spheres";
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  
  // Position (relative to parent link)
  marker.pose.position.x = sphere.position.x();
  marker.pose.position.y = sphere.position.y();
  marker.pose.position.z = sphere.position.z();
  marker.pose.orientation.w = 1.0;
  
  // Scale (diameter = 2 * radius)
  marker.scale.x = sphere.radius * 2.0;
  marker.scale.y = sphere.radius * 2.0;
  marker.scale.z = sphere.radius * 2.0;
  
  // Color
  marker.color.r = marker_color_r_;
  marker.color.g = marker_color_g_;
  marker.color.b = marker_color_b_;
  marker.color.a = marker_color_a_;
  
  marker.lifetime = rclcpp::Duration::from_seconds(0);  // Permanent
  
  return marker;
}

void SphereManager::setMarkerColor(float r, float g, float b, float a)
{
  marker_color_r_ = r;
  marker_color_g_ = g;
  marker_color_b_ = b;
  marker_color_a_ = a;
  
  // Republier pour appliquer la nouvelle couleur
  publishMarkers();
}

}  // namespace curobo_robot_setup