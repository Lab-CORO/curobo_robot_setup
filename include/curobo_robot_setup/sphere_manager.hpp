#ifndef SPHERE_MANAGER_HPP
#define SPHERE_MANAGER_HPP

#include <string>
#include <vector>
#include <map>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>

namespace curobo_robot_setup
{

struct Sphere
{
  std::string id;              // Unique ID: "sphere_0", "sphere_1", etc.
  std::string parent_link;     // Link parent
  double radius;               // Rayon en mètres
  Eigen::Vector3d position;    // Position relative au link (x, y, z)
  
  Sphere() : radius(0.05), position(Eigen::Vector3d::Zero()) {}
  
  Sphere(const std::string& link, double r, const Eigen::Vector3d& pos)
    : parent_link(link), radius(r), position(pos)
  {
    static int counter = 0;
    id = "sphere_" + std::to_string(counter++);
  }
};

class SphereManager
{
public:
  explicit SphereManager(rclcpp::Node::SharedPtr node);
  ~SphereManager() = default;

  // Gestion des sphères
  std::string addSphere(const std::string& parent_link, 
                       double radius, 
                       const Eigen::Vector3d& position);
  
  bool removeSphere(const std::string& sphere_id);
  
  bool updateSphere(const std::string& sphere_id,
                   double radius,
                   const Eigen::Vector3d& position);
  
  void clear();
  
  // Récupération
  const Sphere* getSphere(const std::string& sphere_id) const;
  std::vector<Sphere> getSpheresByLink(const std::string& link) const;
  std::map<std::string, std::vector<Sphere>> getAllSpheresByLink() const;
  std::vector<Sphere> getAllSpheres() const;
  
  size_t getTotalSphereCount() const { return spheres_.size(); }
  
  // Visualisation
  void publishMarkers();
  void setMarkerColor(float r, float g, float b, float a = 0.7f);

private:
  visualization_msgs::msg::Marker createMarker(const Sphere& sphere);
  
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  
  std::map<std::string, Sphere> spheres_;  // sphere_id -> Sphere
  
  // Couleur des markers
  float marker_color_r_;
  float marker_color_g_;
  float marker_color_b_;
  float marker_color_a_;
};

}  // namespace curobo_robot_setup

#endif  // SPHERE_MANAGER_HPP