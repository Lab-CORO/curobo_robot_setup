#ifndef SPHERE_MANAGER_HPP
#define SPHERE_MANAGER_HPP

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace curobo_robot_setup
{

struct Sphere
{
  std::string id;           // "sphere_0", "sphere_1", …
  std::string parent_link;  // Link this sphere is attached to
  double radius;            // Radius in metres
  Eigen::Vector3d position; // Position relative to parent link frame (x, y, z)

  Sphere() : radius(0.05), position(Eigen::Vector3d::Zero()) {}

  Sphere(const std::string & link, double r, const Eigen::Vector3d & pos)
  : parent_link(link), radius(r), position(pos)
  {}
};

class SphereManager
{
public:
  explicit SphereManager(rclcpp::Node::SharedPtr node);
  ~SphereManager() = default;

  // ── CRUD ──────────────────────────────────────────────────────────────────
  std::string addSphere(const std::string & parent_link,
                        double radius,
                        const Eigen::Vector3d & position);

  bool removeSphere(const std::string & sphere_id);

  bool updateSphere(const std::string & sphere_id,
                    double radius,
                    const Eigen::Vector3d & position);

  void clear();

  // ── Query ─────────────────────────────────────────────────────────────────
  const Sphere * getSphere(const std::string & sphere_id) const;
  std::vector<Sphere> getSpheresByLink(const std::string & link) const;
  std::map<std::string, std::vector<Sphere>> getAllSpheresByLink() const;
  std::vector<Sphere> getAllSpheres() const;
  size_t getTotalSphereCount() const;

  // ── Appearance ────────────────────────────────────────────────────────────
  void setMarkerColor(float r, float g, float b, float a = 0.7f);

  // ── Drag callback (called from ROS2 executor thread) ─────────────────────
  // Panel must use QMetaObject::invokeMethod(Qt::QueuedConnection) inside.
  using MovedCallback = std::function<void(const std::string & id,
                                           const Eigen::Vector3d & new_pos)>;
  void setOnMovedCallback(MovedCallback cb) { on_moved_callback_ = std::move(cb); }

private:
  // Interactive marker helpers
  void insertMarker(const Sphere & sphere);
  visualization_msgs::msg::InteractiveMarker buildInteractiveMarker(const Sphere & sphere) const;
  visualization_msgs::msg::Marker buildSphereVisual(const Sphere & sphere) const;

  void processFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & fb);

  std::string generateSphereId();

  // ── Members ───────────────────────────────────────────────────────────────
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server_;

  mutable std::mutex spheres_mutex_;
  std::map<std::string, Sphere> spheres_;
  int next_sphere_id_{0};

  float marker_color_r_{0.0f};
  float marker_color_g_{0.8f};
  float marker_color_b_{1.0f};
  float marker_color_a_{0.7f};

  MovedCallback on_moved_callback_;
};

}  // namespace curobo_robot_setup

#endif  // SPHERE_MANAGER_HPP
