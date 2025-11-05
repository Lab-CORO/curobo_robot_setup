#ifndef CUROBO_CONFIG_HPP
#define CUROBO_CONFIG_HPP

#include <string>
#include <vector>
#include <map>
#include <memory>

#include "curobo_robot_setup/sphere_manager.hpp"
#include <urdf/model.h>

namespace curobo_robot_setup
{

struct JointConfig
{
  // Joint is active (included in cuRobo configuration)
  bool active = true;

  // Note: All joint limits (position, velocity, acceleration) are read
  // directly from the URDF by cuRobo. We only track which joints are active.

  JointConfig() = default;
};

struct CSpaceConfig
{
  // Global limits (not per-joint)
  double max_acceleration = 15.0;
  double max_jerk = 500.0;
  
  // Weights (one per joint)
  std::vector<double> null_space_weight;
  std::vector<double> cspace_distance_weight;
  
  CSpaceConfig() = default;
};

struct SelfCollisionConfig
{
  std::map<std::string, std::vector<std::string>> ignore_pairs;
  std::map<std::string, double> buffer_distances;
  double collision_sphere_buffer = 0.0;
  
  SelfCollisionConfig() = default;
};

class CuRoboConfig
{
public:
  CuRoboConfig() = default;
  ~CuRoboConfig() = default;

  // Export to cuRobo YAML format
  bool saveToYaml(
    const std::string& filepath,
    const std::string& urdf_path,
    const std::string& base_link,
    const std::string& ee_link,
    const std::map<std::string, std::vector<Sphere>>& spheres_by_link,
    const std::map<std::string, JointConfig>& joint_configs,
    const CSpaceConfig& cspace_config,
    const SelfCollisionConfig& collision_config
  );

  // Import from cuRobo YAML format
  bool loadFromYaml(
    const std::string& filepath,
    std::string& urdf_path,
    std::string& base_link,
    std::string& ee_link,
    std::map<std::string, std::vector<Sphere>>& spheres_by_link,
    std::map<std::string, JointConfig>& joint_configs,
    CSpaceConfig& cspace_config,
    SelfCollisionConfig& collision_config
  );

  // Helper: Get default joint config from URDF
  static JointConfig getDefaultJointConfig(
    const std::shared_ptr<urdf::Model>& robot_model,
    const std::string& joint_name
  );

  // Helper: Get all joint names from URDF (non-fixed)
  static std::vector<std::string> getActiveJointNames(
    const std::shared_ptr<urdf::Model>& robot_model
  );

  // Validation
  bool validateConfig(
    const std::shared_ptr<urdf::Model>& robot_model,
    const std::string& base_link,
    const std::string& ee_link,
    const std::map<std::string, JointConfig>& joint_configs,
    std::string& error_message
  );

private:
  std::string last_error_;
};

}  // namespace curobo_robot_setup

#endif  // CUROBO_CONFIG_HPP