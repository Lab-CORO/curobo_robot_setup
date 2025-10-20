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
  bool active = true;
  double pos_min = -3.14;
  double pos_max = 3.14;
  double vel_max = 2.0;
  double acc_max = 5.0;
  double jerk_max = 50.0;
  std::vector<std::string> ignore_collisions;

  JointConfig() = default;
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
    const std::map<std::string, JointConfig>& joint_configs
  );

  // Import from cuRobo YAML format
  bool loadFromYaml(
    const std::string& filepath,
    std::string& urdf_path,
    std::string& base_link,
    std::string& ee_link,
    std::map<std::string, std::vector<Sphere>>& spheres_by_link,
    std::map<std::string, JointConfig>& joint_configs
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