#include "curobo_robot_setup/curobo_config.hpp"

#include <fstream>
#include <yaml-cpp/yaml.h>

namespace curobo_robot_setup
{

bool CuRoboConfig::saveToYaml(
  const std::string& filepath,
  const std::string& urdf_path,
  const std::string& base_link,
  const std::string& ee_link,
  const std::map<std::string, std::vector<Sphere>>& spheres_by_link,
  const std::map<std::string, JointConfig>& joint_configs)
{
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;

    // Root: robot_cfg
    out << YAML::Key << "robot_cfg";
    out << YAML::Value << YAML::BeginMap;

    // Kinematics section
    out << YAML::Key << "kinematics";
    out << YAML::Value << YAML::BeginMap;

    // USD (not used)
    out << YAML::Key << "use_usd_kinematics" << YAML::Value << false;
    out << YAML::Key << "usd_path" << YAML::Value << "";
    out << YAML::Key << "usd_robot_root" << YAML::Value << "";
    out << YAML::Key << "isaac_usd_path" << YAML::Value << "";

    // URDF
    out << YAML::Key << "urdf_path" << YAML::Value << urdf_path;
    out << YAML::Key << "base_link" << YAML::Value << base_link;
    out << YAML::Key << "ee_link" << YAML::Value << ee_link;

    // Collision link names
    out << YAML::Key << "collision_link_names" << YAML::Value << YAML::BeginSeq;
    for (const auto& pair : spheres_by_link) {
      out << pair.first;
    }
    out << YAML::EndSeq;

    // Collision spheres
    out << YAML::Key << "collision_spheres" << YAML::Value << YAML::BeginMap;
    for (const auto& pair : spheres_by_link) {
      const std::string& link_name = pair.first;
      const std::vector<Sphere>& spheres = pair.second;

      out << YAML::Key << link_name << YAML::Value << YAML::BeginSeq;
      for (const Sphere& sphere : spheres) {
        out << YAML::BeginMap;

        // Center
        out << YAML::Key << "center" << YAML::Value << YAML::Flow << YAML::BeginSeq;
        out << sphere.position.x() << sphere.position.y() << sphere.position.z();
        out << YAML::EndSeq;

        // Radius
        out << YAML::Key << "radius" << YAML::Value << sphere.radius;

        out << YAML::EndMap;
      }
      out << YAML::EndSeq;
    }
    out << YAML::EndMap;  // collision_spheres

    // CSpace configuration
    out << YAML::Key << "cspace" << YAML::Value << YAML::BeginMap;

    // Joint names (only active joints)
    out << YAML::Key << "joint_names" << YAML::Value << YAML::BeginSeq;
    for (const auto& pair : joint_configs) {
      if (pair.second.active) {
        out << pair.first;
      }
    }
    out << YAML::EndSeq;

    // Retract config (default to zeros)
    out << YAML::Key << "retract_config" << YAML::Value << YAML::BeginSeq;
    for (const auto& pair : joint_configs) {
      if (pair.second.active) {
        out << 0.0;
      }
    }
    out << YAML::EndSeq;

    // Joint limits
    if (!joint_configs.empty()) {
      out << YAML::Key << "joint_limits" << YAML::Value << YAML::BeginMap;
      for (const auto& pair : joint_configs) {
        if (pair.second.active) {
          const std::string& joint_name = pair.first;
          const JointConfig& config = pair.second;

          out << YAML::Key << joint_name << YAML::Value << YAML::BeginMap;

          out << YAML::Key << "position" << YAML::Value << YAML::Flow << YAML::BeginSeq;
          out << config.pos_min << config.pos_max;
          out << YAML::EndSeq;

          out << YAML::Key << "velocity" << YAML::Value << config.vel_max;
          out << YAML::Key << "acceleration" << YAML::Value << config.acc_max;
          out << YAML::Key << "jerk" << YAML::Value << config.jerk_max;

          out << YAML::EndMap;
        }
      }
      out << YAML::EndMap;  // joint_limits
    }

    // Self collision ignore
    bool has_collision_ignores = false;
    for (const auto& pair : joint_configs) {
      if (!pair.second.ignore_collisions.empty()) {
        has_collision_ignores = true;
        break;
      }
    }

    if (has_collision_ignores) {
      out << YAML::Key << "self_collision_ignore" << YAML::Value << YAML::BeginMap;
      for (const auto& pair : joint_configs) {
        if (!pair.second.ignore_collisions.empty()) {
          out << YAML::Key << pair.first << YAML::Value << YAML::Flow << YAML::BeginSeq;
          for (const auto& link : pair.second.ignore_collisions) {
            out << link;
          }
          out << YAML::EndSeq;
        }
      }
      out << YAML::EndMap;  // self_collision_ignore
    } else {
      out << YAML::Key << "self_collision_ignore" << YAML::Value << YAML::BeginMap << YAML::EndMap;
    }

    out << YAML::EndMap;  // cspace

    // Lock joints (empty for now)
    out << YAML::Key << "lock_joints" << YAML::Value << YAML::BeginMap << YAML::EndMap;

    out << YAML::EndMap;  // kinematics
    out << YAML::EndMap;  // robot_cfg
    out << YAML::EndMap;  // root

    // Write to file
    std::ofstream fout(filepath);
    if (!fout.is_open()) {
      last_error_ = "Could not open file for writing: " + filepath;
      return false;
    }

    fout << out.c_str();
    fout.close();

    return true;

  } catch (const std::exception& e) {
    last_error_ = std::string("Exception during YAML export: ") + e.what();
    return false;
  }
}

bool CuRoboConfig::loadFromYaml(
  const std::string& filepath,
  std::string& urdf_path,
  std::string& base_link,
  std::string& ee_link,
  std::map<std::string, std::vector<Sphere>>& spheres_by_link,
  std::map<std::string, JointConfig>& joint_configs)
{
  try {
    YAML::Node config = YAML::LoadFile(filepath);

    if (!config["robot_cfg"] || !config["robot_cfg"]["kinematics"]) {
      last_error_ = "Invalid cuRobo YAML format: missing robot_cfg.kinematics";
      return false;
    }

    YAML::Node kinematics = config["robot_cfg"]["kinematics"];

    // Load basic info
    if (kinematics["urdf_path"]) {
      urdf_path = kinematics["urdf_path"].as<std::string>();
    }

    if (kinematics["base_link"]) {
      base_link = kinematics["base_link"].as<std::string>();
    }

    if (kinematics["ee_link"]) {
      ee_link = kinematics["ee_link"].as<std::string>();
    }

    // Load collision spheres
    spheres_by_link.clear();
    if (kinematics["collision_spheres"]) {
      YAML::Node spheres_node = kinematics["collision_spheres"];

      for (auto it = spheres_node.begin(); it != spheres_node.end(); ++it) {
        std::string link_name = it->first.as<std::string>();
        YAML::Node link_spheres = it->second;

        std::vector<Sphere> spheres;
        for (size_t i = 0; i < link_spheres.size(); ++i) {
          YAML::Node sphere_node = link_spheres[i];

          Sphere sphere;
          sphere.parent_link = link_name;

          if (sphere_node["radius"]) {
            sphere.radius = sphere_node["radius"].as<double>();
          }

          if (sphere_node["center"] && sphere_node["center"].size() == 3) {
            sphere.position.x() = sphere_node["center"][0].as<double>();
            sphere.position.y() = sphere_node["center"][1].as<double>();
            sphere.position.z() = sphere_node["center"][2].as<double>();
          }

          // Generate ID
          static int counter = 0;
          sphere.id = "sphere_" + std::to_string(counter++);

          spheres.push_back(sphere);
        }

        spheres_by_link[link_name] = spheres;
      }
    }

    // Load joint configs
    joint_configs.clear();
    if (kinematics["cspace"]) {
      YAML::Node cspace = kinematics["cspace"];

      // Get active joint names
      std::vector<std::string> active_joints;
      if (cspace["joint_names"]) {
        active_joints = cspace["joint_names"].as<std::vector<std::string>>();
      }

      // Load joint limits
      if (cspace["joint_limits"]) {
        YAML::Node limits = cspace["joint_limits"];

        for (const auto& joint_name : active_joints) {
          if (limits[joint_name]) {
            YAML::Node joint_limit = limits[joint_name];

            JointConfig config;
            config.active = true;

            if (joint_limit["position"] && joint_limit["position"].size() == 2) {
              config.pos_min = joint_limit["position"][0].as<double>();
              config.pos_max = joint_limit["position"][1].as<double>();
            }

            if (joint_limit["velocity"]) {
              config.vel_max = joint_limit["velocity"].as<double>();
            }

            if (joint_limit["acceleration"]) {
              config.acc_max = joint_limit["acceleration"].as<double>();
            }

            if (joint_limit["jerk"]) {
              config.jerk_max = joint_limit["jerk"].as<double>();
            }

            joint_configs[joint_name] = config;
          }
        }
      }

      // Load self collision ignores
      if (cspace["self_collision_ignore"]) {
        YAML::Node ignore_node = cspace["self_collision_ignore"];

        for (auto it = ignore_node.begin(); it != ignore_node.end(); ++it) {
          std::string joint_name = it->first.as<std::string>();
          std::vector<std::string> ignore_links = it->second.as<std::vector<std::string>>();

          if (joint_configs.find(joint_name) != joint_configs.end()) {
            joint_configs[joint_name].ignore_collisions = ignore_links;
          }
        }
      }
    }

    return true;

  } catch (const std::exception& e) {
    last_error_ = std::string("Exception during YAML import: ") + e.what();
    return false;
  }
}

JointConfig CuRoboConfig::getDefaultJointConfig(
  const std::shared_ptr<urdf::Model>& robot_model,
  const std::string& joint_name)
{
  JointConfig config;

  if (!robot_model) {
    return config;
  }

  auto joint = robot_model->getJoint(joint_name);
  if (joint && joint->limits) {
    config.pos_min = joint->limits->lower;
    config.pos_max = joint->limits->upper;
    config.vel_max = joint->limits->velocity;
    config.acc_max = 5.0;   // Default
    config.jerk_max = 50.0; // Default
  }

  return config;
}

std::vector<std::string> CuRoboConfig::getActiveJointNames(
  const std::shared_ptr<urdf::Model>& robot_model)
{
  std::vector<std::string> joint_names;

  if (!robot_model) {
    return joint_names;
  }

  for (const auto& pair : robot_model->joints_) {
    const auto& joint = pair.second;
    // Include all non-fixed joints
    if (joint->type != urdf::Joint::FIXED) {
      joint_names.push_back(joint->name);
    }
  }

  return joint_names;
}

bool CuRoboConfig::validateConfig(
  const std::shared_ptr<urdf::Model>& robot_model,
  const std::string& base_link,
  const std::string& ee_link,
  const std::map<std::string, JointConfig>& joint_configs,
  std::string& error_message)
{
  if (!robot_model) {
    error_message = "No robot model loaded";
    return false;
  }

  // Check base_link exists
  if (!robot_model->getLink(base_link)) {
    error_message = "Base link '" + base_link + "' does not exist in URDF";
    return false;
  }

  // Check ee_link exists
  if (!robot_model->getLink(ee_link)) {
    error_message = "End-effector link '" + ee_link + "' does not exist in URDF";
    return false;
  }

  // Check at least one active joint
  bool has_active = false;
  for (const auto& pair : joint_configs) {
    if (pair.second.active) {
      has_active = true;
      break;
    }
  }

  if (!has_active) {
    error_message = "No active joints selected";
    return false;
  }

  // Check joint limits are valid
  for (const auto& pair : joint_configs) {
    if (pair.second.active) {
      const JointConfig& config = pair.second;

      if (config.pos_min >= config.pos_max) {
        error_message = "Invalid position limits for joint '" + pair.first + "'";
        return false;
      }

      if (config.vel_max <= 0 || config.acc_max <= 0 || config.jerk_max <= 0) {
        error_message = "Velocity, acceleration and jerk must be positive for joint '" + pair.first + "'";
        return false;
      }
    }
  }

  return true;
}

}  // namespace curobo_robot_setup