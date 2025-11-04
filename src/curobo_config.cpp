#include "curobo_robot_setup/curobo_config.hpp"

#include <fstream>
#include <filesystem>  // Fix Bug #6: For file permission checks
#include <yaml-cpp/yaml.h>

namespace curobo_robot_setup
{

bool CuRoboConfig::saveToYaml(
  const std::string& filepath,
  const std::string& urdf_path,
  const std::string& base_link,
  const std::string& ee_link,
  const std::map<std::string, std::vector<Sphere>>& spheres_by_link,
  const std::map<std::string, JointConfig>& joint_configs,
  const CSpaceConfig& cspace_config,
  const SelfCollisionConfig& collision_config)
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
    out << YAML::Key << "usd_flip_joints" << YAML::Value << YAML::BeginMap << YAML::EndMap;
    out << YAML::Key << "usd_flip_joint_limits" << YAML::Value << YAML::BeginSeq << YAML::EndSeq;
    
    // URDF
    out << YAML::Key << "urdf_path" << YAML::Value << urdf_path;
    out << YAML::Key << "asset_root_path" << YAML::Value << "";
    out << YAML::Key << "base_link" << YAML::Value << base_link;
    out << YAML::Key << "ee_link" << YAML::Value << ee_link;
    
    // Optional fields
    out << YAML::Key << "link_names" << YAML::Value << YAML::Null;
    out << YAML::Key << "lock_joints" << YAML::Value << YAML::Null;
    out << YAML::Key << "extra_links" << YAML::Value << YAML::Null;
    
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
        
        // Center (quoted key for compatibility)
        out << YAML::Key << "center" << YAML::Value << YAML::Flow << YAML::BeginSeq;
        out << sphere.position.x() << sphere.position.y() << sphere.position.z();
        out << YAML::EndSeq;
        
        // Radius (quoted key for compatibility)
        out << YAML::Key << "radius" << YAML::Value << sphere.radius;
        
        out << YAML::EndMap;
      }
      out << YAML::EndSeq;
    }
    out << YAML::EndMap;  // collision_spheres
    
    // Collision configuration

    out << YAML::Key << "collision_sphere_buffer" << YAML::Value << YAML::DoublePrecision(1) << collision_config.collision_sphere_buffer;
    out << YAML::Key << "extra_collision_spheres" << YAML::Value << YAML::BeginMap << YAML::EndMap;
    
    // Self collision ignore
    if (!collision_config.ignore_pairs.empty()) {
      out << YAML::Key << "self_collision_ignore" << YAML::Value << YAML::BeginMap;
      for (const auto& pair : collision_config.ignore_pairs) {
        out << YAML::Key << pair.first << YAML::Value << YAML::Flow << YAML::BeginSeq;
        for (const auto& link : pair.second) {
          out << link;
        }
        out << YAML::EndSeq;
      }
      out << YAML::EndMap;
    } else {
      out << YAML::Key << "self_collision_ignore" << YAML::Value << YAML::BeginMap << YAML::EndMap;
    }
    
    // Self collision buffer
    if (!collision_config.buffer_distances.empty()) {
      out << YAML::Key << "self_collision_buffer" << YAML::Value << YAML::BeginMap;
      for (const auto& pair : collision_config.buffer_distances) {
        out << YAML::Key << pair.first << YAML::Value << pair.second;
      }
      out << YAML::EndMap;
    } else {
      out << YAML::Key << "self_collision_buffer" << YAML::Value << YAML::BeginMap << YAML::EndMap;
    }
    
    // Additional fields
    out << YAML::Key << "use_global_cumul" << YAML::Value << true;
    out << YAML::Key << "mesh_link_names" << YAML::Value << YAML::Null;
    
    // CSpace configuration
    out << YAML::Key << "cspace" << YAML::Value << YAML::BeginMap;
    
    // Joint names (only active joints)
    out << YAML::Key << "joint_names" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (const auto& pair : joint_configs) {
      if (pair.second.active) {
        out << pair.first;
      }
    }
    out << YAML::EndSeq;
    
    // Retract config (default to zeros)
    out << YAML::Key << "retract_config" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (const auto& pair : joint_configs) {
      if (pair.second.active) {
        out << 0.0;
      }
    }
    out << YAML::EndSeq;
    
    // Null space weight
    out << YAML::Key << "null_space_weight" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    if (cspace_config.null_space_weight.empty()) {
      // Default: all 1.0
      for (const auto& pair : joint_configs) {
        if (pair.second.active) {
          out << 1.0;
        }
      }
    } else {
      for (double w : cspace_config.null_space_weight) {
        out << w;
      }
    }
    out << YAML::EndSeq;
    
    // CSpace distance weight
    out << YAML::Key << "cspace_distance_weight" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    if (cspace_config.cspace_distance_weight.empty()) {
      // Default: all 1.0
      for (const auto& pair : joint_configs) {
        if (pair.second.active) {
          out << 1.0;
        }
      }
    } else {
      for (double w : cspace_config.cspace_distance_weight) {
        out << w;
      }
    }
    out << YAML::EndSeq;
    
    // Global limits (NOT per-joint!)
    out << YAML::Key << "max_jerk" << YAML::Value << YAML::DoublePrecision(1) << cspace_config.max_jerk;
    out << YAML::Key << "max_acceleration" << YAML::Value << YAML::DoublePrecision(1) << cspace_config.max_acceleration;
    
    out << YAML::EndMap;  // cspace
    
    out << YAML::EndMap;  // kinematics
    out << YAML::EndMap;  // robot_cfg
    out << YAML::EndMap;  // root

    // Fix Bug #6: Validate file path before attempting to write
    std::filesystem::path file_path(filepath);

    // Check if parent directory exists
    if (file_path.has_parent_path()) {
      std::filesystem::path parent_path = file_path.parent_path();
      if (!std::filesystem::exists(parent_path)) {
        last_error_ = "Parent directory does not exist: " + parent_path.string();
        return false;
      }

      // Check if parent directory is writable
      auto parent_perms = std::filesystem::status(parent_path).permissions();
      if ((parent_perms & std::filesystem::perms::owner_write) == std::filesystem::perms::none) {
        last_error_ = "No write permission for directory: " + parent_path.string();
        return false;
      }
    }

    // Check if file already exists and is writable
    if (std::filesystem::exists(filepath)) {
      auto file_perms = std::filesystem::status(filepath).permissions();
      if ((file_perms & std::filesystem::perms::owner_write) == std::filesystem::perms::none) {
        last_error_ = "No write permission for existing file: " + filepath;
        return false;
      }
    }

    // Write to file
    std::ofstream fout(filepath);
    if (!fout.is_open()) {
      last_error_ = "Could not open file for writing: " + filepath;
      return false;
    }

    fout << out.c_str();
    fout.close();

    // Verify write was successful
    if (!std::filesystem::exists(filepath)) {
      last_error_ = "File was not created: " + filepath;
      return false;
    }

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
  std::map<std::string, JointConfig>& joint_configs,
  CSpaceConfig& cspace_config,
  SelfCollisionConfig& collision_config)
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

      // Fix Bug #2: Use local counter instead of static to avoid persistence issues
      int sphere_counter = 0;

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

          // Fix Bug #2: Generate ID using local counter (resets on each load)
          sphere.id = "sphere_loaded_" + std::to_string(sphere_counter++);

          spheres.push_back(sphere);
        }

        spheres_by_link[link_name] = spheres;
      }
    }
    
    // Load self collision config
    if (kinematics["collision_sphere_buffer"]) {
      collision_config.collision_sphere_buffer = kinematics["collision_sphere_buffer"].as<double>();
    }
    
    if (kinematics["self_collision_ignore"]) {
      YAML::Node ignore_node = kinematics["self_collision_ignore"];
      for (auto it = ignore_node.begin(); it != ignore_node.end(); ++it) {
        std::string link_name = it->first.as<std::string>();
        std::vector<std::string> ignore_links = it->second.as<std::vector<std::string>>();
        collision_config.ignore_pairs[link_name] = ignore_links;
      }
    }
    
    if (kinematics["self_collision_buffer"]) {
      YAML::Node buffer_node = kinematics["self_collision_buffer"];
      for (auto it = buffer_node.begin(); it != buffer_node.end(); ++it) {
        std::string link_name = it->first.as<std::string>();
        double buffer = it->second.as<double>();
        collision_config.buffer_distances[link_name] = buffer;
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
      
      // Create joint configs (active only)
      for (const auto& joint_name : active_joints) {
        JointConfig config;
        config.active = true;
        joint_configs[joint_name] = config;
      }
      
      // Load cspace config
      if (cspace["max_jerk"]) {
        cspace_config.max_jerk = cspace["max_jerk"].as<double>();
      }
      
      if (cspace["max_acceleration"]) {
        cspace_config.max_acceleration = cspace["max_acceleration"].as<double>();
      }
      
      if (cspace["null_space_weight"]) {
        cspace_config.null_space_weight = cspace["null_space_weight"].as<std::vector<double>>();
      }
      
      if (cspace["cspace_distance_weight"]) {
        cspace_config.cspace_distance_weight = cspace["cspace_distance_weight"].as<std::vector<double>>();
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
  
  return true;
}

}  // namespace curobo_robot_setup