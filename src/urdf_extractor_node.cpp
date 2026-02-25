/**
 * urdf_extractor_node.cpp
 *
 * ROS2 node that subscribes to the robot_description topic
 * (std_msgs/msg/String, published with transient_local / latched QoS by
 * robot_state_publisher), formats the XML, and saves it as a .urdf file.
 *
 * Parameters (all settable via --ros-args -p <name>:=<value>):
 *   topic       (string) – topic to subscribe to
 *                          (default: /robot_description)
 *   output_dir  (string) – directory where the .urdf file is written
 *                          (default: /home/ros2_ws/src/curobo_robot_setup/example)
 *   robot_name  (string) – override the output file base-name
 *                          (default: "" → extracted from <robot name="…">)
 *
 * Usage:
 *   ros2 run curobo_robot_setup urdf_extractor
 *   ros2 run curobo_robot_setup urdf_extractor --ros-args \
 *       -p topic:=/robot_description \
 *       -p output_dir:=/some/path \
 *       -p robot_name:=my_robot
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <tinyxml2.h>

#include <filesystem>
#include <fstream>
#include <regex>
#include <string>

namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
class UrdfExtractorNode : public rclcpp::Node
{
public:
  UrdfExtractorNode()
  : rclcpp::Node("urdf_extractor")
  {
    declare_parameter<std::string>("topic", "/robot_description");
    declare_parameter<std::string>(
      "output_dir",
      "/home/ros2_ws/src/curobo_robot_setup/example");
    declare_parameter<std::string>("robot_name", "");

    const std::string topic = get_parameter("topic").as_string();
    output_dir_  = get_parameter("output_dir").as_string();
    robot_name_  = get_parameter("robot_name").as_string();

    // robot_state_publisher publishes with transient_local (latched) QoS;
    // we must match it to receive the retained message even if we connect late.
    auto qos = rclcpp::QoS(1).transient_local();

    sub_ = create_subscription<std_msgs::msg::String>(
      topic, qos,
      std::bind(&UrdfExtractorNode::onRobotDescription, this,
                std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "Subscribed to '%s'. Waiting for robot_description ...", topic.c_str());
  }

private:
  // -------------------------------------------------------------------------
  void onRobotDescription(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data.empty()) {
      RCLCPP_ERROR(get_logger(), "Received empty robot_description message.");
      rclcpp::shutdown();
      return;
    }

    saveUrdf(msg->data);
    rclcpp::shutdown();
  }

  // -------------------------------------------------------------------------
  void saveUrdf(const std::string & urdf_raw)
  {
    // --- Parse XML -----------------------------------------------------------
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError err = doc.Parse(urdf_raw.c_str());
    if (err != tinyxml2::XML_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to parse URDF XML: %s", doc.ErrorStr());
      return;
    }

    // --- Pretty-print --------------------------------------------------------
    tinyxml2::XMLPrinter printer(nullptr, /*compact=*/false);
    doc.Print(&printer);
    std::string pretty_urdf = printer.CStr();

    // tinyxml2 does not add an XML declaration; prepend one explicitly
    static const char kDecl[] = "<?xml version=\"1.0\" ?>\n";
    if (pretty_urdf.rfind("<?xml", 0) != 0) {
      pretty_urdf = kDecl + pretty_urdf;
    }

    // --- Determine output file name ------------------------------------------
    std::string name = robot_name_;
    if (name.empty()) {
      std::regex re(R"(<robot[^>]+name=\"([^\"]+)\")");
      std::smatch m;
      if (std::regex_search(urdf_raw, m, re)) {
        name = m[1].str();
      } else {
        name = "robot";
      }
    }

    // Sanitise: keep alphanumeric, '-', '_', '.'
    std::transform(name.begin(), name.end(), name.begin(), [](char c) -> char {
      return (std::isalnum(static_cast<unsigned char>(c)) ||
              c == '_' || c == '-' || c == '.') ? c : '_';
    });

    // --- Write file ----------------------------------------------------------
    fs::create_directories(output_dir_);
    std::string out_path = output_dir_ + "/" + name + ".urdf";

    std::ofstream ofs(out_path, std::ios::out | std::ios::trunc);
    if (!ofs.is_open()) {
      RCLCPP_ERROR(get_logger(),
        "Cannot open output file for writing: '%s'", out_path.c_str());
      return;
    }

    ofs << pretty_urdf;
    ofs.close();

    RCLCPP_INFO(get_logger(), "URDF saved to: %s", out_path.c_str());
  }

  // -------------------------------------------------------------------------
  std::string output_dir_;
  std::string robot_name_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UrdfExtractorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
