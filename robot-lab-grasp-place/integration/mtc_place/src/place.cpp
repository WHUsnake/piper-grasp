#include <fstream>
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mtc = moveit::task_constructor;
using json = nlohmann::json;

static json path(const trajectory_msgs::msg::JointTrajectory& trajectory) {
  static const std::vector<std::string> joints = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  std::vector<size_t> order;
  for (const auto& name : joints) {
    auto found = std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), name);
    if (found == trajectory.joint_names.end()) throw std::runtime_error("MTC trajectory omitted " + name);
    order.push_back(std::distance(trajectory.joint_names.begin(), found));
  }
  json points = json::array();
  double elapsed = 0.;
  std::vector<double> previous;
  for (const auto& waypoint : trajectory.points) {
    std::vector<double> values;
    for (size_t index : order) values.push_back(waypoint.positions.at(index));
    const double planned = waypoint.time_from_start.sec + waypoint.time_from_start.nanosec * 1e-9;
    if (!previous.empty()) {
      double delta = 0.;
      for (size_t index = 0; index < values.size(); ++index)
        delta = std::max(delta, std::abs(values[index] - previous[index]));
      elapsed = std::max(planned, elapsed + std::max(.02, delta / .35));
    }
    const auto seconds = static_cast<int32_t>(elapsed);
    const auto nanoseconds = static_cast<uint32_t>(std::llround((elapsed - seconds) * 1e9));
    points.push_back({{"positions", values}, {"sec", seconds}, {"nanosec", nanoseconds}});
    previous = std::move(values);
  }
  if (points.size() < 2) throw std::runtime_error("MTC motion has fewer than two waypoints");
  return {{"joint_names", joints}, {"points", points}};
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    if (argc < 2) throw std::runtime_error("Request JSON path required");
    std::ifstream stream(argv[1]);
    json request = json::parse(stream);
    auto node = std::make_shared<rclcpp::Node>("go2_place_mtc", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    mtc::Task task;
    task.setName("RGB-D place " + request.at("phase").get<std::string>());
    task.loadRobotModel(node);
    auto scene = std::make_shared<planning_scene::PlanningScene>(task.getRobotModel());
    static const std::vector<std::string> joints = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"};
    std::vector<double> positions = request.at("joints").get<std::vector<double>>();
    auto jaws = request.at("jaw_positions").get<std::vector<double>>();
    positions.insert(positions.end(), jaws.begin(), jaws.end());
    if (positions.size() != joints.size()) throw std::runtime_error("Expected six arm and two jaw joints");
    for (size_t index = 0; index < joints.size(); ++index) {
      const auto& bounds = task.getRobotModel()->getVariableBounds(joints[index]);
      if (positions[index] < bounds.min_position_ - .001 || positions[index] > bounds.max_position_ + .001)
        throw std::runtime_error("Measured joint exceeds planning bounds: " + joints[index]);
      scene->getCurrentStateNonConst().setVariablePosition(
          joints[index], std::clamp(positions[index], bounds.min_position_, bounds.max_position_));
    }
    scene->getCurrentStateNonConst().update();
    auto initial = std::make_unique<mtc::stages::FixedState>("measured current state");
    initial->setState(scene);
    task.add(std::move(initial));
    auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian->setStepSize(.005);
    std::vector<std::string> expected;
    auto vertical = [](double sign) {
      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "world";
      direction.vector.z = sign;
      return direction;
    };
    if (request.at("phase") == "before_release") {
      auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(node);
      pipeline->setPlannerId("RRTConnectkConfigDefault");
      auto approach = std::make_unique<mtc::stages::MoveTo>("approach support", pipeline);
      approach->setGroup("arm");
      std::map<std::string, double> goal;
      const auto above = request.at("above").get<std::vector<double>>();
      if (above.size() != 6) throw std::runtime_error("Expected six approach joints");
      for (size_t index = 0; index < 6; ++index) goal[joints[index]] = above[index];
      approach->setGoal(goal);
      task.add(std::move(approach));
      // The authoritative simulator performs feedback-controlled 3 mm steps.
      expected = {"approach"};
    } else if (request.at("phase") == "after_release") {
      auto retreat = std::make_unique<mtc::stages::MoveRelative>("retreat from support", cartesian);
      retreat->setGroup("arm");
      retreat->setIKFrame("link6");
      retreat->setMinMaxDistance(.095, .105);
      retreat->setDirection(vertical(1.));
      task.add(std::move(retreat));
      auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(node);
      pipeline->setPlannerId("RRTConnectkConfigDefault");
      auto zero = std::make_unique<mtc::stages::MoveTo>("return to zero", pipeline);
      zero->setGroup("arm");
      std::map<std::string, double> goal;
      for (size_t index = 0; index < 6; ++index) goal[joints[index]] = 0.;
      zero->setGoal(goal);
      task.add(std::move(zero));
      expected = {"retreat", "zero"};
    } else throw std::runtime_error("Unknown MTC placement phase");
    if (!task.plan(1) || task.solutions().empty()) throw std::runtime_error("MTC could not plan requested stages");
    moveit_task_constructor_msgs::msg::Solution message;
    task.solutions().front()->toMsg(message);
    json output = json::object();
    size_t index = 0;
    for (const auto& segment : message.sub_trajectory) {
      if (segment.trajectory.joint_trajectory.points.empty()) continue;
      if (index >= expected.size()) throw std::runtime_error("MTC returned unexpected motion stage");
      output[expected[index++]] = path(segment.trajectory.joint_trajectory);
    }
    if (index != expected.size()) throw std::runtime_error("MTC omitted a motion stage");
    std::ofstream result(std::string(argv[1]).substr(0, std::string(argv[1]).size()-5) + "-result.json");
    result << output.dump(2);
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("go2_place_mtc"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
