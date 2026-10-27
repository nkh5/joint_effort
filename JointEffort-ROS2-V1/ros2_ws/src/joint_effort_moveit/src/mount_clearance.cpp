// Inspect the modeled parked-state distances using MoveIt's actual collision engine.
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <moveit/planning_scene/planning_scene.hpp>
std::string read(const char* path) { std::ifstream f(path); std::ostringstream s; s << f.rdbuf(); return s.str(); }
int main(int argc, char** argv) {
  if (argc != 3) return 2;
  rclcpp::init(argc, argv);
  auto urdf = urdf::parseURDF(read(argv[1]));
  auto srdf = std::make_shared<srdf::Model>();
  if (!urdf || !srdf->initString(*urdf, read(argv[2]))) return 2;
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  planning_scene::PlanningScene scene(model);
  auto& state = scene.getCurrentStateNonConst();
  state.setToDefaultValues(model->getJointModelGroup("iron_arm"), "park");
  state.setToDefaultValues(model->getJointModelGroup("wire_arm"), "park");
  state.update();
  collision_detection::DistanceRequest req;
  req.acm = &scene.getAllowedCollisionMatrix();
  req.type = collision_detection::DistanceRequestTypes::SINGLE;
  req.distance_threshold = 0.02;
  collision_detection::DistanceResult result;
  scene.getCollisionEnv()->distanceSelf(req, result, state);
  for (const auto& pair : result.distances)
    for (const auto& d : pair.second)
      std::cout << pair.first.first << " / " << pair.first.second << ": " << d.distance << " m\n";
  rclcpp::shutdown();
}
