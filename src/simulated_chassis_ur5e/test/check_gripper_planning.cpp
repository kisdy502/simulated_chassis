// Inspect the real MoveIt robot model/ACM without starting move_group or Gazebo.
#include <iostream>
#include <memory>
#include <string>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <moveit/planning_scene/planning_scene.h>

int main(int argc, char ** argv)
{
  if (argc != 4) {return 2;}
  const auto urdf = urdf::parseURDFFile(argv[1]);
  auto srdf = std::make_shared<srdf::Model>();
  if (!urdf || !srdf->initFile(*urdf, argv[2])) {return 3;}
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  auto group = model->getJointModelGroup("ur5e_ur_manipulator");
  if (!group || group->getVariableCount() != 6 || !model->hasLinkModel("ur5e_gripper_tcp")) {return 4;}
  planning_scene::PlanningScene scene(model);
  auto & state = scene.getCurrentStateNonConst();
  state.setToDefaultValues();
  if (!state.setToDefaultValues("ur5e_ur_manipulator", "stowed")) {return 5;}
  state.setVariablePosition("ur5e_gripper_left_joint", std::stod(argv[3]) / 2);
  state.setVariablePosition("ur5e_gripper_right_joint", std::stod(argv[3]) / 2);
  state.update();
  collision_detection::CollisionRequest request;
  request.contacts = true;
  request.max_contacts = 100;
  collision_detection::CollisionResult result;
  scene.checkSelfCollision(request, result);
  // Scope this regression to the new tool. Existing wheel/chassis contacts are
  // independent of this change; do not hide them by altering the production ACM.
  for (const auto & contact : result.contacts) {
    if (contact.first.first.find("ur5e_gripper") != std::string::npos ||
      contact.first.second.find("ur5e_gripper") != std::string::npos)
    {
      std::cerr << contact.first.first << " <-> " << contact.first.second << '\n';
      return 6;
    }
  }
  std::cout << "Stowed model: six arm axes, gripper TCP, no gripper self-collision\n";
  return 0;
}
