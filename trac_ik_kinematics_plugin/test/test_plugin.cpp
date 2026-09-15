// PROTOTYPE HARNESS (wayfinder ticket 02) - throwaway, not production code.
//
// Seam under test: TRAC_IKKinematicsPlugin through the kinematics::KinematicsBase interface, the
// boundary MoveIt actually calls. The plugin is constructed directly against a RobotModel built from
// URDF+SRDF strings, with an rclcpp::Node carrying robot_description_kinematics.<group>.* parameters:
// no move_group, no launch file, no service call (a `ros2 service call` costs ~1.5 s on the VM).
//
// The independent oracle for every pose assertion is moveit::core::RobotState's own forward
// kinematics, never the plugin's getPositionFK - except in the one test that compares the two.
//
// RobotState is also what makes the mimic bug visible: applying a solution with
// setJointGroupPositions re-imposes the mimic coupling exactly as MoveIt does after an IK call, so
// CraneSolutionSurvivesMimicReimposition measures what the caller actually gets.

#include <gtest/gtest.h>

#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <trac_ik/trac_ik_kinematics_plugin.hpp>
#include <urdf_parser/urdf_parser.h>

#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace
{

constexpr double kPosTol = 1e-3;    // metres
constexpr double kRotTol = 1e-3;    // radians
constexpr double kTimeout = 0.05;   // MoveIt's default kinematics_solver_timeout (ticket 18)
constexpr int kSamples = 50;
constexpr double kSolveRateFloor = 0.90;  // generous: tighten once ticket 10 seeds the RNG

std::string readFixture(const std::string& name)
{
  std::ifstream in(std::string(TRAC_IK_TEST_FIXTURE_DIR) + "/" + name);
  EXPECT_TRUE(in.good()) << "fixture not found: " << name;
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

moveit::core::RobotModelPtr loadModel(const std::string& urdf_file, const std::string& srdf_file)
{
  auto urdf_model = urdf::parseURDF(readFixture(urdf_file));
  EXPECT_TRUE(static_cast<bool>(urdf_model));
  auto srdf_model = std::make_shared<srdf::Model>();
  EXPECT_TRUE(srdf_model->initString(*urdf_model, readFixture(srdf_file)));
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

// A node whose parameter overrides stand in for the kinematics.yaml MoveIt would load.
rclcpp::Node::SharedPtr makeNode(const std::string& group, const std::vector<rclcpp::Parameter>& overrides = {})
{
  static int n = 0;
  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  std::vector<rclcpp::Parameter> params;
  const std::string prefix = "robot_description_kinematics." + group + ".";
  for (const auto& p : overrides)
    params.emplace_back(prefix + p.get_name(), p.get_parameter_value());
  options.parameter_overrides(params);
  return std::make_shared<rclcpp::Node>("trac_ik_test_node_" + std::to_string(n++), options);
}

// FK through RobotState: the oracle. Applies the full configuration the way MoveIt does, which
// re-imposes any mimic coupling before the transform is read.
Eigen::Isometry3d fk(const moveit::core::RobotModelPtr& model, const std::string& group,
                     const std::vector<double>& full_config, const std::string& tip,
                     const std::string& base)
{
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  state.setJointGroupPositions(group, full_config);
  state.update();
  return state.getGlobalLinkTransform(base).inverse() * state.getGlobalLinkTransform(tip);
}

geometry_msgs::msg::Pose toPose(const Eigen::Isometry3d& t)
{
  return tf2::toMsg(t);
}

double positionError(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b)
{
  return (a.translation() - b.translation()).norm();
}

double rotationError(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b)
{
  return Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle();
}

std::vector<double> randomGroupConfig(const moveit::core::RobotModelPtr& model, const std::string& group,
                                      std::mt19937& rng)
{
  // Sampled through RobotState so mimic joints are consistent by construction: every target is
  // reachable under the coupling.
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  const auto* jmg = model->getJointModelGroup(group);
  random_numbers::RandomNumberGenerator rn(rng());
  state.setToRandomPositions(jmg, rn);
  state.update();
  std::vector<double> q;
  state.copyJointGroupPositions(group, q);
  return q;
}

struct CraneCase
{
  static constexpr const char* kGroup = "main_boom_jib";
  static constexpr const char* kBase = "base_link";
  static constexpr const char* kTip = "jib_ext_link";
};

}  // namespace

// ---------------------------------------------------------------------------------------------
// The solver joint list. MoveIt requires every group joint with variables, mimic joints included
// (research/moveit2-mimic-contract.md §1); omitting one corrupts memory in release builds.
// ---------------------------------------------------------------------------------------------

TEST(TracIkPlugin, SolverJointListIsEveryChainJointWithVariables)
{
  auto model = loadModel("crane.urdf", "crane.srdf");
  auto node = makeNode(CraneCase::kGroup, {rclcpp::Parameter("solve_type", "Distance")});
  trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin plugin;
  ASSERT_TRUE(plugin.initialize(node, *model, CraneCase::kGroup, CraneCase::kBase, {CraneCase::kTip}, 0.005));

  const std::vector<std::string> expected = {
    "turret_rotation", "main_boom_lift", "main_boom_ext_2", "main_boom_ext_3",
    "main_boom_ext_4", "main_boom_ext", "jib_lift", "jib_ext_2", "jib_ext"
  };
  EXPECT_EQ(plugin.getJointNames(), expected);
}

// ---------------------------------------------------------------------------------------------
// The measured baseline (ticket 01): a full-pose query from a zero seed returns SUCCESS, yet once
// MoveIt re-imposes the mimic coupling the pose is 0.090 m off. RED until tickets 04 and 05.
// ---------------------------------------------------------------------------------------------

TEST(TracIkPlugin, CraneSolutionSurvivesMimicReimposition)
{
  GTEST_SKIP() << "measures the mimic coupling defect; delete this skip in ticket 05";

  auto model = loadModel("crane.urdf", "crane.srdf");
  auto node = makeNode(CraneCase::kGroup, {rclcpp::Parameter("solve_type", "Distance")});
  trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin plugin;
  ASSERT_TRUE(plugin.initialize(node, *model, CraneCase::kGroup, CraneCase::kBase, {CraneCase::kTip}, 0.005));

  // The mimic-consistent state S from ticket 01, in solver joint order.
  const std::vector<double> S = {0.3, 0.7, 0.5, 0.5, 0.5, 0.5, -0.5, 0.4, 0.4};
  const Eigen::Isometry3d target = fk(model, CraneCase::kGroup, S, CraneCase::kTip, CraneCase::kBase);

  const std::vector<double> seed(9, 0.0);
  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes err;
  ASSERT_TRUE(plugin.searchPositionIK(toPose(target), seed, kTimeout, solution, err))
      << "no solution for a reachable crane pose";
  ASSERT_EQ(solution.size(), 9u);

  // What the caller actually gets: the solution applied through RobotState, mimics re-imposed.
  const Eigen::Isometry3d reached = fk(model, CraneCase::kGroup, solution, CraneCase::kTip, CraneCase::kBase);
  EXPECT_LT(positionError(target, reached), kPosTol)
      << "SUCCESS was reported but the pose after mimic re-imposition is off";
  EXPECT_LT(rotationError(target, reached), kRotTol);
}

TEST(TracIkPlugin, CraneSolutionRespectsMimicCoupling)
{
  GTEST_SKIP() << "measures the mimic coupling defect; delete this skip in ticket 05";

  auto model = loadModel("crane.urdf", "crane.srdf");
  auto node = makeNode(CraneCase::kGroup, {rclcpp::Parameter("solve_type", "Distance")});
  trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin plugin;
  ASSERT_TRUE(plugin.initialize(node, *model, CraneCase::kGroup, CraneCase::kBase, {CraneCase::kTip}, 0.005));

  const std::vector<double> S = {0.3, 0.7, 0.5, 0.5, 0.5, 0.5, -0.5, 0.4, 0.4};
  const Eigen::Isometry3d target = fk(model, CraneCase::kGroup, S, CraneCase::kTip, CraneCase::kBase);

  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes err;
  ASSERT_TRUE(plugin.searchPositionIK(toPose(target), std::vector<double>(9, 0.0), kTimeout, solution, err));

  // RED until ticket 04: the returned full configuration must already satisfy the coupling, so that
  // MoveIt's overwrite is a no-op rather than a silent correction.
  const auto& names = plugin.getJointNames();
  for (size_t i = 0; i < names.size(); ++i)
  {
    const auto* jm = model->getJointModel(names[i]);
    const auto* mimicked = jm->getMimic();
    if (!mimicked)
      continue;
    const auto it = std::find(names.begin(), names.end(), mimicked->getName());
    ASSERT_NE(it, names.end()) << "mimicked joint outside the solver joint list";
    const double expected = jm->getMimicFactor() * solution[std::distance(names.begin(), it)] + jm->getMimicOffset();
    EXPECT_NEAR(solution[i], expected, 1e-6) << names[i] << " must follow " << mimicked->getName();
  }
}

// ---------------------------------------------------------------------------------------------
// arm6: a conventional 6-DOF arm with no mimic joints. The control case.
// ---------------------------------------------------------------------------------------------

TEST(TracIkPlugin, Arm6FullPoseRoundTrip)
{
  auto model = loadModel("arm6.urdf", "arm6.srdf");
  auto node = makeNode("arm");
  trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin plugin;
  ASSERT_TRUE(plugin.initialize(node, *model, "arm", "base_link", {"tool_link"}, 0.005));

  std::mt19937 rng(4);
  int solved = 0;
  for (int n = 0; n < kSamples; ++n)
  {
    const auto q = randomGroupConfig(model, "arm", rng);
    const Eigen::Isometry3d target = fk(model, "arm", q, "tool_link", "base_link");
    const auto seed = randomGroupConfig(model, "arm", rng);

    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes err;
    if (!plugin.searchPositionIK(toPose(target), seed, kTimeout, solution, err))
      continue;
    ++solved;
    const Eigen::Isometry3d reached = fk(model, "arm", solution, "tool_link", "base_link");
    EXPECT_LT(positionError(target, reached), kPosTol);
    EXPECT_LT(rotationError(target, reached), kRotTol);
  }
  EXPECT_GE(static_cast<double>(solved) / kSamples, kSolveRateFloor)
      << "solved " << solved << " of " << kSamples;
}

TEST(TracIkPlugin, GetPositionFkAgreesWithRobotState)
{
  auto model = loadModel("arm6.urdf", "arm6.srdf");
  auto node = makeNode("arm");
  trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin plugin;
  ASSERT_TRUE(plugin.initialize(node, *model, "arm", "base_link", {"tool_link"}, 0.005));

  std::mt19937 rng(5);
  for (int n = 0; n < 10; ++n)
  {
    const auto q = randomGroupConfig(model, "arm", rng);
    std::vector<geometry_msgs::msg::Pose> poses;
    ASSERT_TRUE(plugin.getPositionFK({"tool_link"}, q, poses));
    ASSERT_EQ(poses.size(), 1u);

    Eigen::Isometry3d plugin_fk;
    tf2::fromMsg(poses[0], plugin_fk);
    const Eigen::Isometry3d oracle = fk(model, "arm", q, "tool_link", "base_link");
    EXPECT_LT(positionError(oracle, plugin_fk), 1e-9);
    EXPECT_LT(rotationError(oracle, plugin_fk), 1e-9);
  }
}


// ---------------------------------------------------------------------------------------------
// arm7 (wayfinder ticket 22): the redundant control case. 7 joints, no mimic joints, a genuine
// one-dimensional null space against a full-pose goal.
// ---------------------------------------------------------------------------------------------

TEST(TracIkPlugin, Arm7FullPoseRoundTrip)
{
  auto model = loadModel("arm7.urdf", "arm7.srdf");
  auto node = makeNode("arm");
  trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin plugin;
  ASSERT_TRUE(plugin.initialize(node, *model, "arm", "base_link", {"tool_link"}, 0.005));
  EXPECT_EQ(plugin.getJointNames().size(), 7u);

  std::mt19937 rng(7);
  int solved = 0;
  for (int n = 0; n < kSamples; ++n)
  {
    const auto q = randomGroupConfig(model, "arm", rng);
    const Eigen::Isometry3d target = fk(model, "arm", q, "tool_link", "base_link");
    const auto seed = randomGroupConfig(model, "arm", rng);

    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes err;
    if (!plugin.searchPositionIK(toPose(target), seed, kTimeout, solution, err))
      continue;
    ++solved;
    const Eigen::Isometry3d reached = fk(model, "arm", solution, "tool_link", "base_link");
    EXPECT_LT(positionError(target, reached), kPosTol);
    EXPECT_LT(rotationError(target, reached), kRotTol);
  }
  EXPECT_GE(static_cast<double>(solved) / kSamples, kSolveRateFloor)
      << "solved " << solved << " of " << kSamples;
}
