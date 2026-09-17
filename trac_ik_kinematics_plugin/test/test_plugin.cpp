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
//
// Four fixtures, shared with the library seam and found through the TRAC_IK_TEST_FIXTURE_DIR compile
// definition: the mesh-stripped crane (9 joints, 4 mimic, 5 active), arm6 (a plain 6-DOF arm),
// arm7 (redundant 7R) and mimic_arm (one revolute mimic, negative multiplier, nonzero offset).

#include <gtest/gtest.h>

#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <trac_ik/trac_ik_kinematics_plugin.hpp>
#include <urdf_parser/urdf_parser.h>

#include <cstdlib>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace
{

constexpr double kPosTol = 1e-3;    // metres
constexpr double kRotTol = 1e-3;    // radians
constexpr int kSamples = 50;
constexpr double kSolveRateFloor = 0.90;  // generous: tighten once ticket 10 seeds the RNG

// The per-solve budget, in seconds. Defaults to MoveIt's own kinematics_solver_timeout, which is
// what a user's robot gets; CI sets TRAC_IK_TEST_SOLVE_BUDGET to a looser value, because a shared
// runner is slower and noisier than the machine these bounds were chosen on and should fail on
// behaviour, not on wall-clock. No test asserts a duration, so raising it can only help a solve --
// up to the target's own ctest timeout, which is why the value is range-checked.
double solveBudget()
{
  static const double budget = []() {
    constexpr double kDefault = 0.05;
    const char* env = std::getenv("TRAC_IK_TEST_SOLVE_BUDGET");
    if (!env)
      return kDefault;
    // strtod, not atof: atof accepts trailing garbage, so "200ms" would parse as a 200-second
    // budget, quietly blow through the target's own 300 s ctest timeout and kill the cell with no
    // hint that the variable is at fault. A mistyped budget must fail as a misconfiguration.
    char* end = nullptr;
    const double parsed = std::strtod(env, &end);
    // Upper bound because the budget multiplies out: the plugin's sampled tests spend it on all 50
    // samples, so 1 s is already 150 s of the target's 300 s ctest TIMEOUT. Past that the cell dies
    // on wall-clock, which is the failure mode this variable exists to prevent.
    const bool ok = end && *end == '\0' && end != env && parsed > 0.0 && parsed <= 1.0;
    EXPECT_TRUE(ok) << "TRAC_IK_TEST_SOLVE_BUDGET must be a bare number of seconds in (0, 1]: " << env;
    return ok ? parsed : kDefault;
  }();
  return budget;
}

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
  //
  // With a rejection step, because MoveIt samples only the group's ACTIVE joints and then overwrites
  // the mimic ones, without checking that the result lands inside the mimic joint's own limits. On
  // mimic_arm it often does not -- j3's limits are deliberately tighter than j2's map onto, so a
  // third of all draws put j3 out of range (662 of 2000, measured 2026-09-15) -- and such a
  // configuration is not reachable by any solver that respects j3's bounds, so sampling it would
  // make the solve-rate floor measure the fixture instead of the solver.
  //
  // The check is per joint and NOT satisfiesBounds(jmg), which walks the group's ACTIVE joints only:
  // mimic joints are excluded from that vector, so the group-wide overload cannot see the one thing
  // this loop exists to catch. Measured: it reported 0 of those same 662 draws out of bounds, and
  // the solve rate here sat at exactly the floor, 45 of 50, passing on one sample. Per joint: 50.
  moveit::core::RobotState state(model);
  const auto* jmg = model->getJointModelGroup(group);
  random_numbers::RandomNumberGenerator rn(rng());
  for (int attempt = 0;; ++attempt)
  {
    state.setToDefaultValues();
    state.setToRandomPositions(jmg, rn);
    state.update();
    bool in_bounds = true;
    for (const auto* joint : jmg->getJointModels())
      in_bounds = in_bounds && state.satisfiesBounds(joint);
    if (in_bounds)
      break;
    if (attempt == 99)
    {
      ADD_FAILURE() << "no in-bounds configuration for group " << group << " in 100 draws: its mimic"
                    << " bounds and its mimicked joints' cannot be satisfied together";
      break;
    }
  }
  std::vector<double> q;
  state.copyJointGroupPositions(group, q);
  return q;
}

// Apply a solution the way MoveIt does: by name, over the solver joint list. Setting a mimicked
// joint re-imposes every coupling that follows it, including one on a joint outside the chain, which
// the solver never saw and the caller never named.
moveit::core::RobotState applySolution(const moveit::core::RobotModelPtr& model,
                                       const std::vector<std::string>& names,
                                       const std::vector<double>& solution)
{
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  for (size_t i = 0; i < names.size(); ++i)
    state.setVariablePosition(names[i], solution[i]);
  state.update();
  return state;
}

// A group configuration restricted to the solver joint list: the shape of the seed MoveIt hands the
// plugin. The two differ whenever the group holds a joint the chain does not reach.
std::vector<double> seedFromGroupConfig(const moveit::core::RobotModelPtr& model, const std::string& group,
                                        const std::vector<double>& group_config,
                                        const std::vector<std::string>& names)
{
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  state.setJointGroupPositions(group, group_config);
  state.update();
  std::vector<double> out;
  out.reserve(names.size());
  for (const auto& name : names)
    out.push_back(state.getVariablePosition(name));
  return out;
}

// Every coupling among the solver joint list's joints, checked against the returned configuration:
// MoveIt's overwrite must be a no-op rather than a silent correction.
void expectCouplingsSatisfied(const moveit::core::RobotModelPtr& model,
                              const std::vector<std::string>& names,
                              const std::vector<double>& solution)
{
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
// The measured baseline (ticket 01): a full-pose query from a zero seed reported SUCCESS and landed
// 0.090 m away once MoveIt re-imposed the mimic coupling. Green since ticket 06 wired the couplings
// through to the solver, which is what makes this fork usable on the crane.
// ---------------------------------------------------------------------------------------------

TEST(TracIkPlugin, CraneSolutionSurvivesMimicReimposition)
{
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
  ASSERT_TRUE(plugin.searchPositionIK(toPose(target), seed, solveBudget(), solution, err))
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
  auto model = loadModel("crane.urdf", "crane.srdf");
  // TRAP, pinned deliberately: solve_type Distance, not the shipped default. In Speed mode the KDL
  // branch answers in ~1 ms with a solution whose four parallel prismatic joints happen to come out
  // equal -- their Jacobian columns are identical and repeated limit clipping collapses them
  // together -- so a coupling assertion goes green while the solver knows nothing about mimics.
  // Distance collects solutions for the whole budget and its winner violates the coupling outright.
  // Every crane coupling test pins Distance, and no mimic work may take a green Speed-mode run as
  // evidence.
  auto node = makeNode(CraneCase::kGroup, {rclcpp::Parameter("solve_type", "Distance")});
  trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin plugin;
  ASSERT_TRUE(plugin.initialize(node, *model, CraneCase::kGroup, CraneCase::kBase, {CraneCase::kTip}, 0.005));

  const std::vector<double> S = {0.3, 0.7, 0.5, 0.5, 0.5, 0.5, -0.5, 0.4, 0.4};
  const Eigen::Isometry3d target = fk(model, CraneCase::kGroup, S, CraneCase::kTip, CraneCase::kBase);

  // TRAP, pinned deliberately: NOT a zero seed. From an all-equal seed a pseudo-inverse step moves
  // the four parallel main_boom_ext* joints by equal amounts, for the same symmetry reason, and the
  // coupling violation stays invisible. An asymmetric seed is what exposes it.
  const std::vector<double> seed = {0.1, 0.2, 0.05, 0.4, 0.15, 0.3, -0.2, 0.1, 0.35};
  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes err;
  ASSERT_TRUE(plugin.searchPositionIK(toPose(target), seed, solveBudget(), solution, err));

  // The returned full configuration must already satisfy the coupling, so that MoveIt's overwrite is
  // a no-op rather than a silent correction.
  expectCouplingsSatisfied(model, plugin.getJointNames(), solution);
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
    if (!plugin.searchPositionIK(toPose(target), seed, solveBudget(), solution, err))
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
    if (!plugin.searchPositionIK(toPose(target), seed, solveBudget(), solution, err))
      continue;
    ++solved;
    const Eigen::Isometry3d reached = fk(model, "arm", solution, "tool_link", "base_link");
    EXPECT_LT(positionError(target, reached), kPosTol);
    EXPECT_LT(rotationError(target, reached), kRotTol);
  }
  EXPECT_GE(static_cast<double>(solved) / kSamples, kSolveRateFloor)
      << "solved " << solved << " of " << kSamples;
}

// ---------------------------------------------------------------------------------------------
// TRAP, pinned deliberately: the base frame MoveIt hands the plugin is the model's root link, not
// the chain base named in the SRDF. The crane's SRDF reads `turret_link -> jib_ext_link`, but the
// group also lists `turret_rotation`, whose parent is `base_link`; MoveIt takes the parent link of
// the first group joint, so `base_frame` is `base_link` and the chain has 9 joints
// (research/how-moveit-calls-ik.md). Initialising from `turret_link` instead succeeds and yields a
// silently different, 8-joint solver -- which then rejects every seed MoveIt sends it. Cost one
// build cycle during the prototype; pinned here so no later ticket repeats it.
// ---------------------------------------------------------------------------------------------

TEST(TracIkPlugin, BaseFrameIsTheRootLinkNotTheSrdfChainBase)
{
  auto model = loadModel("crane.urdf", "crane.srdf");
  const auto* jmg = model->getJointModelGroup(CraneCase::kGroup);
  ASSERT_NE(jmg, nullptr);

  // The rule MoveIt applies, and the reason the rest of this suite passes base_link: the base frame
  // is the parent link of the group's first joint, which here is turret_rotation's parent.
  const auto& group_joints = jmg->getActiveJointModels();
  ASSERT_FALSE(group_joints.empty());
  ASSERT_NE(group_joints.front()->getParentLinkModel(), nullptr);
  EXPECT_EQ(group_joints.front()->getParentLinkModel()->getName(), CraneCase::kBase);
  EXPECT_EQ(model->getRootLinkName(), CraneCase::kBase);

  auto node = makeNode(CraneCase::kGroup, {rclcpp::Parameter("solve_type", "Distance")});
  trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin from_root;
  ASSERT_TRUE(from_root.initialize(node, *model, CraneCase::kGroup, CraneCase::kBase, {CraneCase::kTip}, 0.005));
  ASSERT_EQ(from_root.getJointNames().size(), 9u);

  // The SRDF chain base drops turret_rotation: the chain is one joint short and, being short, the
  // 9-value seed MoveIt passes is refused outright rather than solved against the wrong joint set.
  auto other_node = makeNode(CraneCase::kGroup, {rclcpp::Parameter("solve_type", "Distance")});
  trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin from_chain_base;
  ASSERT_TRUE(from_chain_base.initialize(other_node, *model, CraneCase::kGroup, "turret_link",
                                         {CraneCase::kTip}, 0.005));
  EXPECT_EQ(from_chain_base.getJointNames().size(), 8u);

  const std::vector<double> S = {0.3, 0.7, 0.5, 0.5, 0.5, 0.5, -0.5, 0.4, 0.4};
  const Eigen::Isometry3d target = fk(model, CraneCase::kGroup, S, CraneCase::kTip, CraneCase::kBase);
  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes err;
  EXPECT_FALSE(from_chain_base.searchPositionIK(toPose(target), S, solveBudget(), solution, err));
  EXPECT_EQ(err.val, moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION);
}

// A load-time failure, the cheapest kind for a user to hit: a tip frame that is not in the model.
// initialize must say no rather than come up half-built, because MoveIt drops the solver on a false
// return and logs it, while a half-built one fails later with no hint why.
TEST(TracIkPlugin, InitializeRejectsATipFrameOutsideTheModel)
{
  auto model = loadModel("arm6.urdf", "arm6.srdf");
  auto node = makeNode("arm");
  trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin plugin;
  EXPECT_FALSE(plugin.initialize(node, *model, "arm", "base_link", {"no_such_link"}, 0.005));
}

// ---------------------------------------------------------------------------------------------
// mimic_arm (wayfinder ticket 05): one revolute mimic at a negative multiplier and a nonzero
// offset, its own limits tighter than its mimicked joint's map onto. Nothing else in the suite
// covers a coupling that is not "prismatic, x1, +0", so a sign error in the bound mapping or in the
// folded Jacobian would be invisible. The behavioural assertions arrive with tickets 04 and 05;
// what is pinned here is the contract the plugin must already honour: the solver joint list is the
// group's joints with variables, the mimic joint included, and the couplings the plugin will read
// come from the RobotModel, not from the URDF text.
// ---------------------------------------------------------------------------------------------

TEST(TracIkPlugin, MimicArmSolverJointListIncludesTheMimicJoint)
{
  auto model = loadModel("mimic_arm.urdf", "mimic_arm.srdf");
  auto node = makeNode("arm");
  trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin plugin;
  ASSERT_TRUE(plugin.initialize(node, *model, "arm", "base_link", {"tool_link"}, 0.005));

  const std::vector<std::string> expected = {"j1", "j2", "j3", "j4"};
  EXPECT_EQ(plugin.getJointNames(), expected) << "the mimic joint j3 is part of the solver joint list";
}

TEST(TracIkPlugin, MimicArmCouplingIsNegativeAndOffsetInTheRobotModel)
{
  auto model = loadModel("mimic_arm.urdf", "mimic_arm.srdf");
  const auto* j3 = model->getJointModel("j3");
  ASSERT_NE(j3, nullptr);
  const auto* mimicked = j3->getMimic();
  ASSERT_NE(mimicked, nullptr) << "RobotModel, not the URDF text, is where couplings come from";
  EXPECT_EQ(mimicked->getName(), "j2");
  EXPECT_DOUBLE_EQ(j3->getMimicFactor(), -0.5);
  EXPECT_DOUBLE_EQ(j3->getMimicOffset(), 0.3);

  // RobotState is the oracle for what MoveIt does with that coupling: it overwrites j3 whatever the
  // solver returns, so a solution that does not already satisfy it is silently changed underneath
  // the caller. This is the mechanism the crane tests measure, on a robot whose multiplier is
  // neither 1 nor positive.
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  std::vector<double> q = {0.2, 0.8, 0.0, -0.3};  // j3 deliberately inconsistent
  state.setJointGroupPositions("arm", q);
  state.update();
  std::vector<double> applied;
  state.copyJointGroupPositions("arm", applied);
  EXPECT_NEAR(applied[2], -0.5 * applied[1] + 0.3, 1e-9) << "MoveIt re-imposes the coupling";
}

TEST(TracIkPlugin, MimicArmPositionOnlyQueriesAreAnswered)
{
  auto model = loadModel("mimic_arm.urdf", "mimic_arm.srdf");
  // Three active joints against a three-dimensional position goal: orientation must be free for the
  // query to stay solvable once tickets 04 and 05 give the solver 3 variables instead of 4.
  auto node = makeNode("arm", {rclcpp::Parameter("position_only_ik", true)});
  trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin plugin;
  ASSERT_TRUE(plugin.initialize(node, *model, "arm", "base_link", {"tool_link"}, 0.005));

  // Reachable targets, so the floor measures the solver and not the fixture. The pose assertion is
  // the point: before ticket 06 the solver chose j3 for itself, MoveIt overwrote it on the way back,
  // and the position landed up to 0.18 m off (measured on the VM, 2026-09-15) -- the crane defect on
  // a negative multiplier. Only position is checked, because orientation is freed by the query.
  std::mt19937 rng(12);
  int solved = 0;
  for (int n = 0; n < kSamples; ++n)
  {
    const auto q = randomGroupConfig(model, "arm", rng);
    const Eigen::Isometry3d target = fk(model, "arm", q, "tool_link", "base_link");
    const auto seed = randomGroupConfig(model, "arm", rng);

    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes err;
    if (!plugin.searchPositionIK(toPose(target), seed, solveBudget(), solution, err))
    {
      EXPECT_EQ(err.val, moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION);
      continue;
    }
    ++solved;
    ASSERT_EQ(solution.size(), 4u);
    const Eigen::Isometry3d reached = fk(model, "arm", solution, "tool_link", "base_link");
    EXPECT_LT(positionError(target, reached), kPosTol)
        << "the position after MoveIt re-imposes j3's coupling";
    // err is deliberately not asserted on the success path: today the plugin returns true without
    // touching error_code when no validity callback is given, so the caller reads whatever it
    // passed in. Measured here, 2026-09-15; ticket 11 owns the per-call error codes and is where
    // that assertion belongs, red-first.
  }
  EXPECT_GE(static_cast<double>(solved) / kSamples, kSolveRateFloor)
      << "solved " << solved << " of " << kSamples;
}

// ---------------------------------------------------------------------------------------------
// What initialisation refuses (implementation ticket 06). The couplings are read off MoveIt's robot
// model and validated once at load, so a robot this fork cannot answer for correctly fails to load
// with the joint named, instead of returning -1 from every call or being dropped by MoveIt silently.
//
// The other refusals -- a mimicked joint outside the chain, a variable count other than one, a
// non-finite multiplier -- are the JointCouplings value's, and the library suite exercises them
// there: each needs a contrived robot description, and the plugin's own path to them is the same
// throwaway value.
// ---------------------------------------------------------------------------------------------

TEST(TracIkPlugin, InitializeRejectsAChainJointOutsideTheGroup)
{
  auto model = loadModel("crane.urdf", "crane_group_missing_joint.srdf");
  const auto* jmg = model->getJointModelGroup(CraneCase::kGroup);
  ASSERT_NE(jmg, nullptr);
  ASSERT_FALSE(jmg->hasJointModel("turret_rotation")) << "the fixture's whole point";

  auto node = makeNode(CraneCase::kGroup, {rclcpp::Parameter("solve_type", "Distance")});
  trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin plugin;
  EXPECT_FALSE(plugin.initialize(node, *model, CraneCase::kGroup, CraneCase::kBase, {CraneCase::kTip}, 0.005))
      << "a chain joint the group does not list has nowhere for a seed to come from; MoveIt's own"
         " answer is to drop the solver with one line naming nothing";
}

TEST(TracIkPlugin, InitializeRejectsAChainJointWithMoreThanOneVariable)
{
  auto model = loadModel("planar_arm.urdf", "planar_arm.srdf");
  const auto* j2 = model->getJointModel("j2");
  ASSERT_NE(j2, nullptr);
  ASSERT_EQ(j2->getVariableCount(), 3u) << "MoveIt gives a planar joint three variables";

  auto node = makeNode("arm");
  trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin plugin;
  EXPECT_FALSE(plugin.initialize(node, *model, "arm", "base_link", {"tool_link"}, 0.005))
      << "kdl_parser turned j2 into a fixed joint, so a chain index and a configuration entry no"
         " longer denote the same joint; a solver joint list longer than the configuration is how a"
         " kinematics plugin corrupts memory in a release build";
}

// ---------------------------------------------------------------------------------------------
// The SRDF's passive_joint marking means nothing here (wayfinder ticket 05, answer 11): it marks
// "not actuated", a different property from "value determined by another joint". The crane's four
// entries coincide with its four mimic joints and nothing may depend on that coincidence.
// ---------------------------------------------------------------------------------------------

TEST(TracIkPlugin, PassiveJointMarkingChangesNothing)
{
  auto marked = loadModel("crane.urdf", "crane.srdf");
  auto plain = loadModel("crane.urdf", "crane_no_passive.srdf");

  auto marked_node = makeNode(CraneCase::kGroup, {rclcpp::Parameter("solve_type", "Distance")});
  trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin from_marked;
  ASSERT_TRUE(from_marked.initialize(marked_node, *marked, CraneCase::kGroup, CraneCase::kBase,
                                     {CraneCase::kTip}, 0.005));

  auto plain_node = makeNode(CraneCase::kGroup, {rclcpp::Parameter("solve_type", "Distance")});
  trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin from_plain;
  ASSERT_TRUE(from_plain.initialize(plain_node, *plain, CraneCase::kGroup, CraneCase::kBase,
                                    {CraneCase::kTip}, 0.005));

  // The two models really do differ in the one property under test -- crane_no_passive.srdf is
  // crane.srdf with its four <passive_joint> lines removed and nothing else -- and they agree on
  // every mimic relation regardless, because MoveIt derives those from the URDF's <mimic> tags.
  ASSERT_EQ(marked->getSRDF()->getPassiveJoints().size(), 4u);
  ASSERT_TRUE(plain->getSRDF()->getPassiveJoints().empty());
  for (const char* name : { "main_boom_ext_2", "main_boom_ext_3", "main_boom_ext_4", "jib_ext_2" })
  {
    const auto* marked_mimicked = marked->getJointModel(name)->getMimic();
    const auto* plain_mimicked = plain->getJointModel(name)->getMimic();
    ASSERT_NE(marked_mimicked, nullptr);
    ASSERT_NE(plain_mimicked, nullptr) << name << " is a mimic joint whether or not it is marked passive";
    EXPECT_EQ(marked_mimicked->getName(), plain_mimicked->getName());
    EXPECT_EQ(marked->getJointModel(name)->getMimicFactor(), plain->getJointModel(name)->getMimicFactor());
  }

  EXPECT_EQ(from_marked.getJointNames(), from_plain.getJointNames());

  // And the couplings still bind: same query as CraneSolutionRespectsMimicCoupling, on the model
  // whose SRDF says nothing about passive joints.
  const std::vector<double> S = {0.3, 0.7, 0.5, 0.5, 0.5, 0.5, -0.5, 0.4, 0.4};
  const Eigen::Isometry3d target = fk(plain, CraneCase::kGroup, S, CraneCase::kTip, CraneCase::kBase);
  const std::vector<double> seed = {0.1, 0.2, 0.05, 0.4, 0.15, 0.3, -0.2, 0.1, 0.35};
  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes err;
  ASSERT_TRUE(from_plain.searchPositionIK(toPose(target), seed, solveBudget(), solution, err));
  expectCouplingsSatisfied(plain, from_plain.getJointNames(), solution);
  const Eigen::Isometry3d reached = fk(plain, CraneCase::kGroup, solution, CraneCase::kTip, CraneCase::kBase);
  EXPECT_LT(positionError(target, reached), kPosTol);
  EXPECT_LT(rotationError(target, reached), kRotTol);
}

// ---------------------------------------------------------------------------------------------
// mimic_arm_branch (implementation ticket 06): j5 mimics j2 from OUTSIDE the chain, inside the
// group. The library is handed a chain that does not contain j5 and cannot know it exists, so the
// plugin is what must fold j5's bounds into j2's before the solver ever runs. Without it the solve
// succeeds, MoveIt re-imposes the coupling, j5 lands outside its own bounds, and the planner rejects
// the state downstream with nothing in the IK return code to explain why.
// ---------------------------------------------------------------------------------------------

TEST(TracIkPlugin, OutOfChainMimicJointBoundsAreFolded)
{
  auto model = loadModel("mimic_arm_branch.urdf", "mimic_arm_branch.srdf");
  // Position-only, as on mimic_arm: three active joints cannot also serve an orientation goal.
  auto node = makeNode("arm", {rclcpp::Parameter("position_only_ik", true)});
  trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin plugin;
  ASSERT_TRUE(plugin.initialize(node, *model, "arm", "base_link", {"tool_link"}, 0.005));

  const std::vector<std::string> expected = {"j1", "j2", "j3", "j4"};
  EXPECT_EQ(plugin.getJointNames(), expected) << "j5 is in the group but not in the chain";
  const auto* j5 = model->getJointModel("j5");
  ASSERT_NE(j5, nullptr);

  // Poses the mechanism cannot hold, and the half of this test that bites. Each is forward
  // kinematics of a configuration whose j2 lies outside the +/-0.2 that j5 can follow: a pose, but
  // not one available to this robot. The honest answers are therefore "no solution" or "a solution
  // whose j5 is in bounds". What the plugin must never do -- and did before the folding -- is
  // report SUCCESS having driven j5 outside its own bounds, because MoveIt writes j5 from j2 afterwards
  // and the planner then rejects the state with nothing in the IK return code to explain it
  // (measured on the VM, 2026-09-17: j2 = 0.900 against j5's own bounds of +/-0.200).
  for (const double j2 : {0.9, -0.9, 0.5, -0.5})
  {
    const std::vector<double> reach = {0.2, j2, -0.5 * j2 + 0.3, -0.3, j2};
    const Eigen::Isometry3d target = fk(model, "arm", reach, "tool_link", "base_link");
    const std::vector<double> seed = {0.0, 0.0, 0.3, 0.0};

    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes err;
    if (!plugin.searchPositionIK(toPose(target), seed, solveBudget(), solution, err))
    {
      // The expected outcome once the bounds are folded, and a fine one: the pose is not available
      // to this mechanism. What this half of the test forbids is the other answer, and it is
      // deliberately one-sided -- with the folding in place there is usually nothing here to check,
      // and without it every one of these four targets came back SUCCESS with j5 out of bounds.
      EXPECT_EQ(err.val, moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION);
      continue;
    }
    const moveit::core::RobotState state = applySolution(model, plugin.getJointNames(), solution);
    EXPECT_TRUE(state.satisfiesBounds(j5))
        << "j2 = " << j2 << " asked for: solved with j2 = " << solution[1] << ", so j5 = "
        << state.getVariablePosition("j5") << ", outside the bounds of a joint the chain drives"
        << " and the library cannot see";
  }

  // And the regression half: targets the mechanism CAN hold are still answered, and answered
  // correctly, once the bounds have been narrowed.
  std::mt19937 rng(21);
  int solved = 0;
  for (int n = 0; n < kSamples; ++n)
  {
    // Sampled through the group with per-joint bound rejection, so j5 -- and therefore j2 -- is in
    // range by construction and every target is one a correct solver can reach.
    const auto q = randomGroupConfig(model, "arm", rng);
    const Eigen::Isometry3d target = fk(model, "arm", q, "tool_link", "base_link");
    const auto seed = seedFromGroupConfig(model, "arm", randomGroupConfig(model, "arm", rng),
                                          plugin.getJointNames());

    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes err;
    if (!plugin.searchPositionIK(toPose(target), seed, solveBudget(), solution, err))
      continue;
    ++solved;
    ASSERT_EQ(solution.size(), 4u);

    const moveit::core::RobotState state = applySolution(model, plugin.getJointNames(), solution);
    EXPECT_TRUE(state.satisfiesBounds(j5))
        << "j5 = " << state.getVariablePosition("j5") << " from j2 = " << solution[1];
    const Eigen::Isometry3d reached =
        state.getGlobalLinkTransform("base_link").inverse() * state.getGlobalLinkTransform("tool_link");
    EXPECT_LT(positionError(target, reached), kPosTol);
  }
  EXPECT_GE(static_cast<double>(solved) / kSamples, kSolveRateFloor)
      << "solved " << solved << " of " << kSamples;
}

// ---------------------------------------------------------------------------------------------
// The two paths that take a configuration FROM the caller. Forward kinematics is the one plugin
// path with no library call under it, so it is the one place the plugin repairs anything; a seed
// goes to the library untouched, because the library repairs it for every caller it has.
// ---------------------------------------------------------------------------------------------

TEST(TracIkPlugin, GetPositionFkRecomputesMimicEntries)
{
  auto model = loadModel("mimic_arm.urdf", "mimic_arm.srdf");
  auto node = makeNode("arm");
  trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin plugin;
  ASSERT_TRUE(plugin.initialize(node, *model, "arm", "base_link", {"tool_link"}, 0.005));

  // j3 is deliberately inconsistent: it follows j2 at -0.5 with a +0.3 offset, so 0.8 asks for
  // -0.1 and the configuration as given is not one this robot can hold. MoveIt promises nothing
  // about those entries, and the pose of a configuration that violates the robot's own couplings is
  // not a pose the robot has.
  const std::vector<double> given = {0.2, 0.8, 0.0, -0.3};
  std::vector<geometry_msgs::msg::Pose> poses;
  ASSERT_TRUE(plugin.getPositionFK({"tool_link"}, given, poses));
  ASSERT_EQ(poses.size(), 1u);
  Eigen::Isometry3d plugin_fk;
  tf2::fromMsg(poses[0], plugin_fk);

  // RobotState re-imposes the coupling, which is exactly the repair being asserted: the two agree
  // only if the plugin recomputed j3 rather than believing the 0.0 it was handed.
  const Eigen::Isometry3d oracle = fk(model, "arm", given, "tool_link", "base_link");
  EXPECT_LT(positionError(oracle, plugin_fk), 1e-9);
  EXPECT_LT(rotationError(oracle, plugin_fk), 1e-9);
}

TEST(TracIkPlugin, AnInconsistentSeedIsTheLibrarysToRepair)
{
  auto model = loadModel("mimic_arm.urdf", "mimic_arm.srdf");
  auto node = makeNode("arm", {rclcpp::Parameter("position_only_ik", true)});
  trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin plugin;
  ASSERT_TRUE(plugin.initialize(node, *model, "arm", "base_link", {"tool_link"}, 0.005));

  const std::vector<double> target_config = {0.3, 0.4, 0.1, -0.2};
  const Eigen::Isometry3d target = fk(model, "arm", target_config, "tool_link", "base_link");

  // The seed's j3 does not follow its j2, which is what a caller who leaves mimic entries alone
  // sends. The plugin passes it through as it stands -- the library recomputes mimic entries and
  // clamps an out-of-bounds mimicked joint for every caller it has, and two repairs would be two
  // places to keep in step -- so what is asserted here is the answer, not the seed.
  const std::vector<double> seed = {0.0, 0.5, 0.0, 0.0};
  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes err;
  ASSERT_TRUE(plugin.searchPositionIK(toPose(target), seed, solveBudget(), solution, err));
  expectCouplingsSatisfied(model, plugin.getJointNames(), solution);
  const Eigen::Isometry3d reached = fk(model, "arm", solution, "tool_link", "base_link");
  EXPECT_LT(positionError(target, reached), kPosTol);
}
