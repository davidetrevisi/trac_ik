/********************************************************************************
Copyright (c) 2015, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/


// Jazzy's urdf 2.10.1 has no <urdf/model.hpp>; on Kilted and Rolling <urdf/model.h> is a shim that
// warns. Delete this guard, keeping the .hpp, when Jazzy reaches EOL (May 2029) -- or sooner, with
// the code it serves: only files that parse a URDF file keep it, and the URDF-reading code here is
// deleted by the tickets that take couplings and joint limits from MoveIt's robot model.
#if __has_include(<urdf/model.hpp>)
#include <urdf/model.hpp>
#else
#include <urdf/model.h>
#endif
#include <tf2_kdl/tf2_kdl.hpp>
#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>
#include <trac_ik/trac_ik_kinematics_plugin.hpp>
#include <limits>
#include <moveit/robot_state/robot_state.hpp>
#include <trac_ik_kinematics_plugin/trac_ik_kinematics_plugin_parameters.hpp>

namespace trac_ik_kinematics_plugin
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("trac_ik_kinematics_plugin.trac_ik_kinematics_plugin");

bool TRAC_IKKinematicsPlugin::initialize(const rclcpp::Node::SharedPtr &node,
    const moveit::core::RobotModel& robot_model,
    const std::string& group_name,
    const std::string& base_frame,
    const std::vector<std::string>& tip_frames,
    double search_discretization)
{
  node_ = node;

  // Get Solver Parameters
  std::string kinematics_param_prefix = "robot_description_kinematics." + group_name;
  param_listener_ = std::make_shared<trac_ik_kinematics::ParamListener>(node, kinematics_param_prefix);
  params_ = std::make_shared<trac_ik_kinematics::Params>(param_listener_->get_params());

  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(*robot_model.getURDF(), tree))
  {
    RCLCPP_FATAL(LOGGER, "Failed to extract kdl tree from xml robot description");
    return false;
  }

  if (tip_frames.size() != 1)
  {
    RCLCPP_FATAL(LOGGER, "Tip frames has a size different than 1");
    return false;
  }

  if (!tree.getChain(base_frame, tip_frames[0], chain))
  {
    RCLCPP_FATAL(LOGGER, "Couldn't find chain %s to %s", base_frame.c_str(), tip_frames[0].c_str());
    return false;
  }

  num_joints_ = chain.getNrOfJoints();

  std::vector<KDL::Segment> chain_segs = chain.segments;

  urdf::JointConstSharedPtr joint;

  std::vector<double> l_bounds, u_bounds;

  joint_min.resize(num_joints_);
  joint_max.resize(num_joints_);

  uint joint_num = 0;
  for (unsigned int i = 0; i < chain_segs.size(); ++i)
  {

    link_names_.push_back(chain_segs[i].getName());
    joint = robot_model_->getURDF()->getJoint(chain_segs[i].getJoint().getName());
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      joint_num++;

      // The correspondence everything after this point assumes: chain joint i holds configuration
      // entry i, on both sides of this seam. It is not free. joint_names_ takes one name per
      // non-fixed URDF joint, while num_joints_ counts the KDL joints that have a VARIABLE, and
      // kdl_parser has no planar or floating joint -- it converts one to a fixed joint. Such a joint
      // takes a name and no variable, so from there on the two disagree: the write below runs off
      // the end of the bounds, couplings attach to the wrong joint, and the solver joint list comes
      // out longer than the configuration MoveIt writes a solution back into, which is how a
      // kinematics plugin corrupts memory in a release build. So: refuse the robot, naming the
      // joint. (This is where upstream asserted the count, which says nothing in a release build
      // and nothing about which joint was at fault.)
      const moveit::core::JointModel* joint_model = robot_model.getJointModel(joint->name);
      if (joint_model && joint_model->getVariableCount() != 1)
      {
        RCLCPP_FATAL_STREAM(LOGGER, "Chain joint " << joint->name << " has "
                            << joint_model->getVariableCount() << " variables in the robot model;"
                            << " this solver can only index a joint that has exactly one");
        return false;
      }
      if (joint_num > num_joints_)
      {
        RCLCPP_FATAL_STREAM(LOGGER, "The chain " << base_frame_ << " -> " << tip_frames_[0]
                            << " has more movable joints than the " << num_joints_ << " joint"
                            << " variables KDL gives it; a configuration entry cannot be matched to"
                            << " a joint");
        return false;
      }

      float lower, upper;
      int hasLimits;
      joint_names_.push_back(joint->name);
      if (joint->type != urdf::Joint::CONTINUOUS)
      {
        if (joint->safety)
        {
          lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
          upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
        }
        else
        {
          lower = joint->limits->lower;
          upper = joint->limits->upper;
        }
        hasLimits = 1;
      }
      else
      {
        hasLimits = 0;
      }
      if (hasLimits)
      {
        joint_min(joint_num - 1) = lower;
        joint_max(joint_num - 1) = upper;
      }
      else
      {
        joint_min(joint_num - 1) = std::numeric_limits<float>::lowest();
        joint_max(joint_num - 1) = std::numeric_limits<float>::max();
      }
      RCLCPP_INFO_STREAM(LOGGER, "IK Using joint " << chain_segs[i].getName() << " " << joint_min(joint_num - 1) << " " << joint_max(joint_num - 1));
    }
  }

  if (!buildCouplings(robot_model, group_name))
    return false;

  position_ik_ = params_->position_only_ik;
  solve_type = params_->solve_type;
  RCLCPP_INFO(LOGGER, "Using solve type %s", solve_type.c_str());

  rng_ = std::make_shared<random_numbers::RandomNumberGenerator>();

  active_ = true;
  return true;
}


// How the chain's joints are coupled, read off MoveIt's robot model rather than re-parsed from the
// URDF text: the model is what MoveIt re-imposes on our solution afterwards (updateMimicJoints), it
// has already flattened mimic-of-mimic chains, and it carries joint_limits.yaml overrides. Matching
// it is what makes a solution stable under MoveIt's own post-processing.
//
// The plugin wires and validates; it does not map. Everything that can be checked once is checked
// here, so a robot this fork cannot solve correctly fails to load instead of returning -1 from every
// CartToJnt, or -- for a chain joint missing from the group -- being dropped by MoveIt with one log
// line and no hint which joint was at fault.
bool TRAC_IKKinematicsPlugin::buildCouplings(const moveit::core::RobotModel& robot_model,
    const std::string& group_name)
{
  const moveit::core::JointModelGroup* group = robot_model.getJointModelGroup(group_name);
  if (!group)
  {
    RCLCPP_FATAL_STREAM(LOGGER, "No group " << group_name << " in the robot model");
    return false;
  }

  couplings_.assign(num_joints_, TRAC_IK::JointCoupling());
  for (uint i = 0; i < num_joints_; ++i)
  {
    const std::string& name = joint_names_[i];
    const moveit::core::JointModel* joint = robot_model.getJointModel(name);
    if (!joint)
    {
      RCLCPP_FATAL_STREAM(LOGGER, "Chain joint " << name << " is not in the robot model");
      return false;
    }

    // MoveIt maps a seed and a solution onto this solver by name, over the GROUP's joints. A chain
    // joint the group does not list has nowhere to come from or go to, and today MoveIt answers
    // that by dropping the whole solver with one line naming nothing.
    if (!group->hasJointModel(name))
    {
      RCLCPP_FATAL_STREAM(LOGGER, "Chain joint " << name << " is not a joint of group " << group_name
                          << "; add it to the group in your SRDF (a <chain> tag alone does not"
                          << " include a joint whose parent link is outside the chain)");
      return false;
    }

    couplings_[i].name = name;
    couplings_[i].variable_count = static_cast<int>(joint->getVariableCount());

    const moveit::core::JointModel* mimicked = joint->getMimic();
    if (!mimicked)
      continue;

    const auto it = std::find(joint_names_.begin(), joint_names_.end(), mimicked->getName());
    if (it == joint_names_.end())
    {
      // Nothing downstream can even encode this: the description indexes the chain, and there is no
      // index to point at. Solving it anyway would mean moving a joint whose value we do not know.
      RCLCPP_FATAL_STREAM(LOGGER, "Chain joint " << name << " mimics " << mimicked->getName()
                          << ", which is outside the chain " << base_frame_ << " -> " << tip_frames_[0]);
      return false;
    }
    couplings_[i].mimicked_index = static_cast<int>(std::distance(joint_names_.begin(), it));
    couplings_[i].multiplier = joint->getMimicFactor();
    couplings_[i].offset = joint->getMimicOffset();
  }

  if (!foldOutOfChainMimicBounds(robot_model, *group))
    return false;

  // One throwaway value validates the description: a variable count other than one, a non-finite
  // multiplier, and a joint that follows a mimic joint or itself are all its refusals, and it names
  // the offending joint. The plugin keeps the plain description and hands a fresh JointCouplings to
  // each solver instance.
  const TRAC_IK::JointCouplings couplings(couplings_);
  if (!couplings.valid())
  {
    RCLCPP_FATAL_STREAM(LOGGER, "Cannot solve for group " << group_name << ": " << couplings.error());
    return false;
  }
  // ...and tightening a throwaway copy of the bounds asks it the one question left: whether any
  // configuration satisfies this mechanism at all. The answer is discarded -- the library tightens
  // for itself, per instance -- but asking it here is what turns "-1 from every call, for the whole
  // timeout" into one refusal at load naming the joint.
  KDL::JntArray lb(joint_min), ub(joint_max);
  std::string why;
  if (!couplings.tighten(lb, ub, why))
  {
    RCLCPP_FATAL_STREAM(LOGGER, "Cannot solve for group " << group_name << ": " << why);
    return false;
  }

  for (uint i = 0; i < num_joints_; ++i)
    if (couplings_[i].mimicked_index >= 0)
      RCLCPP_INFO(LOGGER, "%s mimics %s (x%.3f %+.3f)", joint_names_[i].c_str(),
                  joint_names_[couplings_[i].mimicked_index].c_str(), couplings_[i].multiplier,
                  couplings_[i].offset);
  RCLCPP_INFO(LOGGER, "Group %s: %u chain joints, %u of them active", group_name.c_str(), num_joints_,
              couplings.reducedSize());
  return true;
}


// A mimic joint OUTSIDE the chain, inside the group: a chain joint drives it, the library is handed
// a chain that does not contain it and cannot know it exists, and MoveIt writes it from our solution
// whatever that does to its own bounds. So the bounds handed to the library are pre-tightened here.
// The two tightenings compose because both are intersections: MoveIt's bounds -> the plugin folds the
// out-of-chain mimic joints -> the library folds the couplings inside the chain.
//
// Its bounds come from the robot model, while the chain's own still come from the URDF text a few
// lines above. That is not an inconsistency worth a second URDF walk: the model is the source ticket
// 08 moves everything to, and this is the only place a joint outside the chain can be read at all.
bool TRAC_IKKinematicsPlugin::foldOutOfChainMimicBounds(const moveit::core::RobotModel& robot_model,
    const moveit::core::JointModelGroup& group)
{
  for (uint i = 0; i < num_joints_; ++i)
  {
    const moveit::core::JointModel* joint = robot_model.getJointModel(joint_names_[i]);
    for (const moveit::core::JointModel* mimic : joint->getMimicRequests())
    {
      const std::string& mimic_name = mimic->getName();
      if (std::find(joint_names_.begin(), joint_names_.end(), mimic_name) != joint_names_.end())
        continue;  // in the chain: the library folds it, through the effective bounds it computes
      if (!group.hasJointModel(mimic_name))
        continue;  // outside the group too: not a joint this solver is asked to answer for
      if (mimic->getVariableCount() != 1)
      {
        RCLCPP_FATAL_STREAM(LOGGER, "Joint " << mimic_name << " mimics chain joint " << joint_names_[i]
                            << " but has " << mimic->getVariableCount() << " variables; exactly one"
                            << " is required to bound the joint it follows");
        return false;
      }

      const moveit::core::VariableBounds& b = mimic->getVariableBounds()[0];
      const double mimic_lb = b.position_bounded_ ? b.min_position_ : -std::numeric_limits<double>::infinity();
      const double mimic_ub = b.position_bounded_ ? b.max_position_ : std::numeric_limits<double>::infinity();

      std::string why;
      if (!TRAC_IK::tightenThroughCoupling(mimic->getMimicFactor(), mimic->getMimicOffset(),
                                           mimic_lb, mimic_ub, joint_min(i), joint_max(i), why))
      {
        RCLCPP_FATAL_STREAM(LOGGER, "Joint " << mimic_name << ", which mimics chain joint "
                            << joint_names_[i] << " from outside the chain, " << why);
        return false;
      }
      RCLCPP_INFO_STREAM(LOGGER, mimic_name << " mimics " << joint_names_[i] << " from outside the"
                         << " chain, bounding it to " << joint_min(i) << " .. " << joint_max(i));
    }
  }
  return true;
}


int TRAC_IKKinematicsPlugin::getKDLSegmentIndex(const std::string &name) const
{
  int i = 0;
  while (i < (int)chain.getNrOfSegments())
  {
    if (chain.getSegment(i).getName() == name)
    {
      return i + 1;
    }
    i++;
  }
  return -1;
}


bool TRAC_IKKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
    const std::vector<double> &joint_angles,
    std::vector<geometry_msgs::msg::Pose> &poses) const
{
  if (!active_)
  {
    RCLCPP_ERROR(LOGGER, "kinematics not active");
    return false;
  }
  poses.resize(link_names.size());
  if (joint_angles.size() != num_joints_)
  {
    RCLCPP_ERROR(LOGGER, "Joint angles vector must have size: %d", num_joints_);
    return false;
  }

  KDL::Frame p_out;

  KDL::JntArray jnt_pos_in(num_joints_);
  for (unsigned int i = 0; i < num_joints_; i++)
  {
    jnt_pos_in(i) = joint_angles[i];
  }

  // Forward kinematics is the one plugin path with no library call under it, so it is the one place
  // the plugin repairs a configuration: MoveIt does not promise the mimic entries it hands us are
  // consistent, and the pose of a configuration that violates the robot's own couplings is not a
  // pose the robot has. Silently, because a caller that simply leaves those entries alone is doing
  // nothing wrong and a warning would fire on every call. A mimicked joint is never itself a mimic
  // joint, so reading the input value is right whichever end of the chain it sits at.
  for (unsigned int i = 0; i < num_joints_; i++)
    if (couplings_[i].mimicked_index >= 0)
      jnt_pos_in(i) = couplings_[i].multiplier * joint_angles[couplings_[i].mimicked_index] +
                      couplings_[i].offset;

  KDL::ChainFkSolverPos_recursive fk_solver(chain);

  bool valid = true;
  for (unsigned int i = 0; i < poses.size(); i++)
  {
    RCLCPP_DEBUG(LOGGER, "End effector index: %d", getKDLSegmentIndex(link_names[i]));
    if (fk_solver.JntToCart(jnt_pos_in, p_out, getKDLSegmentIndex(link_names[i])) >= 0)
    {
      poses[i] = tf2::toMsg(p_out);
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Could not compute FK for %s", link_names[i].c_str());
      valid = false;
    }
  }

  return valid;
}


bool TRAC_IKKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    std::vector<double> &solution,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          default_timeout_,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    std::vector<double> &solution,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    const std::vector<double> &consistency_limits,
    std::vector<double> &solution,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    std::vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    const std::vector<double> &consistency_limits,
    std::vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    std::vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const std::vector<double> &consistency_limits,
    const kinematics::KinematicsQueryOptions &options) const
{
  RCLCPP_DEBUG_STREAM(LOGGER, "getPositionIK");

  if (!active_)
  {
    RCLCPP_ERROR(LOGGER, "kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if (ik_seed_state.size() != num_joints_)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Seed state must have size " << num_joints_ << " instead of size " << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  KDL::Frame frame;
  tf2::fromMsg(ik_pose, frame);

  KDL::JntArray in(num_joints_), out(num_joints_);

  for (uint z = 0; z < num_joints_; z++)
    in(z) = ik_seed_state[z];

  TRAC_IK::Query query;
  query.epsilon = params_->epsilon;

  if (position_ik_)
  {
    query.tolerance_bounds.rot.x(std::numeric_limits<float>::max());
    query.tolerance_bounds.rot.y(std::numeric_limits<float>::max());
    query.tolerance_bounds.rot.z(std::numeric_limits<float>::max());
  }

  if (solve_type == "Manipulation1")
    query.solve_type = TRAC_IK::Manip1;
  else if (solve_type == "Manipulation2")
    query.solve_type = TRAC_IK::Manip2;
  else if (solve_type == "Manipulation3")
    query.solve_type = TRAC_IK::Manip3;
  else if (solve_type == "Distance")
    query.solve_type = TRAC_IK::Distance;
  else
  {
    if (solve_type != "Speed")
    {
      RCLCPP_WARN_STREAM(LOGGER, solve_type << " is not a valid solve_type; setting to default: Speed");
    }
    query.solve_type = TRAC_IK::Speed;
  }

  // One solver for the whole call: the mechanism does not change between retries, only the budget
  // left to spend on the next one, and that rides on the query. The couplings were read off the
  // robot model and validated at load, so this cannot fail for a robot that initialised.
  TRAC_IK::TRAC_IK ik_solver(chain, joint_min, joint_max, TRAC_IK::JointCouplings(couplings_),
                             node_->get_logger());

  auto end_time = std::chrono::system_clock::now() + std::chrono::duration<double>(timeout);
  while (std::chrono::system_clock::now() < end_time)
  {
    query.timeout = std::chrono::duration<double>(end_time - std::chrono::system_clock::now()).count();

    int rc = ik_solver.CartToJnt(in, frame, out, query);

    // If you want to retrieve all the returned solutions, the (commented) code below does it
    // Note that you have to call getSolutions() AFTER a successful code to CartToJnt to get all the solutions generated
    // CartToJnt returns only one solution, but more could have been generated
    // usually, Speed returns 1 solution.  The other modes return more
    // rc is the number of solutions obtained
    /*
    if(rc > 0)
    {
      std::vector<KDL::JntArray> sols;
      bool res = ik_solver.getSolutions(sols);
      RCLCPP_WARN(LOGGER, "Generated %u solutions, retrieved %lu solutions ", rc, sols.size());
    }*/

    solution.resize(num_joints_);

    if (rc >= 0)
    {
      for (uint z = 0; z < num_joints_; z++)
        solution[z] = out(z);

      // check for collisions if a callback is provided
      if (solution_callback)
      {
        solution_callback(ik_pose, solution, error_code);
        if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
          RCLCPP_DEBUG_STREAM(LOGGER, "Solution passes callback");
          return true;
        }
        else
        {
          RCLCPP_DEBUG_STREAM(LOGGER, "Solution has error code " << error_code.val);
          std::vector<double> random_state(ik_seed_state);
          robot_model_->getVariableRandomPositions(*rng_, random_state);
          for (uint z = 0; z < num_joints_; z++)
            in(z) = random_state[z];
          RCLCPP_DEBUG_STREAM(LOGGER, "Retrying with new seed");
        }
      }
      else
        return true; // no collision check callback provided
    }
  }

  error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
  return false;
}



} // end namespace

//register TRAC_IKKinematicsPlugin as a KinematicsBase implementation
#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin, kinematics::KinematicsBase);
