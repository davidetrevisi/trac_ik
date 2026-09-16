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


#include <trac_ik/trac_ik.hpp>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace TRAC_IK
{

namespace
{
// Same size and the same value in every entry: the only question configureSolvers has to answer.
bool sameBounds(const KDL::JntArray& a, const KDL::JntArray& b)
{
  return a.data.size() == b.data.size() && (a.data.size() == 0 || a.data == b.data);
}
}  // namespace

TRAC_IK::TRAC_IK(const KDL::Chain& _chain, const KDL::JntArray& _q_min, const KDL::JntArray& _q_max,
                 const JointCouplings& _couplings, const rclcpp::Logger& _logger):
  logger(_logger),
  initialized(false),
  chain(_chain),
  lb(_q_min),
  ub(_q_max),
  couplings(_couplings)
{
  initialize();
}

void TRAC_IK::failInitialization(const std::string& why)
{
  init_error = why;
  initialized = false;
  RCLCPP_FATAL(logger, "TRAC-IK cannot use this mechanism: %s", why.c_str());
}

void TRAC_IK::configureSolvers(const Query& query)
{
  if (nl_solver && iksolver && sameBounds(query.q_min, solver_lb) && sameBounds(query.q_max, solver_ub))
  {
    // The bounds are baked into the inner solvers at construction, so only a change of bounds costs
    // a rebuild; epsilon is settable and the timeout is set per iteration by runSolver.
    nl_solver->setEps(query.epsilon);
    iksolver->setEps(query.epsilon);
    return;
  }

  solver_lb = query.q_min;
  solver_ub = query.q_max;
  nl_solver.reset(new NLOPT_IK::NLOPT_IK(chain, query.q_min, query.q_max, couplings, query.timeout, query.epsilon, logger));
  iksolver.reset(new KDL::ChainIkSolverPos_TL(chain, query.q_min, query.q_max, couplings, query.timeout, query.epsilon, true, true));
}

void TRAC_IK::classifyJoints(const KDL::JntArray& q_min, const KDL::JntArray& q_max)
{
  types.resize(kinds.size());
  for (uint i = 0; i < kinds.size(); i++)
  {
    if (kinds[i] != KDL::BasicJointType::RotJoint)
    {
      types[i] = kinds[i];
      continue;
    }
    const bool unbounded = q_max(i) >= std::numeric_limits<float>::max() &&
                           q_min(i) <= std::numeric_limits<float>::lowest();
    types[i] = unbounded ? KDL::BasicJointType::Continuous : KDL::BasicJointType::RotJoint;
  }
}

void TRAC_IK::classifyRevolutions()
{
  shares_revolutions.assign(chain.getNrOfJoints(), true);
  for (uint i = 0; i < chain.getNrOfJoints(); ++i)
  {
    if (!couplings.isMimic(i))
      continue;
    const JointCoupling& c = couplings.coupling(i);
    // A whole-number multiplier onto a joint of the same kind carries a revolution to a revolution;
    // a half of one, or a prismatic joint driven by a rotational one, carries it to a real motion.
    const bool whole = std::abs(c.multiplier - std::round(c.multiplier)) <= 1e-12;
    if (!whole || kinds[i] != kinds[c.mimicked_index])
      shares_revolutions[c.mimicked_index] = false;
  }
}

void TRAC_IK::initialize()
{

  assert(chain.getNrOfJoints() == lb.data.size());
  assert(chain.getNrOfJoints() == ub.data.size());

  jacsolver.reset(new KDL::ChainJntToJacSolver(chain));

  for (uint i = 0; i < chain.segments.size(); i++)
  {
    std::string type = chain.segments[i].getJoint().getTypeName();
    if (type.find("Rot") != std::string::npos)
      kinds.push_back(KDL::BasicJointType::RotJoint);
    else if (type.find("Trans") != std::string::npos)
      kinds.push_back(KDL::BasicJointType::TransJoint);
  }

  assert(kinds.size() == lb.data.size());

  // Validity first: a rejected description reports zero active joints, so substituting for it by
  // size would quietly turn a refusal into an uncoupled chain.
  if (!couplings.valid())
  {
    failInitialization(couplings.error());
    return;
  }

  // An empty description means an uncoupled chain, so it becomes the uncoupled description of the
  // right size once and nothing below has to ask which of the two it is holding.
  if (couplings.size() == 0)
    couplings = JointCouplings(chain.getNrOfJoints());
  if (couplings.size() != chain.getNrOfJoints())
  {
    failInitialization("the joint couplings have " + std::to_string(couplings.size()) +
                       " entries, but the chain has " + std::to_string(chain.getNrOfJoints()) + " joints");
    return;
  }

  // Tightened in place, once, here: from now on lb and ub ARE the effective bounds, so the restarts
  // sample inside the reachable interval and a continuous mimicked joint whose mimic is bounded
  // stops being continuous without anything special-casing it below.
  std::string why;
  if (!couplings.tighten(lb, ub, why))
  {
    failInitialization(why);
    return;
  }

  // The mechanism's own bounds are the default query's bounds, so the solvers and the classification
  // both start from them; CartToJnt redoes each per call.
  Query mechanism;
  mechanism.q_min = lb;
  mechanism.q_max = ub;
  configureSolvers(mechanism);
  classifyJoints(lb, ub);
  // Fixed by the mechanism, so it is decided once rather than per call.
  classifyRevolutions();

  initialized = true;
}

bool TRAC_IK::unique_solution(const KDL::JntArray& sol)
{

  for (uint i = 0; i < solutions.size(); i++)
    if (myEqual(sol, solutions[i]))
      return false;
  return true;

}

inline void normalizeAngle(double& val, const double& min, const double& max)
{
  if (val > max)
  {
    //Find actual angle offset
    double diffangle = fmod(val - max, 2 * M_PI);
    // Add that to upper bound and go back a full rotation
    val = max + diffangle - 2 * M_PI;
  }

  if (val < min)
  {
    //Find actual angle offset
    double diffangle = fmod(min - val, 2 * M_PI);
    // Add that to upper bound and go back a full rotation
    val = min - diffangle + 2 * M_PI;
  }
}

inline void normalizeAngle(double& val, const double& target)
{
  normalizeAngle(val, target - M_PI, target + M_PI);
}


template<typename T1, typename T2>
bool TRAC_IK::runSolver(T1& solver, T2& other_solver,
                        const Query& query,
                        const KDL::JntArray &q_init,
                        const KDL::Frame &p_in)
{
  KDL::JntArray q_out;

  double fulltime = query.timeout;
  KDL::JntArray seed = q_init;

  while (true)
  {
    auto timediff = system_clock.now() - start_time;
    auto time_left = fulltime - timediff.seconds();

    if (time_left <= 0)
      break;

    solver.setMaxtime(time_left);

    int RC = solver.CartToJnt(seed, p_in, q_out, query.tolerance_bounds);
    if (RC >= 0)
    {
      switch (query.solve_type)
      {
      case Manip1:
      case Manip2:
      case Manip3:
        normalize_limits(q_init, q_out, query.q_min, query.q_max);
        break;
      default:
        normalize_seed(q_init, q_out, query.q_min, query.q_max);
        break;
      }
      mtx_.lock();
      if (unique_solution(q_out))
      {
        solutions.push_back(q_out);
        uint curr_size = solutions.size();
        errors.resize(curr_size);
        double err, penalty, manip_value;
        switch (query.solve_type)
        {
        case Manip1:
          penalty = manipPenalty(q_out, query.q_min, query.q_max);
          manip_value = TRAC_IK::manipValue1(q_out);
          err = penalty * manip_value;
          break;
        case Manip2:
          penalty = manipPenalty(q_out, query.q_min, query.q_max);
          manip_value = TRAC_IK::manipValue2(q_out);
          err = penalty * manip_value;
          break;
        case Manip3:
          penalty = manipPenalty(q_out, query.q_min, query.q_max);
          manip_value = TRAC_IK::manipValue3(q_out);
          err = penalty * manip_value;
          break;
        default:
          err = TRAC_IK::JointErr(q_init, q_out);
          break;
        }
        errors[curr_size - 1] = std::make_pair(err, curr_size - 1);
      }
      mtx_.unlock();
    }

    if (!solutions.empty() && query.solve_type == Speed)
      break;

    // Resample the joints the solver chooses, then rebuild the rest: a random FULL configuration
    // would start every restart off the couplings, and the branch would spend the restart walking
    // back onto them.
    for (const uint j : couplings.activeIndices())
      if (types[j] == KDL::BasicJointType::Continuous)
        seed(j) = fRand(q_init(j) - 2 * M_PI, q_init(j) + 2 * M_PI);
      else
        seed(j) = fRand(query.q_min(j), query.q_max(j));
    rebuildMimicEntries(seed);
  }
  other_solver.abort();

  solver.setMaxtime(fulltime);

  return true;
}


KDL::JntArray TRAC_IK::repairSeed(const KDL::JntArray& q_init, const Query& query) const
{
  KDL::JntArray seed = q_init;

  // Only the joints a coupling reads are clamped. An active joint nothing follows is left exactly
  // as the caller gave it, so an uncoupled chain gets the seed it always got, and the two inner
  // solvers keep their own, better-informed handling of an out-of-range seed (NLopt wraps a
  // rotational one into range rather than clipping it).
  for (const uint i : couplings.activeIndices())
  {
    if (!couplings.isMimicked(i) || types[i] == KDL::BasicJointType::Continuous)
      continue;
    const double clamped = std::min(std::max(seed(i), query.q_min(i)), query.q_max(i));
    if (clamped != seed(i))
    {
      RCLCPP_DEBUG(logger, "Seed value %f for joint %d is outside the bounds its couplings leave it "
                   "[%f, %f]; clamped to %f", seed(i), (int)i, query.q_min(i), query.q_max(i), clamped);
      seed(i) = clamped;
    }
  }

  rebuildMimicEntries(seed);
  return seed;
}


void TRAC_IK::rebuildMimicEntries(KDL::JntArray& full) const
{
  // reduce-then-expand: the mimic entries are dropped and written back from the joints they follow,
  // which is the whole repair -- a full configuration's mimic entries are never read.
  KDL::JntArray reduced;
  couplings.reduce(full, reduced);
  couplings.expand(reduced, full);
}


void TRAC_IK::normalize_seed(const KDL::JntArray& seed, KDL::JntArray& solution,
                             const KDL::JntArray& q_min, const KDL::JntArray& q_max)
{
  // Make sure rotational joint values are within 1 revolution of seed; then
  // ensure joint limits are met.

  for (const uint i : couplings.activeIndices())
  {

    if (types[i] == KDL::BasicJointType::TransJoint)
      continue;

    // Shifting this one by a revolution would move the tip, because something follows it that does
    // not turn by a revolution when it does. Better a solution far from the seed than one that no
    // longer reaches the goal.
    if (!shares_revolutions[i])
      continue;

    double target = seed(i);
    double val = solution(i);

    normalizeAngle(val, target);

    if (types[i] == KDL::BasicJointType::Continuous)
    {
      solution(i) = val;
      continue;
    }

    normalizeAngle(val, q_min(i), q_max(i));

    solution(i) = val;
  }

  rebuildMimicEntries(solution);
}

void TRAC_IK::normalize_limits(const KDL::JntArray& seed, KDL::JntArray& solution,
                               const KDL::JntArray& q_min, const KDL::JntArray& q_max)
{
  // Make sure rotational joint values are within 1 revolution of middle of
  // limits; then ensure joint limits are met.

  for (const uint i : couplings.activeIndices())
  {

    if (types[i] == KDL::BasicJointType::TransJoint)
      continue;

    // As in normalize_seed: a revolution of this joint is not a revolution of the mechanism.
    if (!shares_revolutions[i])
      continue;

    double target = seed(i);

    if (types[i] == KDL::BasicJointType::RotJoint && types[i] != KDL::BasicJointType::Continuous)
      target = (q_max(i) + q_min(i)) / 2.0;

    double val = solution(i);

    normalizeAngle(val, target);

    if (types[i] == KDL::BasicJointType::Continuous)
    {
      solution(i) = val;
      continue;
    }

    normalizeAngle(val, q_min(i), q_max(i));

    solution(i) = val;
  }

  rebuildMimicEntries(solution);
}


double TRAC_IK::manipPenalty(const KDL::JntArray& arr, const KDL::JntArray& q_min, const KDL::JntArray& q_max)
{
  double penalty = 1.0;
  for (uint i = 0; i < arr.data.size(); i++)
  {
    if (types[i] == KDL::BasicJointType::Continuous)
      continue;
    double range = q_max(i) - q_min(i);
    penalty *= ((arr(i) - q_min(i)) * (q_max(i) - arr(i)) / (range * range));
  }
  return std::max(0.0, 1.0 - exp(-1 * penalty));
}


double TRAC_IK::manipValue1(const KDL::JntArray& arr)
{
  Eigen::MatrixXd singular_values = computeSingularValues(arr);

  double error = 1.0;
  for (unsigned int i = 0; i < singular_values.rows(); ++i)
    error *= singular_values(i, 0);
  return error;
}

double TRAC_IK::manipValue2(const KDL::JntArray& arr)
{
  Eigen::MatrixXd singular_values = computeSingularValues(arr);

  return singular_values.minCoeff() / singular_values.maxCoeff();
}

double TRAC_IK::manipValue3(const KDL::JntArray& arr)
{
    Eigen::MatrixXd singular_values = computeSingularValues(arr);

    return singular_values.minCoeff();
}

Eigen::MatrixXd TRAC_IK::computeSingularValues(const KDL::JntArray& arr)
{
    KDL::Jacobian jac(arr.data.size());

    jacsolver->JntToJac(arr, jac);

    Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jac.data);
    return svdsolver.singularValues();
}


int TRAC_IK::CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const Query& query)
{

  if (!initialized)
  {
    RCLCPP_ERROR(logger, "TRAC-IK was not properly initialized with a valid chain or limits.  IK cannot proceed");
    return -1;
  }

  // A seed is a full configuration, and everything below indexes it by chain joint -- the repair
  // first of all. Checked here rather than left to the inner solvers, which would each read past it
  // before reporting the size they wanted.
  if (q_init.rows() != chain.getNrOfJoints())
  {
    RCLCPP_ERROR(logger, "IK seeded with wrong number of joints.  Expected %d but got %d",
                 (int)chain.getNrOfJoints(), (int)q_init.rows());
    return -1;
  }

  // Empty per-call bounds mean the mechanism's own; a non-empty pair must fit the chain.
  const bool per_call_bounds = query.q_min.data.size() != 0 || query.q_max.data.size() != 0;
  if (per_call_bounds &&
      (query.q_min.data.size() != chain.getNrOfJoints() || query.q_max.data.size() != chain.getNrOfJoints()))
  {
    RCLCPP_ERROR(logger, "Query joint bounds must have one entry per chain joint (%d), or none at all",
                 (int)chain.getNrOfJoints());
    return -1;
  }
  // One resolved query from here on: the empty pair that means "the mechanism's own" is filled in
  // once, so nothing downstream has to remember what empty meant.
  Query resolved = query;
  if (per_call_bounds)
  {
    // The couplings describe the mechanism, so they hold for a narrowed search too. Without this a
    // query's own bounds would undo the tightening the constructor did -- including the
    // reclassification of a mimicked joint that is no longer continuous -- and the restarts would
    // sample outside the interval the couplings prove the joint cannot leave.
    std::string why;
    if (!couplings.tighten(resolved.q_min, resolved.q_max, why))
    {
      RCLCPP_ERROR(logger, "This query's joint bounds leave no configuration the couplings allow: %s",
                   why.c_str());
      return -1;
    }
  }
  else
  {
    resolved.q_min = lb;
    resolved.q_max = ub;
  }

  configureSolvers(resolved);
  classifyJoints(resolved.q_min, resolved.q_max);

  // One seed both branches start from, and the only place the caller's configuration is repaired.
  const KDL::JntArray seed = repairSeed(q_init, resolved);

  start_time = system_clock.now();

  nl_solver->reset();
  iksolver->reset();

  // No lock as no threading yet
  solutions.clear();
  errors.clear();

  // cref, not a copy: the query lives on this stack frame and both threads are joined below.
  task1 = std::thread(&TRAC_IK::runKDL, this, std::cref(resolved), seed, p_in);
  // NLopt needs two variables to optimise, which a mechanism with one active joint does not have --
  // a two-joint chain where one joint follows the other is exactly that, and is a mechanism this
  // library supports. Starting the branch anyway would spin its retry loop against a solver that
  // refuses at once, burning a core for the whole timeout while the Newton branch does the work.
  if (couplings.reducedSize() >= 2)
    task2 = std::thread(&TRAC_IK::runNLOPT, this, std::cref(resolved), seed, p_in);

  if (task1.joinable())
      task1.join();
  if (task2.joinable())
      task2.join();

  // No lock as no threading anymore
  if (solutions.empty())
  {
    q_out = q_init;
    return -3;
  }

  switch (query.solve_type)
  {
  case Manip1:
  case Manip2:
  case Manip3:
    std::sort(errors.rbegin(), errors.rend()); // rbegin/rend to sort by max
    break;
  default:
    std::sort(errors.begin(), errors.end());
    break;
  }

  q_out = solutions[errors[0].second];

  return solutions.size();
}


TRAC_IK::~TRAC_IK()
{
  if (initialized)
  {
    iksolver->abort();
    nl_solver->abort();
  }
  if (task1.joinable())
    task1.join();
  if (task2.joinable())
    task2.join();
}
}
