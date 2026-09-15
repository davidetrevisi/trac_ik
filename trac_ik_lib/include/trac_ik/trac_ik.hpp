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


#ifndef TRAC_IK_HPP
#define TRAC_IK_HPP

#include <trac_ik/joint_coupling.hpp>
#include <trac_ik/nlopt_ik.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <mutex>
#include <memory>

namespace TRAC_IK
{

enum SolveType { Speed, Distance, Manip1, Manip2, Manip3 };

/**
 * Everything that may differ between one call to CartToJnt and the next.
 *
 * The constructor describes the mechanism -- the chain and the joint bounds it was built with --
 * and a Query describes one question asked of it, so a single solver instance serves many queries
 * and nothing about a call is hidden in solver state.
 */
struct Query
{
  /// Wall-clock budget for the whole solve, in seconds.
  double timeout = 0.005;

  /// Residual floor: the solve is accepted once the pose error is within it on every axis.
  double epsilon = 1e-5;

  /// Which solution the search returns, and therefore how long it keeps looking.
  SolveType solve_type = Speed;

  /// Per-axis tolerance bounds in the goal frame: vel is x, y, z and rot is rx, ry, rz. An axis
  /// whose error falls inside its bound counts as met. An infinite bound frees the axis outright; a
  /// finite one is still aimed at and only judged leniently; zero, the default, asks for the axis to
  /// be met to within epsilon.
  KDL::Twist tolerance_bounds = KDL::Twist::Zero();

  /// Joint bounds for this call alone. Empty -- the default -- means the mechanism's own, the pair
  /// the constructor was given. A non-empty pair must have one entry per chain joint. A rotational
  /// joint counts as continuous for the call exactly when the bounds in force for it are infinite,
  /// so narrowing a continuous joint genuinely bounds it for that call and nothing else.
  KDL::JntArray q_min, q_max;

  /// How long the search may go without improving before it gives up, as a fraction of timeout.
  /// Carried here so the stall exit has a per-call home; no solver reads it yet (ticket 12).
  double stall_window = 0.25;
};

class TRAC_IK
{
public:
  /**
   * The mechanism, and nothing about any one question asked of it (that is Query's).
   *
   * `_couplings` describes how the chain's joints are coupled; the default, an empty value, means an
   * uncoupled chain and constructs exactly as before. The bounds are tightened through the couplings
   * in place here, so getKDLLimits answers with effective bounds and not with what was passed.
   *
   * Construction can fail -- a description the couplings reject, or a mechanism no configuration
   * satisfies. It does not throw: a FATAL line names the offending joint, isInitialized() is false,
   * initializationError() says why, and CartToJnt returns -1.
   */
  TRAC_IK(const KDL::Chain& _chain, const KDL::JntArray& _q_min, const KDL::JntArray& _q_max,
          const JointCouplings& _couplings = JointCouplings(),
          const rclcpp::Logger& _logger = rclcpp::get_logger("trac_ik.trac_ik_lib"));

  ~TRAC_IK();

  bool isInitialized() const
  {
    return initialized;
  }

  /// Why initialisation failed, naming the joint at fault; empty when it did not fail.
  const std::string& initializationError() const
  {
    return init_error;
  }

  bool getKDLChain(KDL::Chain& chain_)
  {
    chain_ = chain;
    return initialized;
  }

  /// The EFFECTIVE bounds: the constructor's, tightened through the couplings. For a coupled chain
  /// these are narrower than what was passed, and they are what "inside its limits" means here --
  /// a caller sampling configurations from them samples ones the mechanism can actually hold.
  bool getKDLLimits(KDL::JntArray& lb_, KDL::JntArray& ub_)
  {
    lb_ = lb;
    ub_ = ub;
    return initialized;
  }

  // Requires a previous call to CartToJnt()
  bool getSolutions(std::vector<KDL::JntArray>& solutions_)
  {
    solutions_ = solutions;
    return initialized && !solutions.empty();
  }

  /**
   * The candidates the last call collected, with the key each was ranked by.
   *
   * Each pair is (ordering key, index into solutions_). The key is NOT a distance in general: under
   * Speed and Distance it is the squared joint distance from the seed and smaller is better, while
   * under the Manip solve types it is a manipulability score and larger is better. Both orderings
   * put the returned solution first, which is the only thing a caller can rely on across types.
   */
  bool getSolutions(std::vector<KDL::JntArray>& solutions_, std::vector<std::pair<double, uint> >& errors_)
  {
    errors_ = errors;
    return getSolutions(solutions_);
  }

  static double JointErr(const KDL::JntArray& arr1, const KDL::JntArray& arr2)
  {
    double err = 0;
    for (uint i = 0; i < arr1.data.size(); i++)
    {
      err += pow(arr1(i) - arr2(i), 2);
    }

    return err;
  }

  int CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const Query& query = Query());

private:
  rclcpp::Logger logger;
  bool initialized;
  /// Why initialisation failed; empty while it has not.
  std::string init_error;
  KDL::Chain chain;
  /// The mechanism's effective joint bounds: the constructor's, tightened through the couplings at
  /// construction. Nothing keeps the untightened pair -- a bound the mechanism cannot reach is not
  /// a fact any part of a solve wants.
  KDL::JntArray lb, ub;
  /// How this chain's joints are coupled. Empty-as-passed becomes the uncoupled description of the
  /// right size, so nothing downstream has to spell "no couplings" twice.
  JointCouplings couplings;
  /// The bounds the inner solvers currently hold, so a query that does not change them costs no
  /// rebuild.
  KDL::JntArray solver_lb, solver_ub;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacsolver;

  std::unique_ptr<NLOPT_IK::NLOPT_IK> nl_solver;
  std::unique_ptr<KDL::ChainIkSolverPos_TL> iksolver;

  rclcpp::Clock system_clock;
  rclcpp::Time start_time;

  // These take a RESOLVED query: one whose joint bounds are filled in, never the empty pair that
  // means "the mechanism's own". CartToJnt resolves it once, on entry.
  template<typename T1, typename T2>
  bool runSolver(T1& solver, T2& other_solver,
                 const Query& query,
                 const KDL::JntArray &q_init,
                 const KDL::Frame &p_in);

  bool runKDL(const Query& query, const KDL::JntArray &q_init, const KDL::Frame &p_in);
  bool runNLOPT(const Query& query, const KDL::JntArray &q_init, const KDL::Frame &p_in);

  void normalize_seed(const KDL::JntArray& seed, KDL::JntArray& solution,
                      const KDL::JntArray& q_min, const KDL::JntArray& q_max);
  void normalize_limits(const KDL::JntArray& seed, KDL::JntArray& solution,
                        const KDL::JntArray& q_min, const KDL::JntArray& q_max);

  /// Rotational or translational, from the chain's segments. Fixed by the mechanism.
  std::vector<KDL::BasicJointType> kinds;
  /// The classification in force for the current solve: kinds, with each rotational joint called
  /// continuous or not according to the bounds this solve runs under. Written before the two
  /// threads start and only read while they run.
  std::vector<KDL::BasicJointType> types;

  std::mutex mtx_;
  std::vector<KDL::JntArray> solutions;
  std::vector<std::pair<double, uint> >  errors;

  std::thread task1, task2;

  bool unique_solution(const KDL::JntArray& sol);

  inline static double fRand(double min, double max)
  {
    double f = (double)rand() / RAND_MAX;
    return min + f * (max - min);
  }

  /* @brief Manipulation metrics and penalties taken from "Workspace
  Geometric Characterization and Manipulability of Industrial Robots",
  Ming-June, Tsia, PhD Thesis, Ohio State University, 1986.
  https://etd.ohiolink.edu/!etd.send_file?accession=osu1260297835
  */
  double manipPenalty(const KDL::JntArray& arr, const KDL::JntArray& q_min, const KDL::JntArray& q_max);
  double manipValue1(const KDL::JntArray& arr);
  double manipValue2(const KDL::JntArray& arr);
  double manipValue3(const KDL::JntArray& arr);

  Eigen::MatrixXd computeSingularValues(const KDL::JntArray& arr);

  inline bool myEqual(const KDL::JntArray& a, const KDL::JntArray& b, const double eps=1e-4)
  {
    return (a.data - b.data).isZero(eps);
  }

  void initialize();

  /// Leave the object uninitialised, with a FATAL line and a retrievable reason. The fork's one
  /// failure convention: no exceptions, and never a solver that answers as if nothing were wrong.
  void failInitialization(const std::string& why);

  /// Decide, for these bounds, which rotational joints are continuous. The test is the one both
  /// inner solvers apply to the bounds they are built with, so all three agree on every joint.
  void classifyJoints(const KDL::JntArray& q_min, const KDL::JntArray& q_max);

  /// Point the inner solvers at a resolved query's joint bounds and epsilon, rebuilding them only
  /// when the bounds actually changed.
  void configureSolvers(const Query& query);
};

inline bool TRAC_IK::runKDL(const Query& query, const KDL::JntArray &q_init, const KDL::Frame &p_in)
{
  return runSolver(*iksolver.get(), *nl_solver.get(), query, q_init, p_in);
}

inline bool TRAC_IK::runNLOPT(const Query& query, const KDL::JntArray &q_init, const KDL::Frame &p_in)
{
  return runSolver(*nl_solver.get(), *iksolver.get(), query, q_init, p_in);
}

}

#endif
