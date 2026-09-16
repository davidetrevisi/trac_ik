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

#ifndef NLOPT_IK_HPP
#define NLOPT_IK_HPP

#include <rclcpp/rclcpp.hpp>
#include <trac_ik/kdl_tl.hpp>
#include <nlopt.hpp>
#include <cmath>


namespace NLOPT_IK
{

class NLOPT_IK
{
  friend class TRAC_IK::TRAC_IK;
public:
  /**
   * The bounds are the EFFECTIVE ones and the couplings describe the chain they were tightened
   * through; an empty description means an uncoupled chain, which reduces to itself.
   */
  NLOPT_IK(const KDL::Chain& _chain, const KDL::JntArray& _q_min, const KDL::JntArray& _q_max,
           const TRAC_IK::JointCouplings& _couplings = TRAC_IK::JointCouplings(),
           double _maxtime = 0.005, double _eps = 1e-3,
           const rclcpp::Logger& _logger = rclcpp::get_logger("trac_ik.trac_ik_lib"));

  ~NLOPT_IK() {};

  /// Seed and solution are FULL configurations; the decision vector inside is the reduced one, so
  /// the couplings hold by construction rather than by an equality constraint the optimiser has to
  /// satisfy -- and the search is over as many variables as the mechanism actually has.
  int CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out, const KDL::Twist bounds = KDL::Twist::Zero());

  /// `x` is a reduced configuration: the objective expands it before forward kinematics.
  void cartSumSquaredError(const std::vector<double>& x, double error[]);

  inline void setMaxtime(double t)
  {
    maxtime = t;
  }

  inline void setEps(double e)
  {
    eps = std::abs(e);
  }

private:

  inline void abort()
  {
    aborted = true;
  }

  inline void reset()
  {
    aborted = false;
  }

  rclcpp::Logger logger_;
  rclcpp::Clock system_clock;

  /// Reduced, like the decision vector they bound.
  std::vector<double> lb;
  std::vector<double> ub;

  const KDL::Chain chain;
  TRAC_IK::JointCouplings couplings;
  /// The objective's expand() target, held rather than allocated per iteration.
  KDL::JntArray q_full;

  KDL::ChainFkSolverPos_recursive fksolver;

  double maxtime;
  double eps;

  KDL::Frame targetPose;

  /// Reduced, to match lb, ub and the decision vector.
  std::vector<KDL::BasicJointType> types;

  nlopt::opt opt;

  KDL::Frame currentPose;

  /// The best reduced configuration the solve has visited; CartToJnt expands it into the solution.
  std::vector<double> best_x;
  int progress;
  bool aborted;

  KDL::Twist bounds;

  inline static double fRand(double min, double max)
  {
    double f = (double)rand() / RAND_MAX;
    return min + f * (max - min);
  }


};

}

#endif
