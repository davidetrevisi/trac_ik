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


#ifndef KDLCHAINIKSOLVERPOS_TL_HPP
#define KDLCHAINIKSOLVERPOS_TL_HPP

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <rclcpp/clock.hpp>
#include <trac_ik/joint_coupling.hpp>
#include <kdl/utilities/svd_eigen_HH.hpp>
#include <Eigen/Core>
#include <cmath>
#include <vector>

namespace TRAC_IK
{
class TRAC_IK;
}

namespace KDL
{

enum BasicJointType { RotJoint, TransJoint, Continuous };

class ChainIkSolverPos_TL
{
  friend class TRAC_IK::TRAC_IK;

public:
  /**
   * The bounds are the EFFECTIVE ones and the couplings describe the chain they were tightened
   * through; an empty description means an uncoupled chain, which reduces to itself.
   */
  ChainIkSolverPos_TL(const Chain& chain, const JntArray& q_min, const JntArray& q_max,
                      const TRAC_IK::JointCouplings& couplings = TRAC_IK::JointCouplings(),
                      double maxtime = 0.005, double eps = 1e-3, bool random_restart = false,
                      bool try_jl_wrap = false);

  ~ChainIkSolverPos_TL();

  /// Seed and solution are FULL configurations; the Newton loop inside runs on the reduced one, so
  /// a solution's mimic entries are computed from their mimicked joints rather than searched for.
  int CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out, const KDL::Twist bounds = KDL::Twist::Zero());

  inline void setMaxtime(double t)
  {
    maxtime = t;
  }

  inline void setEps(double e)
  {
    eps = std::abs(e);
  }

private:
  const Chain chain;
  TRAC_IK::JointCouplings couplings;
  /// The effective bounds, restricted to the joints this solver actually chooses.
  JntArray q_min, q_max;

  KDL::Twist bounds;

  KDL::ChainJntToJacSolver jacsolver;
  KDL::ChainFkSolverPos_recursive fksolver;
  double maxtime;

  double eps;

  bool rr;
  bool wrap;

  /// Reduced, to match the bounds and the state below. A joint's type is a fact about the joint, so
  /// it is read off the full chain and then restricted, rather than derived from the reduction.
  std::vector<KDL::BasicJointType> types;

  /// The Newton loop's state and its scratch. A reduced configuration is what the loop steps; the
  /// full one is materialised only for forward kinematics and the Jacobian.
  JntArray q, q_curr, q_full;
  Jacobian jac;
  Eigen::MatrixXd jac_reduced;
  Eigen::VectorXd delta_q;
  /// Sized once: this runs every Newton iteration, and an SVD allocating per call would show.
  Eigen::MatrixXd svd_u, svd_v;
  Eigen::VectorXd svd_s, svd_tmp, svd_rhs;

  /**
   * The velocity step, over the reduced Jacobian: a truncated SVD pseudo-inverse, at a tenth of
   * KDL's cutoff (see the measurement at the truncation itself). KDL has no coupled velocity solver and MoveIt's ChainIkSolverVelMimicSVD ships no
   * exported link target, so this is ours; on an uncoupled chain it is a plain pseudo-inverse, and
   * measured faster than the KDL::ChainIkSolverVel_pinv it replaces on arm6, arm7 and the crane.
   */
  void reducedVelocityStep(const Eigen::MatrixXd& jacobian, const Twist& twist, Eigen::VectorXd& qdot);

  inline void abort()
  {
    aborted = true;
  }

  inline void reset()
  {
    aborted = false;
  }

  bool aborted;

  Frame f;
  Twist delta_twist;

  inline static double fRand(double min, double max)
  {
    double f = (double)rand() / RAND_MAX;
    return min + f * (max - min);
  }
  
  rclcpp::Clock system_clock;

};

/**
 * determines the rotation axis necessary to rotate from frame b1 to the
 * orientation of frame b2 and the vector necessary to translate the origin
 * of b1 to the origin of b2, and stores the result in a Twist
 * datastructure.  The result is w.r.t. frame b1.
 * \param F_a_b1 frame b1 expressed with respect to some frame a.
 * \param F_a_b2 frame b2 expressed with respect to some frame a.
 * \warning The result is not a real Twist!
 * \warning In contrast to standard KDL diff methods, the result of
 * diffRelative is w.r.t. frame b1 instead of frame a.
 */
IMETHOD Twist diffRelative(const Frame & F_a_b1, const Frame & F_a_b2, double dt = 1)
{
  return Twist(F_a_b1.M.Inverse() * diff(F_a_b1.p, F_a_b2.p, dt),
               F_a_b1.M.Inverse() * diff(F_a_b1.M, F_a_b2.M, dt));
}

}

#endif
