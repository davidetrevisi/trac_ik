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

#include <trac_ik/kdl_tl.hpp>
#include <cfloat>

namespace KDL
{
ChainIkSolverPos_TL::ChainIkSolverPos_TL(const Chain& _chain, const JntArray& _q_min, const JntArray& _q_max,
                                        const TRAC_IK::JointCouplings& _couplings, double _maxtime, double _eps,
                                        bool _random_restart, bool _try_jl_wrap):
  chain(_chain),
  couplings(_couplings.size() == 0 ? TRAC_IK::JointCouplings(_chain.getNrOfJoints()) : _couplings),
  jacsolver(chain), fksolver(chain), jac(chain.getNrOfJoints()),
  maxtime(_maxtime), eps(_eps), rr(_random_restart), wrap(_try_jl_wrap)
{

  assert(chain.getNrOfJoints() == _q_min.data.size());
  assert(chain.getNrOfJoints() == _q_max.data.size());
  assert(chain.getNrOfJoints() == couplings.size());

  reset();

  std::vector<KDL::BasicJointType> full_types;
  for (uint i = 0; i < chain.segments.size(); i++)
  {
    std::string type = chain.segments[i].getJoint().getTypeName();
    if (type.find("Rot") != std::string::npos)
    {
      if (_q_max(full_types.size()) >= std::numeric_limits<float>::max() &&
          _q_min(full_types.size()) <= std::numeric_limits<float>::lowest())
        full_types.push_back(KDL::BasicJointType::Continuous);
      else full_types.push_back(KDL::BasicJointType::RotJoint);
    }
    else if (type.find("Trans") != std::string::npos)
      full_types.push_back(KDL::BasicJointType::TransJoint);

  }

  assert(full_types.size() == _q_max.data.size());

  // Everything the Newton loop reads is reduced here, once: the bounds it clips against, the types
  // it classifies by, and the vectors it steps.
  types = couplings.reduceVector(full_types);
  couplings.reduce(_q_min, q_min);
  couplings.reduce(_q_max, q_max);
  q.resize(couplings.reducedSize());
  q_curr.resize(couplings.reducedSize());
  q_full.resize(couplings.size());
  jac_reduced.setZero(6, couplings.reducedSize());
  delta_q.setZero(couplings.reducedSize());
  svd_u.setZero(6, couplings.reducedSize());
  svd_v.setZero(couplings.reducedSize(), couplings.reducedSize());
  svd_s.setZero(couplings.reducedSize());
  svd_tmp.setZero(couplings.reducedSize());
  svd_rhs.setZero(couplings.reducedSize());
}


void ChainIkSolverPos_TL::reducedVelocityStep(const Eigen::MatrixXd& jacobian, const Twist& twist,
                                              Eigen::VectorXd& qdot)
{
  Eigen::Matrix<double, 6, 1> v;
  for (int i = 0; i < 6; i++)
    v(i) = twist[i];

  // KDL's Householder SVD, the one its own ChainIkSolverVel_pinv runs: what this solver replaces is
  // the Jacobian that solver is given, not the numerics it applies to it. Measured on this fixture
  // set it is also the faster of the two available -- Eigen's JacobiSVD costs about 20% of a solve
  // more on arm6 -- so taking KDL's keeps the uncoupled path quicker than it was before the fold.
  if (KDL::svd_eigen_HH(jacobian, svd_u, svd_s, svd_v, svd_tmp) < 0)
  {
    // No decomposition, so no honest step. A zero one reads as "stuck" to the loop below, which is
    // what a random restart is for; inventing a direction here would be worse than not moving.
    qdot.setZero();
    return;
  }

  // KDL's own cutoff. A coupled chain is where it earns its keep: the crane's four parallel
  // telescope stages are a rank deficiency in the full Jacobian, and folding them is what removes
  // it, so what is left to truncate is a genuine singularity rather than a coupling.
  const double tol = 1e-6;
  // V * S^-1 * U^T * v, in two products through the smaller vector rather than one expression that
  // builds a matrix per iteration.
  svd_rhs.noalias() = svd_u.transpose() * v;
  for (int i = 0; i < svd_s.size(); i++)
    svd_rhs(i) = (svd_s(i) < tol) ? 0.0 : svd_rhs(i) / svd_s(i);

  qdot.noalias() = svd_v * svd_rhs;
}


int ChainIkSolverPos_TL::CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist _bounds)
{

  if (aborted)
    return -3;

  auto start_time = system_clock.now();
  bounds = _bounds;

  // The seam is full, the search is reduced: the seed's mimic entries are dropped here and written
  // back by expand(), so they are recomputed from their mimicked joints rather than trusted.
  couplings.reduce(q_init, q);
  couplings.expand(q, q_out);

  double time_left;

  do
  {
    couplings.expand(q, q_full);
    fksolver.JntToCart(q_full, f);
    delta_twist = diffRelative(p_in, f);

    if (std::abs(delta_twist.vel.x()) <= std::abs(bounds.vel.x()))
      delta_twist.vel.x(0);

    if (std::abs(delta_twist.vel.y()) <= std::abs(bounds.vel.y()))
      delta_twist.vel.y(0);

    if (std::abs(delta_twist.vel.z()) <= std::abs(bounds.vel.z()))
      delta_twist.vel.z(0);

    if (std::abs(delta_twist.rot.x()) <= std::abs(bounds.rot.x()))
      delta_twist.rot.x(0);

    if (std::abs(delta_twist.rot.y()) <= std::abs(bounds.rot.y()))
      delta_twist.rot.y(0);

    if (std::abs(delta_twist.rot.z()) <= std::abs(bounds.rot.z()))
      delta_twist.rot.z(0);

    if (Equal(delta_twist, Twist::Zero(), eps))
    {
      couplings.expand(q, q_out);
      return 1;
    }

    delta_twist = diff(f, p_in);

    // The reduced Jacobian is this chain's actual Jacobian: no other tip velocity is commandable,
    // so the step is taken over the joints the solver chooses and nothing has to be projected back.
    jacsolver.JntToJac(q_full, jac);
    couplings.foldJacobian(jac, jac_reduced);
    reducedVelocityStep(jac_reduced, delta_twist, delta_q);

    for (unsigned int j = 0; j < q.rows(); j++)
      q_curr(j) = q(j) + delta_q(j);

    for (unsigned int j = 0; j < q_min.data.size(); j++)
    {
      if (types[j] == KDL::BasicJointType::Continuous)
        continue;
      if (q_curr(j) < q_min(j))
      {
        if (!wrap || types[j] == KDL::BasicJointType::TransJoint)
          // KDL's default
          q_curr(j) = q_min(j);
        else
        {
          // Find actual wrapped angle between limit and joint
          double diffangle = fmod(q_min(j) - q_curr(j), 2 * M_PI);
          // Subtract that angle from limit and go into the range by a
          // revolution
          double curr_angle = q_min(j) - diffangle + 2 * M_PI;
          if (curr_angle > q_max(j))
            q_curr(j) = q_min(j);
          else
            q_curr(j) = curr_angle;
        }
      }
    }

    for (unsigned int j = 0; j < q_max.data.size(); j++)
    {
      if (types[j] == KDL::BasicJointType::Continuous)
        continue;

      if (q_curr(j) > q_max(j))
      {
        if (!wrap || types[j] == KDL::BasicJointType::TransJoint)
          // KDL's default
          q_curr(j) = q_max(j);
        else
        {
          // Find actual wrapped angle between limit and joint
          double diffangle = fmod(q_curr(j) - q_max(j), 2 * M_PI);
          // Add that angle to limit and go into the range by a revolution
          double curr_angle = q_max(j) + diffangle - 2 * M_PI;
          if (curr_angle < q_min(j))
            q_curr(j) = q_max(j);
          else
            q_curr(j) = curr_angle;
        }
      }
    }

    Subtract(q, q_curr, q);

    if (q.data.isZero(FLT_EPSILON))
    {
      if (rr)
      {
        // Restarts sample the decision variables only; expand() puts the mimic joints back where
        // the couplings say they are, so a restart cannot begin off the relation.
        for (unsigned int j = 0; j < q.data.size(); j++)
          if (types[j] == KDL::BasicJointType::Continuous)
            q_curr(j) = fRand(q_curr(j) - 2 * M_PI, q_curr(j) + 2 * M_PI);
          else
            q_curr(j) = fRand(q_min(j), q_max(j));
      }

      // Below would be an optimization to the normal KDL, where when it
      // gets stuck, it returns immediately.  Don't use to compare KDL with
      // random restarts or TRAC-IK to plain KDL.

      // else {
      //   q_out=q_curr;
      //   return -3;
      // }
    }

    q = q_curr;

    auto timediff = system_clock.now() - start_time;
    time_left = maxtime - timediff.seconds();
  }
  while (time_left > 0 && !aborted);

  couplings.expand(q, q_out);
  return -3;
}

ChainIkSolverPos_TL::~ChainIkSolverPos_TL()
{
}


}
