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

#include <trac_ik/nlopt_ik.hpp>
#include <limits>
#include <cmath>
#include <cfloat>



namespace NLOPT_IK
{

double minfuncSumSquared(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
  // Auxilory function to minimize (Sum of Squared joint angle error
  // from the requested configuration).  Because we wanted a Class
  // without static members, but NLOpt library does not support
  // passing methods of Classes, we use these auxilary functions.

  NLOPT_IK *c = (NLOPT_IK *) data;

  std::vector<double> vals(x);

  double jump = FLT_EPSILON;
  double result[1];
  c->cartSumSquaredError(vals, result);

  if (!grad.empty())
  {
    double v1[1];
    for (uint i = 0; i < x.size(); i++)
    {
      double original = vals[i];

      vals[i] = original + jump;
      c->cartSumSquaredError(vals, v1);

      vals[i] = original;
      grad[i] = (v1[0] - result[0]) / (2.0 * jump);
    }
  }

  return result[0];
}


NLOPT_IK::NLOPT_IK(const KDL::Chain& _chain, const KDL::JntArray& _q_min, const KDL::JntArray& _q_max,
                   const TRAC_IK::JointCouplings& _couplings, double _maxtime, double _eps,
                   const rclcpp::Logger& _logger):
  logger_(_logger), chain(_chain),
  couplings(_couplings.size() == 0 ? TRAC_IK::JointCouplings(_chain.getNrOfJoints()) : _couplings),
  q_full(_chain.getNrOfJoints()), fksolver(chain), maxtime(_maxtime), eps(std::abs(_eps))
{
  assert(chain.getNrOfJoints() == _q_min.data.size());
  assert(chain.getNrOfJoints() == _q_max.data.size());
  assert(chain.getNrOfJoints() == couplings.size());

  //Constructor for an IK Class.  Takes in a Chain to operate on,
  //the min and max joint limits, an (optional) maximum number of
  //iterations, and an (optional) desired error.
  reset();

  // Active joints, not chain joints: a chain of nine whose couplings leave one free is a
  // one-variable problem, whatever its length suggests.
  if (couplings.reducedSize() < 2)
  {
    RCLCPP_WARN_THROTTLE(logger_, system_clock, 1000.0, "NLOpt_IK can only be run for chains of length 2 or more");
    return;
  }
  // The decision vector IS the reduced configuration. Enforcing the couplings by reducing what the
  // optimiser may choose is exact and costs nothing; an equality constraint would be neither.
  opt = nlopt::opt(nlopt::LD_SLSQP, couplings.reducedSize());

  for (const uint i : couplings.activeIndices())
  {
    lb.push_back(_q_min(i));
    ub.push_back(_q_max(i));
  }

  std::vector<KDL::BasicJointType> full_types;
  for (uint i = 0; i < chain.segments.size(); i++)
  {
    std::string type = chain.segments[i].getJoint().getTypeName();
    if (type.find("Rot") != std::string::npos)
    {
      if (_q_max(full_types.size()) >= std::numeric_limits<float>::max() &&
          _q_min(full_types.size()) <= std::numeric_limits<float>::lowest())
        full_types.push_back(KDL::BasicJointType::Continuous);
      else
        full_types.push_back(KDL::BasicJointType::RotJoint);
    }
    else if (type.find("Trans") != std::string::npos)
      full_types.push_back(KDL::BasicJointType::TransJoint);
  }
  types = couplings.reduceVector(full_types);

  assert(types.size() == lb.size());

  opt.set_xtol_abs(FLT_EPSILON);
  opt.set_min_objective(minfuncSumSquared, this);
}


void NLOPT_IK::cartSumSquaredError(const std::vector<double>& x, double error[])
{
  // Actual function to compute Euclidean distance error.  This uses
  // the KDL Forward Kinematics solver to compute the Cartesian pose
  // of the current joint configuration and compares that to the
  // desired Cartesian pose for the IK solve.

  if (aborted || progress != -3)
  {
    opt.force_stop();
    return;
  }


  KDL::JntArray q_red(x.size());

  for (uint i = 0; i < x.size(); i++)
    q_red(i) = x[i];

  couplings.expand(q_red, q_full);

  int rc = fksolver.JntToCart(q_full, currentPose);

  if (rc < 0)
    RCLCPP_FATAL_STREAM(logger_, "KDL FKSolver is failing: " << q_full.data);

  if (std::isnan(currentPose.p.x()))
  {
    RCLCPP_ERROR(logger_, "NaNs from NLOpt!!");
    error[0] = std::numeric_limits<float>::max();
    progress = -1;
    return;
  }

  KDL::Twist delta_twist = KDL::diffRelative(targetPose, currentPose);

  for (int i = 0; i < 6; i++)
  {
    if (std::abs(delta_twist[i]) <= std::abs(bounds[i]))
      delta_twist[i] = 0.0;
  }

  error[0] = KDL::dot(delta_twist.vel, delta_twist.vel) + KDL::dot(delta_twist.rot, delta_twist.rot);

  if (KDL::Equal(delta_twist, KDL::Twist::Zero(), eps))
  {
    progress = 1;
    best_x = x;
    return;
  }
}



int NLOPT_IK::CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist _bounds)
{
  // User command to start an IK solve.  Takes in a seed
  // configuration and a Cartesian pose.  Outputs the joint
  // configuration found that solves the IK.

  // Returns -3 if a configuration could not be found within the eps
  // set up in the constructor.

  auto start_time = system_clock.now();

  bounds = _bounds;
  q_out = q_init;

  if (couplings.reducedSize() < 2)
  {
    RCLCPP_ERROR_THROTTLE(logger_, system_clock, 1000.0, "NLOpt_IK can only be run for chains of length 2 or more");
    return -3;
  }

  // A seed is a full configuration, so it is the chain that is counted here, not the search space.
  if (q_init.data.size() != couplings.size())
  {
    RCLCPP_ERROR_THROTTLE(logger_, system_clock, 1000.0, "IK seeded with wrong number of joints.  Expected %d but got %d", (int)couplings.size(), (int)q_init.data.size());
    return -3;
  }

  opt.set_maxtime(maxtime);


  double minf; /* the minimum objective value, upon return */

  targetPose = p_in;

  // Reduce the seed: its mimic entries are dropped here and recomputed by expand() on the way out,
  // so a seed that does not satisfy its couplings is repaired rather than searched from.
  KDL::JntArray q_red_init;
  couplings.reduce(q_init, q_red_init);

  std::vector<double> x(couplings.reducedSize());

  for (uint i = 0; i < x.size(); i++)
  {
    x[i] = q_red_init(i);

    if (types[i] == KDL::BasicJointType::Continuous)
      continue;

    if (types[i] == KDL::BasicJointType::TransJoint)
    {
      x[i] = std::min(x[i], ub[i]);
      x[i] = std::max(x[i], lb[i]);
    }
    else
    {

      // Below is to handle bad seeds outside of limits

      if (x[i] > ub[i])
      {
        //Find actual angle offset
        double diffangle = fmod(x[i] - ub[i], 2 * M_PI);
        // Add that to upper bound and go back a full rotation
        x[i] = ub[i] + diffangle - 2 * M_PI;
      }

      if (x[i] < lb[i])
      {
        //Find actual angle offset
        double diffangle = fmod(lb[i] - x[i], 2 * M_PI);
        // Subtract that from lower bound and go forward a full rotation
        x[i] = lb[i] - diffangle + 2 * M_PI;
      }

      if (x[i] > ub[i])
        x[i] = (ub[i] + lb[i]) / 2.0;
    }
  }

  best_x = x;
  progress = -3;

  std::vector<double> artificial_lower_limits(lb.size());

  for (uint i = 0; i < lb.size(); i++)
    if (types[i] == KDL::BasicJointType::Continuous)
      artificial_lower_limits[i] = best_x[i] - 2 * M_PI;
    else if (types[i] == KDL::BasicJointType::TransJoint)
      artificial_lower_limits[i] = lb[i];
    else
      artificial_lower_limits[i] = std::max(lb[i], best_x[i] - 2 * M_PI);

  opt.set_lower_bounds(artificial_lower_limits);

  std::vector<double> artificial_upper_limits(lb.size());

  for (uint i = 0; i < ub.size(); i++)
    if (types[i] == KDL::BasicJointType::Continuous)
      artificial_upper_limits[i] = best_x[i] + 2 * M_PI;
    else if (types[i] == KDL::BasicJointType::TransJoint)
      artificial_upper_limits[i] = ub[i];
    else
      artificial_upper_limits[i] = std::min(ub[i], best_x[i] + 2 * M_PI);

  opt.set_upper_bounds(artificial_upper_limits);

  try
  {
    opt.optimize(x, minf);
  }
  catch (...)
  {
  }

  if (progress == -1) // Got NaNs
    progress = -3;


  if (!aborted && progress < 0)
  {
    auto diff = system_clock.now() - start_time;
    auto time_left = maxtime - diff.seconds();

    while (time_left > 0 && !aborted && progress < 0)
    {

      for (uint i = 0; i < x.size(); i++)
        x[i] = fRand(artificial_lower_limits[i], artificial_upper_limits[i]);

      opt.set_maxtime(time_left);

      try
      {
        opt.optimize(x, minf);
      }
      catch (...) {}

      if (progress == -1) // Got NaNs
        progress = -3;

      auto diff = system_clock.now() - start_time;
      time_left = maxtime - diff.seconds();
    }
  }


  // Solutions leave as full configurations, with their mimic entries computed from the values the
  // optimiser actually chose.
  KDL::JntArray best_red(best_x.size());
  for (uint i = 0; i < best_x.size(); i++)
    best_red(i) = best_x[i];
  couplings.expand(best_red, q_out);

  return progress;

}


}
