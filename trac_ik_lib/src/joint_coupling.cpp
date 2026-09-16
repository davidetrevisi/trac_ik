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

#include <trac_ik/joint_coupling.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace TRAC_IK
{

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

// The fork's current spelling of an unbounded joint: a bound at or past the float sentinels. Ticket
// 09 replaces the spelling with a real infinity; until then this is the one place that has to know
// both, because it is the only place that does arithmetic on a bound.
const double kSentinelMax = static_cast<double>(std::numeric_limits<float>::max());
const double kSentinelMin = static_cast<double>(std::numeric_limits<float>::lowest());

// Read a stored bound as the number it stands for. Mapping a sentinel through a coupling as if it
// were finite is how an unbounded joint quietly becomes a bounded one: a multiplier of 2 would turn
// a 3.4e38 "infinity" into a 1.7e38 limit that every continuous-joint test then reads as finite.
double asBound(double stored)
{
  if (stored >= kSentinelMax)
    return kInf;
  if (stored <= kSentinelMin)
    return -kInf;
  return stored;
}

// ...and write one back in the spelling the rest of the fork reads. A bound that lands past a
// sentinel is unbounded under this convention, so saturating is the honest store, not a clamp.
double asStored(double bound)
{
  if (bound >= kSentinelMax)
    return kSentinelMax;
  if (bound <= kSentinelMin)
    return kSentinelMin;
  return bound;
}

// Slack for the one comparison a user's numbers have to survive: a pinned joint sitting exactly on
// its own bound, where the URDF and the <mimic> offset are two roundings of the same decimal.
constexpr double kBoundTol = 1e-9;

// An interval's ends, the right way round. Mapping an interval through a coupling can hand them back
// swapped, and a negative multiplier is exactly when it does -- the one place the sign matters, so
// it lives in one place.
std::pair<double, double> ordered(double a, double b)
{
  return a <= b ? std::make_pair(a, b) : std::make_pair(b, a);
}

}  // namespace

JointCouplings::JointCouplings(unsigned int chain_joints)
  : couplings_(chain_joints)
{
  active_.reserve(chain_joints);
  reduced_index_.resize(chain_joints);
  mimicked_.assign(chain_joints, false);
  for (unsigned int i = 0; i < chain_joints; ++i)
  {
    reduced_index_[i] = static_cast<unsigned int>(active_.size());
    active_.push_back(i);
  }
}

JointCouplings::JointCouplings(const std::vector<JointCoupling>& description)
  : couplings_(description)
{
  const unsigned int n = size();

  for (unsigned int i = 0; i < n; ++i)
  {
    const JointCoupling& c = couplings_[i];

    if (c.variable_count != 1)
    {
      reject(describe(i) + " has " + std::to_string(c.variable_count) +
             " variables; a chain joint must have exactly one");
      continue;
    }

    if (c.mimicked_index == -1)
      continue;

    if (c.mimicked_index < 0 || static_cast<unsigned int>(c.mimicked_index) >= n)
    {
      // Treating it as a constant would silently move the tip whenever the real joint moved, which
      // is a wrong answer with nothing in it to notice.
      reject(describe(i) + " follows a joint outside the chain");
      continue;
    }
    if (static_cast<unsigned int>(c.mimicked_index) == i)
    {
      reject(describe(i) + " follows itself");
      continue;
    }
    if (couplings_[c.mimicked_index].mimicked_index >= 0)
    {
      // A mimic of a mimic is representable here but not expanded here: the rejection can be
      // relaxed into flattening later, and semantics cannot be taken back.
      reject(describe(i) + " follows " + describe(c.mimicked_index) + ", which is itself a mimic joint");
      continue;
    }
    if (!std::isfinite(c.multiplier) || !std::isfinite(c.offset))
    {
      reject(describe(i) + " has a non-finite multiplier or offset");
      continue;
    }
  }

  if (!valid())
    return;

  reduced_index_.resize(n);
  mimicked_.assign(n, false);
  for (unsigned int i = 0; i < n; ++i)
    if (isMimic(i))
      mimicked_[couplings_[i].mimicked_index] = true;
  for (unsigned int i = 0; i < n; ++i)
    if (!isMimic(i))
    {
      reduced_index_[i] = static_cast<unsigned int>(active_.size());
      active_.push_back(i);
    }
  // Second pass, so that a mimic joint upstream of the joint it follows -- which is the crane's
  // shape -- resolves like any other.
  for (unsigned int i = 0; i < n; ++i)
    if (isMimic(i))
      reduced_index_[i] = reduced_index_[couplings_[i].mimicked_index];
}

void JointCouplings::reject(const std::string& why)
{
  if (error_.empty())
    error_ = why;
}

std::string JointCouplings::describe(unsigned int i) const
{
  std::string out = "joint " + std::to_string(i);
  if (!couplings_[i].name.empty())
    out += " (" + couplings_[i].name + ")";
  return out;
}

void JointCouplings::expand(const KDL::JntArray& reduced, KDL::JntArray& full) const
{
  // A rejected description has no active joints and no indices worth trusting, so the four
  // operations do nothing at all rather than read past the vectors they never built. TRAC_IK
  // refuses to initialise on one; a caller validating by hand can still hold one safely.
  if (!valid())
    return;

  if (full.rows() != size())
    full.resize(size());

  for (unsigned int k = 0; k < reducedSize(); ++k)
    full(active_[k]) = reduced(k);

  // Every mimicked joint is active, so its value is already in place whichever side of the chain it
  // sits on.
  for (unsigned int i = 0; i < size(); ++i)
    if (isMimic(i))
      full(i) = couplings_[i].multiplier * full(couplings_[i].mimicked_index) + couplings_[i].offset;
}

void JointCouplings::reduce(const KDL::JntArray& full, KDL::JntArray& reduced) const
{
  if (!valid())
    return;

  if (reduced.rows() != reducedSize())
    reduced.resize(reducedSize());

  for (unsigned int k = 0; k < reducedSize(); ++k)
    reduced(k) = full(active_[k]);
}

bool JointCouplings::tighten(KDL::JntArray& lb, KDL::JntArray& ub, std::string& why) const
{
  if (!valid())
  {
    why = error_;
    return false;
  }

  if (lb.rows() != size() || ub.rows() != size())
  {
    why = "joint bounds must have one entry per chain joint (" + std::to_string(size()) + ")";
    return false;
  }

  std::vector<double> lo(size()), hi(size());
  for (unsigned int i = 0; i < size(); ++i)
  {
    lo[i] = asBound(lb(i));
    hi[i] = asBound(ub(i));
  }

  // A mimic joint's own bounds, mapped back through its coupling, bound the joint it follows.
  for (unsigned int i = 0; i < size(); ++i)
  {
    if (!isMimic(i))
      continue;
    const JointCoupling& c = couplings_[i];
    const unsigned int m = c.mimicked_index;

    if (c.multiplier == 0.0)
    {
      // Pinned at the offset: it says nothing about the joint it follows, but the pin has to be a
      // value the pinned joint can actually hold, or the mechanism is a contradiction.
      if (c.offset < lo[i] - kBoundTol || c.offset > hi[i] + kBoundTol)
      {
        why = describe(i) + " is pinned at " + std::to_string(c.offset) + ", outside its own bounds";
        return false;
      }
      continue;
    }

    const std::pair<double, double> back = ordered((lo[i] - c.offset) / c.multiplier,
                                                   (hi[i] - c.offset) / c.multiplier);
    lo[m] = std::max(lo[m], back.first);
    hi[m] = std::min(hi[m], back.second);
  }

  for (unsigned int i = 0; i < size(); ++i)
    if (!isMimic(i) && lo[i] > hi[i])
    {
      why = describe(i) + " has no value that its own bounds and its mimic joints' both allow";
      return false;
    }

  // ...and, the other way round, a mimic joint can only take what its mimicked joint's effective
  // interval maps onto, which is what makes the accessor honest about a mimic joint too. The image
  // is assigned rather than intersected because the pass above already shrank the mimicked joint
  // into the preimage of this joint's own bounds, so the image cannot escape them.
  for (unsigned int i = 0; i < size(); ++i)
  {
    if (!isMimic(i))
      continue;
    const JointCoupling& c = couplings_[i];
    if (c.multiplier == 0.0)
    {
      lo[i] = hi[i] = c.offset;
      continue;
    }
    const std::pair<double, double> image = ordered(c.multiplier * lo[c.mimicked_index] + c.offset,
                                                    c.multiplier * hi[c.mimicked_index] + c.offset);
    lo[i] = image.first;
    hi[i] = image.second;
  }

  for (unsigned int i = 0; i < size(); ++i)
  {
    lb(i) = asStored(lo[i]);
    ub(i) = asStored(hi[i]);
  }
  return true;
}

void JointCouplings::foldJacobian(const KDL::Jacobian& jac, Eigen::MatrixXd& reduced) const
{
  if (!valid())
    return;

  reduced.setZero(6, reducedSize());
  for (unsigned int i = 0; i < size(); ++i)
  {
    const double scale = isMimic(i) ? couplings_[i].multiplier : 1.0;
    reduced.col(reduced_index_[i]) += scale * jac.data.col(i);
  }
}

}  // namespace TRAC_IK
