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

#ifndef TRAC_IK_JOINT_COUPLING_HPP
#define TRAC_IK_JOINT_COUPLING_HPP

#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <Eigen/Core>
#include <string>
#include <vector>

namespace TRAC_IK
{

/**
 * One chain joint's coupling, in chain order: the wire format a caller fills in.
 *
 * Plain data on purpose. It is everything a producer of the description can see without knowing
 * anything about the solver -- the plugin reads it off MoveIt's robot model, a standalone caller off
 * the URDF's <mimic> tags -- and JointCouplings is the value the solver actually talks to.
 */
struct JointCoupling
{
  /// The joint's name. Diagnostics only: a refusal names it, so a user can find the joint in their
  /// URDF. An empty name degrades the message to a chain index; it never changes what is checked.
  std::string name;

  /// Chain index of the joint this one follows, or -1 for an active joint.
  int mimicked_index = -1;

  /// The coupling: this joint's value is multiplier * (the mimicked joint) + offset. A zero
  /// multiplier is a joint pinned at offset, which is well-formed URDF and is supported.
  double multiplier = 1.0;
  double offset = 0.0;

  /// How many variables the joint has in the model it came from. One is the only case this library
  /// can index: a planar or floating joint that survived kdl_parser has more, and would silently
  /// break the correspondence between a chain index and a configuration entry. It is carried on the
  /// wire because only the producer can see it -- a KDL chain has already forgotten.
  int variable_count = 1;
};

/**
 * How a chain's joints are coupled, and the four things the solver needs to know because of it:
 * expand a reduced configuration to a full one, reduce a full one, tighten bounds into effective
 * bounds, and fold a Jacobian into the reduced Jacobian.
 *
 * Construction validates the description and never throws; a rejected description leaves the value
 * invalid, with error() naming the offending joint. Everything else here assumes a valid value.
 */
class JointCouplings
{
public:
  /// An uncoupled mechanism of `chain_joints` joints: every joint active, nothing to expand or
  /// tighten. The default is the empty one, which is what a caller who has no couplings passes.
  explicit JointCouplings(unsigned int chain_joints = 0);

  /// One entry per chain joint, in chain order.
  explicit JointCouplings(const std::vector<JointCoupling>& description);

  /// False when the description was rejected; error() then says which joint and why.
  bool valid() const
  {
    return error_.empty();
  }
  const std::string& error() const
  {
    return error_;
  }

  /// Chain joints: the size of a full configuration.
  unsigned int size() const
  {
    return static_cast<unsigned int>(couplings_.size());
  }

  /// Active joints: the size of a reduced configuration.
  unsigned int reducedSize() const
  {
    return static_cast<unsigned int>(active_.size());
  }

  /// Chain index of each active joint, ascending. Reduced index k is chain index activeIndices()[k].
  const std::vector<unsigned int>& activeIndices() const
  {
    return active_;
  }

  bool isMimic(unsigned int i) const
  {
    return couplings_[i].mimicked_index >= 0;
  }

  /// True when at least one mimic joint follows this one. Distinct from isMimic: a mimicked joint is
  /// active, and it is the joint whose value a coupling reads.
  bool isMimicked(unsigned int i) const
  {
    // Bounds-checked, so a rejected description answers "no" like every other operation here
    // answers with a no-op, rather than reading a vector it never built.
    return i < mimicked_.size() && mimicked_[i];
  }

  /// Any per-chain-joint fact, restricted to the active joints -- the reduced counterpart of a full
  /// vector. Joint type is what this library reduces through it: a joint's type and its coupling are
  /// independent facts, so the type stays where it is and is restricted here rather than folded in.
  template<typename T>
  std::vector<T> reduceVector(const std::vector<T>& full) const
  {
    std::vector<T> out;
    out.reserve(active_.size());
    for (const unsigned int i : active_)
      out.push_back(full[i]);
    return out;
  }

  const JointCoupling& coupling(unsigned int i) const
  {
    return couplings_[i];
  }

  /// Full configuration from a reduced one: active entries copied across, mimic entries computed.
  /// Resizes `full` if it has to.
  void expand(const KDL::JntArray& reduced, KDL::JntArray& full) const;

  /// Reduced configuration from a full one: the active entries, nothing else. A full configuration's
  /// mimic entries are never read, so reduce-then-expand is exactly the repair a seed needs.
  /// Resizes `reduced` if it has to.
  void reduce(const KDL::JntArray& full, KDL::JntArray& reduced) const;

  /**
   * Turn a pair of joint bounds into effective bounds, in place.
   *
   * Each mimic joint's own bounds are mapped back through its coupling and intersected into the
   * bounds of the joint it follows -- sign-aware, so a negative multiplier swaps the ends -- and
   * then each mimic joint's bounds become the image of what its mimicked joint was left with, since
   * no other value is reachable. A joint pinned by a zero multiplier contributes no bound and comes
   * out as the single point it is pinned at.
   *
   * False, with `why` naming the joint, for a mechanism no configuration satisfies: a pin outside
   * the pinned joint's own bounds, or an intersection that comes out empty.
   */
  bool tighten(KDL::JntArray& lb, KDL::JntArray& ub, std::string& why) const;

  /**
   * The reduced Jacobian: 6 x reducedSize(), each mimic joint's column folded into the column of
   * the joint it follows, scaled by the multiplier. The chain rule, and the coupled chain's actual
   * Jacobian, since no other tip velocity is commandable.
   */
  void foldJacobian(const KDL::Jacobian& jac, Eigen::MatrixXd& reduced) const;

private:
  /// Record the first refusal; later ones are noise once the value is already invalid.
  void reject(const std::string& why);

  /// "joint 4 (jib_ext_2)", or "joint 4" when the description carried no name.
  std::string describe(unsigned int i) const;

  std::vector<JointCoupling> couplings_;
  std::vector<unsigned int> active_;
  /// Whether each chain joint is followed by at least one mimic joint. All false on an uncoupled
  /// chain, and on a rejected description, where nothing may be trusted.
  std::vector<bool> mimicked_;
  /// Reduced index of each chain joint: its own for an active joint, its mimicked joint's for a
  /// mimic joint. Empty when the value is invalid.
  std::vector<unsigned int> reduced_index_;
  std::string error_;
};

/**
 * Tighten one joint's bounds by a coupling attached to a joint that is NOT in the chain, in place.
 *
 * JointCouplings::tighten folds every coupling the chain can see. A chain joint can also drive a
 * joint the chain never reaches, and only a caller that sees the whole robot -- the MoveIt plugin --
 * can find those; this is the same sign-aware mapping for that one joint at a time -- literally the
 * same, since both tighteners share the preimage this file computes -- so a negative multiplier, a
 * pinned joint and the unbounded spelling are decided here and nowhere else.
 *
 * [lb, ub] is intersected with the preimage of [mimic_lb, mimic_ub] under `multiplier * x + offset`.
 * An unbounded end is an infinity on the way in and the stored sentinel on the way out.
 *
 * False, with `why` completing a sentence about the mimic joint, for a non-finite coupling, a zero
 * multiplier that pins the mimic joint outside its own bounds, or an empty intersection.
 */
bool tightenThroughCoupling(double multiplier, double offset, double mimic_lb, double mimic_ub,
                            double& lb, double& ub, std::string& why);

}  // namespace TRAC_IK

#endif
